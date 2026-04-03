/*
 * dac_audio.c - DAC audio / CTCSS tone driver for AT32F403A on the RT-950 Pro
 *
 * DAC output fed by DMA2 channel 3, triggered by TIM6 TRGO.
 *
 * V0.27 binary evidence:
 *   - DMA2_CH3 ISR active at fw 0x0800D1D5 (vector table entry)
 *   - DAC SDK init function at fw 0x0800A548 (config struct pattern)
 *   - DAC CR bit-set/clear functions at fw 0x0800A508
 *   - TIM6 base literal pool ref at fw 0x0800697C (CRM clock enable area)
 *   - TIM6 ref at fw 0x0801BF70 (audio/radio control area)
 *   - No TIM6 ISR used: vector entry points to default handler at fw 0x080032BB
 *   - OEM may use DAC channel 2 (bit 16 = EN2 in DAC_CR) - needs further analysis
 *
 * TIM6 clock = 120 MHz (APB1 bus = 60 MHz, timer clock doubled because
 * APB1 prescaler > 1).
 */

#include "drivers/dac_audio.h"

/* TIM6 timer instance ------------------------------------------------ */
#define TIM6    ((TIM_TypeDef *)TIM6_BASE)

/* TIM6 clock frequency ----------------------------------------------- */
#define TIM6_CLOCK_HZ   120000000UL

/* TIM CR1 / CR2 bits used for basic timer ---------------------------- */
#define TIM_CR1_CEN     (1UL << 0)      /* Counter enable */
#define TIM_CR2_MMS_UPDATE (2UL << 4)   /* MMS = 010: Update event as TRGO */

/* TIM EGR bits ------------------------------------------------------- */
#define TIM_EGR_UG      (1UL << 0)      /* Update generation */

/* DMA Channel Configuration Register (CCR) bits ---------------------- */
#define DMA_CCR_EN      (1UL << 0)      /* Channel enable */
#define DMA_CCR_DIR     (1UL << 4)      /* Direction: 1 = read from memory */
#define DMA_CCR_CIRC    (1UL << 5)      /* Circular mode */
#define DMA_CCR_MINC    (1UL << 7)      /* Memory increment mode */
#define DMA_CCR_PSIZE_16 (1UL << 8)     /* Peripheral size: 16-bit */
#define DMA_CCR_MSIZE_16 (1UL << 10)    /* Memory size: 16-bit */
#define DMA_CCR_PL_HIGH (2UL << 12)     /* Priority: high */

/* DMA2 ISR/IFCR bits for channel 3 (flags at bits 8-11) */
#define DMA2_IFCR_CGIF3 (1UL << 8)     /* Clear channel 3 global flag */

/*
 * 32-sample sine lookup table, 12-bit values (0-4095), centered at 2048.
 * Values: 2048 + 2047 * sin(2pi * i / 32), rounded to nearest integer.
 */
static const uint16_t sine_table[SINE_TABLE_SAMPLES] = {
    2048, 2448, 2831, 3185, 3496, 3750, 3939, 4056,
    4095, 4056, 3939, 3750, 3496, 3185, 2831, 2448,
    2048, 1648, 1265,  911,  600,  346,  157,   40,
       1,   40,  157,  346,  600,  911, 1265, 1648
};

/* Internal helper: configure DMA2 CH3 for DAC1 transfer -------------- */

static void dma2_ch3_setup(const uint16_t *src, uint16_t count, int circular)
{
    DMA_Channel_TypeDef *ch = DMA2_CH(3);

    /* Disable channel before reconfiguring */
    ch->CCR = 0;

    /* Clear all channel 3 interrupt flags */
    DMA2->IFCR = DMA2_IFCR_CGIF3;

    /* Number of data items to transfer */
    ch->CNDTR = count;

    /* Peripheral address: DAC channel 1, 12-bit right-aligned data */
    ch->CPAR = (uint32_t)&DAC->DHR12R1;

    /* Memory address: source buffer */
    ch->CMAR = (uint32_t)src;

    /* Configure: mem-to-periph, mem-increment, 16-bit, high priority */
    uint32_t ccr = DMA_CCR_DIR | DMA_CCR_MINC
                 | DMA_CCR_PSIZE_16 | DMA_CCR_MSIZE_16
                 | DMA_CCR_PL_HIGH;

    if (circular) {
        ccr |= DMA_CCR_CIRC;
    }

    /* Enable channel */
    ch->CCR = ccr | DMA_CCR_EN;
}

/* ========================================================================
 *  dac_audio_init - Enable DAC/TIM6/DMA2 clocks, configure DAC1 for
 *                   TIM6-triggered DMA output.
 *
 *  PA4 does not need explicit GPIO configuration - the DAC automatically
 *  overrides the pin when DAC channel 1 is enabled.
 * ======================================================================== */

void dac_audio_init(void)
{
    /* Enable peripheral clocks */
    CRM->APB1EN |= CRM_APB1EN_DACEN | CRM_APB1EN_TIM6EN;
    CRM->AHBEN  |= CRM_AHBEN_DMA2EN;

    /* Configure DAC channel 1:
     *   - Trigger select = TIM6 TRGO (TSEL1 = 000)
     *   - Trigger enable
     *   - DMA enable
     *   - Output buffer enabled (BOFF1 = 0)
     *   - Channel enabled
     */
    DAC->CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6 | DAC_CR_DMAEN1;

    /* TIM6 will be configured per-tone in dac_audio_play_tone() */
    TIM6->CR1 = 0;
    TIM6->CR2 = TIM_CR2_MMS_UPDATE;    /* Update event -> TRGO */
    TIM6->PSC = 0;                      /* No prescaler */
    TIM6->ARR = 0xFFFF;                 /* Default (overridden per tone) */
}

/* ========================================================================
 *  dac_audio_play_tone - Generate continuous CTCSS sine tone via DMA.
 *
 *  For a tone at freq_hz_x10 (e.g., 670 = 67.0 Hz):
 *    actual_freq   = freq_hz_x10 / 10
 *    sample_rate   = actual_freq x 32 samples/cycle
 *    TIM6 ARR      = (TIM6_CLOCK / sample_rate) - 1
 *                  = (TIM6_CLOCK x 10) / (freq_hz_x10 x 32) - 1
 * ======================================================================== */

void dac_audio_play_tone(uint16_t freq_hz_x10)
{
    if (freq_hz_x10 == 0) {
        dac_audio_stop();
        return;
    }

    /* Stop any current playback */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA2_CH(3)->CCR = 0;

    /* Calculate TIM6 auto-reload value */
    uint32_t divisor = (uint32_t)freq_hz_x10 * SINE_TABLE_SAMPLES;
    uint32_t arr = (TIM6_CLOCK_HZ * 10UL + divisor / 2) / divisor - 1;

    /* Configure TIM6 timing */
    TIM6->PSC = 0;
    TIM6->ARR = arr;
    TIM6->EGR = TIM_EGR_UG;            /* Force update to load PSC/ARR */

    /* Start circular DMA from sine table to DAC */
    dma2_ch3_setup(sine_table, SINE_TABLE_SAMPLES, 1);

    /* Start TIM6 */
    TIM6->CR1 = TIM_CR1_CEN;
}

/* ========================================================================
 *  dac_audio_stop - Stop tone/audio playback.
 * ======================================================================== */

void dac_audio_stop(void)
{
    /* Stop TIM6 */
    TIM6->CR1 &= ~TIM_CR1_CEN;

    /* Disable DMA2 channel 3 */
    DMA2_CH(3)->CCR = 0;

    /* Clear DMA flags */
    DMA2->IFCR = DMA2_IFCR_CGIF3;

    /* Set DAC output to mid-scale (silence) */
    DAC->DHR12R1 = 2048;
}

/* ========================================================================
 *  dac_audio_play_buffer - Play arbitrary waveform via DMA.
 *
 *  Caller must configure TIM6 timing beforehand (or call after
 *  dac_audio_play_tone which sets up TIM6).  If the caller wants a
 *  specific sample rate, set TIM6->ARR = (TIM6_CLOCK / rate) - 1.
 * ======================================================================== */

void dac_audio_play_buffer(const uint16_t *samples, uint16_t count,
                           int circular)
{
    /* Stop current transfer */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA2_CH(3)->CCR = 0;

    /* Arm DMA with the provided buffer */
    dma2_ch3_setup(samples, count, circular);

    /* Start TIM6 */
    TIM6->CR1 = TIM_CR1_CEN;
}

/* ========================================================================
 *  dac_audio_is_playing - Returns non-zero if DMA2 CH3 is still enabled.
 * ======================================================================== */

int dac_audio_is_playing(void)
{
    return (DMA2_CH(3)->CCR & DMA_CCR_EN) ? 1 : 0;
}
