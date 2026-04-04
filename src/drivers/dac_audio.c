/*
 * dac_audio.c - DAC audio / CTCSS tone driver for AT32F403A on the RT-950 Pro
 *
 * DAC output fed by DMA2 channel 3, triggered by TIM6 TRGO.
 *
 * OEM V0.27 function addresses:
 *   dac_init               @ 0x0800A4F6 (18B)  - DAC peripheral init
 *   dac_enable              @ 0x0800A508 (26B)  - DAC CR enable bits
 *   dac_dma_enable          @ 0x0800A528 (26B)  - Enable DMA trigger on DAC
 *   dac_dma_timer_config    @ 0x0800A548 (36B)  - Configure trigger/DMA bits in CR
 *   tim6_dac_dma_init       @ 0x08018EC0 (174B) - Complete TIM6+DMA2_CH3+DAC setup
 *   beep_play               @ 0x08003808 (360B) - Single-tone via TIM6+DMA+DAC
 *
 * OEM TIM6 configurations (verified):
 *   Beep:  PSC=3, ARR=781  → sample rate ~38.4 kHz (~1200 Hz with 32 samples)
 *   Init:  PSC=119, ARR=125 → sample rate ~7.9 kHz (~248 Hz with 32 samples)
 *   CR2 MMS = 0x20 (update event triggers TRGO → DMA2_CH3 → DAC_DHR12R1)
 *
 * OEM hardware registers:
 *   DAC base = 0x40007400, DHR12R1 = 0x40007408
 *   TIM6 base = 0x40001000
 *   DMA2_CH3 target = 0x40007408 (DAC data reg)
 *   Waveform buffer = 0x2000E87C (16 halfwords in RAM)
 *
 * DAC config: BOFF1=1 (buffer OFF), TSEL1=0 (TIM6 TRGO trigger), DMA enabled.
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
 *                   TIM6-triggered DMA output on PA4.
 *
 *  PA4 must be set to analog mode (MODE=00, CNF=00) before enabling DAC.
 *  Per AT32/STM32 reference manual, this disables the digital input
 *  circuitry (Schmitt trigger + input buffer) to avoid parasitic loading
 *  on the DAC analog output.  OEM gpio_modes_init @ 0x0801391C
 *  configures PA4 as analog before DAC init.
 * ======================================================================== */

void dac_audio_init(void)
{
    /* Enable peripheral clocks */
    CRM->APB1EN |= CRM_APB1EN_DACEN | CRM_APB1EN_TIM6EN;
    CRM->AHBEN  |= CRM_AHBEN_DMA2EN;

    /* PA4 + PA5 = analog mode (disable digital input circuitry).
     * Required for clean DAC output per reference manual.
     * PA4 = DAC CH1 (audio signal), PA5 = DAC CH2 (amp bias/reference).
     * GPIOA clock already enabled by SystemInit. */
    {
        volatile uint32_t *crl = &((GPIO_TypeDef *)GPIOA_BASE)->CRL;
        uint32_t v = *crl;
        v &= ~(0xFFUL << 16);  /* PA4+PA5: bits [23:16] = MODE=00 CNF=00 */
        *crl = v;               /* analog mode for both DAC channels */
    }

    /* Configure DAC: EN1 deferred to play_tone(), but EN2 enabled NOW.
     * OEM dac_init @ 0x0800A4F6 enables EN2 at init (never disables it).
     * OEM does NOT set BOFF2, so CH2 output buffer stays ON (low-Z ~1Ω).
     * PA5 (CH2) likely provides DC bias/reference for the audio amplifier.
     * Without CH2 enabled, the amp may not function. */
    DAC->CR = DAC_CR_EN2 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6;

    /* Set CH2 output to mid-scale (1.65V) as amp bias reference */
    DAC->DHR12R2 = 2048;

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

    /* Clear DMA underrun flag (DAC_SR offset 0x34, bit 13 = DMAUDR1).
     * This flag latches if a TRGO arrived while DMA was not ready and
     * permanently blocks further DMA requests until cleared. */
    *(volatile uint32_t *)(DAC_BASE + 0x34U) = (1UL << 13);

    /* Calculate TIM6 auto-reload value */
    uint32_t divisor = (uint32_t)freq_hz_x10 * SINE_TABLE_SAMPLES;
    uint32_t arr = (TIM6_CLOCK_HZ * 10UL + divisor / 2) / divisor - 1;

    /* Configure TIM6 timing */
    TIM6->PSC = 0;
    TIM6->ARR = arr;

    /* Start circular DMA from sine table to DAC.
     * MUST be before EGR=UG, because UG generates a TRGO via MMS=Update
     * which triggers a DMA request - DMA must be ready to service it. */
    dma2_ch3_setup(sine_table, SINE_TABLE_SAMPLES, 1);

    /* Enable DAC channel 1 + DMA - preserve EN2 for amp bias.
     * Matches OEM beep_play sequence: DMA config → dac_enable → dac_dma_enable.
     * OEM beep_play @ 0x08003808. */
    DAC->CR = DAC_CR_EN2 | DAC_CR_EN1 | DAC_CR_BOFF1 | DAC_CR_TEN1
            | DAC_CR_TSEL1_TIM6 | DAC_CR_DMAEN1;

    /* Force update to load PSC/ARR shadow registers */
    TIM6->EGR = TIM_EGR_UG;

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

    /* Clear DMAUDR1 in case it got set */
    *(volatile uint32_t *)(DAC_BASE + 0x34U) = (1UL << 13);

    /* Disable DAC CH1 DMA and channel - keep EN2 for amp bias */
    DAC->CR = DAC_CR_EN2 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6;

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
