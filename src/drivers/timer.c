/*
 * timer.c - Timer ISR architecture for the RT-950 Pro
 *
 * TIM2 @ 1200 Hz - CTCSS tone detection/generation timing
 * TIM3 @ 1000 Hz - Fast system tick (VOX sampling, scanner dwell, etc.)
 *
 * NOTE: The OEM V0.27 firmware does NOT use any timer interrupts.  All
 * timer vector entries (TIM1-TIM7) point to the default handler at fw
 * 0x080032BB (b . - infinite loop).  The OEM likely handles CTCSS via
 * BK4829 hardware registers and uses SysTick for all periodic timing.
 *
 * Our ISR-based timer architecture is a custom design choice providing
 * cleaner callback separation.  It is functionally correct but does not
 * match OEM structure.
 *
 * Both timers use APB1 timer clock at 120 MHz (APB1 bus = 60 MHz, doubled
 * when prescaler > 1) with PSC=119 -> 1 MHz counter tick.
 * ISR handlers dispatch to registered callbacks (max 4 each).
 */

#include "drivers/timer.h"
#include "cortex_m4.h"

/* TIM CR1 / DIER / SR / EGR bits ----------------------------------- */
#define TIM_CR1_CEN     (1UL << 0)
#define TIM_DIER_UIE    (1UL << 0)      /* Update interrupt enable */
#define TIM_SR_UIF      (1UL << 0)      /* Update interrupt flag */
#define TIM_EGR_UG      (1UL << 0)      /* Force update generation */

/* Callback tables --------------------------------------------------- */

static timer_callback_t ctcss_cbs[TIMER_MAX_CALLBACKS];
static uint8_t          ctcss_cb_count;

static timer_callback_t fast_cbs[TIMER_MAX_CALLBACKS];
static uint8_t          fast_cb_count;

static volatile uint32_t fast_tick_counter;

/* ISR: TIM2 - CTCSS tick @ 1200 Hz --------------------------------- */

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;        /* Clear flag (rc_w0) */

        for (uint8_t i = 0; i < ctcss_cb_count; i++)
            ctcss_cbs[i]();
    }
}

/* ISR: TIM3 - Fast tick @ 1000 Hz ---------------------------------- */

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;

        fast_tick_counter++;

        for (uint8_t i = 0; i < fast_cb_count; i++)
            fast_cbs[i]();
    }
}

/* Public API -------------------------------------------------------- */

void timer_init(void)
{
    /* Enable APB1 clocks for TIM2 and TIM3 */
    CRM->APB1EN |= CRM_APB1EN_TIM2EN | CRM_APB1EN_TIM3EN;

    /* TIM2: CTCSS @ 1200 Hz --------------------------------------- */
    TIM2->CR1 = 0;                      /* Stop */
    TIM2->PSC = TIM2_PSC;
    TIM2->ARR = TIM2_ARR;
    TIM2->CNT = 0;
    TIM2->EGR = TIM_EGR_UG;            /* Load shadow registers */
    TIM2->SR  = 0;                      /* Clear all flags */
    TIM2->DIER = TIM_DIER_UIE;         /* Enable update IRQ */

    NVIC_SetPriority(TIM2_IRQn, 3);    /* Medium-low priority */
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 = TIM_CR1_CEN;           /* Start */

    /* TIM3: Fast tick @ 1000 Hz ----------------------------------- */
    TIM3->CR1 = 0;
    TIM3->PSC = TIM3_PSC;
    TIM3->ARR = TIM3_ARR;
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR  = 0;
    TIM3->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 4);    /* Lower priority than CTCSS */
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 = TIM_CR1_CEN;

    /* Init callback tables */
    ctcss_cb_count = 0;
    fast_cb_count = 0;
    fast_tick_counter = 0;
}

int timer_register_ctcss_cb(timer_callback_t cb)
{
    if (ctcss_cb_count >= TIMER_MAX_CALLBACKS || !cb) return -1;
    ctcss_cbs[ctcss_cb_count++] = cb;
    return 0;
}

int timer_register_fast_cb(timer_callback_t cb)
{
    if (fast_cb_count >= TIMER_MAX_CALLBACKS || !cb) return -1;
    fast_cbs[fast_cb_count++] = cb;
    return 0;
}

uint32_t timer_get_fast_tick(void)
{
    return fast_tick_counter;
}
