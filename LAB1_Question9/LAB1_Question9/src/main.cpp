/*
 * ECNG 3006 - Data Acquisition: 1 kHz ADC sampling, jitter, SNR/ENOB, PMF
 * Platform   : Arduino MKR Zero (ATSAMD21G18A)
 * Student    : Nicholas Ramnarine
 * Student ID : 816041866
 *
 * Compiler   : PlatformIO ARM GCC toolchain (C-style code built as C++)
 * Sample tick: TC3 hardware timer interrupt, 1 ms  -> 1000 Hz sampling rate
 * Timestamps : TC4+TC5 free-running 32-bit counter at 48 MHz (20.8 ns/count)
 * Serial     : SERCOM5 (PB22 TX / PB23 RX), 115200 baud (output only)
 *
 * No floating point is used anywhere (all maths is integer / fixed point).
 *
 * ANALOG CIRCUIT (Option 2 - resistor divider / potentiometer)
 *   3V3 ---[ R_TOP = 10k ]---+---[ R2 = unknown / pot ]--- GND
 *                            |
 *                            +--> A0 (PA02 = ADC AIN[0])
 *   Power the divider from the MKR Zero 3V3 pin: the ADC uses VDDANA as its
 *   reference, so the reading is ratiometric and R2 does not need Vref.
 *
 * STRATEGY
 * 1. TC3 fires every 1 ms. Its ISR timestamps the sample, starts one ADC
 *    conversion and BUSY-WAITS on RESRDY, then stores the result in DataBuf
 *    and the timestamp in TimeBuf (1000 samples each).
 * 2. Hardware averaging (ADC->AVGCTRL.SAMPLENUM) is swept from 1 to 64
 *    samples per result. Each setting captures a fresh 1000-sample record.
 *    Larger settings make the busy-wait longer; the deadline check shows
 *    whether every sample still met its 1 ms slot (hard real-time).
 * 3. From TimeBuf: maxT, minT, jitter = maxT - minT, missed deadlines.
 * 4. From DataBuf: mean (Signal), standard deviation (Noise),
 *    SNR = mean / std, ENOB = log2(SNR), all in integer / fixed point.
 * 5. A probability mass function (PMF) of DataBuf is printed so the effect of
 *    averaging on the noise distribution (Central Limit Theorem) is visible.
 * 6. Optional (CRITICAL_SECTION_DEMO): ISR toggles pin A three times with a
 *    non-atomic XOR, the main loop toggles pin B with the same non-atomic
 *    XOR on the same OUT register -> a read-modify-write race.
 */

#include <sam.h>          /* SAMD21 register definitions and CMSIS functions. */
#include <stdint.h>       /* Fixed-width integer types.                       */
#include <stdio.h>        /* printf() and standard-output functions.          */
#include <errno.h>        /* errno codes for the low-level I/O stubs.         */
#include <sys/stat.h>     /* struct stat and S_IFCHR.                         */
#include <sys/types.h>    /* off_t for _lseek().                              */

/* ---------------------- Configuration constants ---------------------- */

#define CPU_CLOCK_HZ          (48000000UL)  /* GCLK0 frequency: 48 MHz.              */
#define SAMPLE_RATE_HZ        (1000UL)      /* Required sampling rate.               */
#define NUM_SAMPLES           (1000U)       /* Samples per record.                   */

#define TC3_PRESCALER         (64UL)        /* TC3 clock divider.                    */
#define TC3_COUNTS_PER_SAMPLE (CPU_CLOCK_HZ / TC3_PRESCALER / SAMPLE_RATE_HZ) /* 750 */

/* Timestamp counter runs at CPU_CLOCK_HZ, so one sample period is 48000 counts. */
#define NOMINAL_PERIOD_COUNTS (CPU_CLOCK_HZ / SAMPLE_RATE_HZ)
#define MISSED_DEADLINE_COUNTS (NOMINAL_PERIOD_COUNTS + (NOMINAL_PERIOD_COUNTS / 2UL))
#define COUNTS_TO_NS(c)       ((uint32_t)(((uint64_t)(c) * 125ULL) / 6ULL)) /* 1 count = 20.833 ns */

#define UART_BAUD_REG_VALUE   (63019U)      /* SERCOM5 BAUD value: 115200 @ 48 MHz.  */

#define ADC_SAMPLEN_HALF_CYC  (4U)          /* Sampling time = (4+1)/2 ADC clocks.   */
#define ADC_FULL_SCALE        (4095UL)      /* 12-bit full scale for ONE conversion. */
#define ADC_MAX_SETTINGS      (7U)          /* Averaging settings in the sweep.      */

#define R_TOP_OHMS            (10000UL)     /* Fixed top resistor of the divider.    */
#define VCC_MV                (3313UL)      /* Nominal VDDANA in millivolts.         */

#define PMF_BINS              (64U)         /* Maximum PMF rows printed.             */
#define PMF_BAR_WIDTH         (40U)         /* Width of the text bar, characters.    */
#define START_DELAY_MS        (5000UL)      /* Time to open the terminal after reset. */

/*
 * Critical-section demonstration (item 8):
 *   0 = off, 1 = BUGGY non-atomic XOR (the lab requirement),
 *   2 = FIXED, single atomic write to OUTTGL.
 * Pin A = D2 (PA10), pin B = D3 (PA11). Both are in PORT group 0, so both
 * share the SAME OUT register, which is what makes the race possible.
 */
#define CRITICAL_SECTION_DEMO (2)
#define PIN_A_MASK            (PORT_PA10)
#define PIN_B_MASK            (PORT_PA11)

#if CRITICAL_SECTION_DEMO == 1
#define TOGGLE_PIN(mask)      (PORT->Group[0].OUT.reg ^= (mask))   /* read, modify, write */
#elif CRITICAL_SECTION_DEMO == 2
#define TOGGLE_PIN(mask)      (PORT->Group[0].OUTTGL.reg = (mask)) /* one atomic write    */
#endif

/* ------------------------------- Data -------------------------------- */

typedef struct
{
    uint32_t averages;      /* Hardware samples averaged per result.       */
    uint32_t max_delta;     /* maxT: largest sample-to-sample time.        */
    uint32_t min_delta;     /* minT: smallest sample-to-sample time.       */
    uint32_t jitter;        /* maxT - minT, in timestamp counts.           */
    uint32_t missed;        /* Samples that missed their 1 ms deadline.    */
    uint32_t max_conv;      /* Worst-case ADC conversion time, counts.     */
    uint32_t mean_x100;     /* Mean (Signal) x 100, in result units.       */
    uint32_t std_x100;      /* Standard deviation (Noise) x 100.           */
    uint32_t snr_q8;        /* SNR = mean / std, fixed point (x256).       */
    int32_t  enob_q8;       /* ENOB = log2(SNR), fixed point (x256).       */
    uint32_t scale;         /* Result scale relative to one 12-bit reading. */
} result_t;

static uint16_t DataBuf[NUM_SAMPLES];  /* ADC results, written by the ISR. */
static uint32_t TimeBuf[NUM_SAMPLES];  /* Timestamps (48 MHz counts).      */

static volatile uint32_t g_tick_count = 0U;       /* 1 ms ticks since start.        */
static volatile uint32_t g_sample_index = 0U;     /* Next DataBuf/TimeBuf slot.     */
static volatile bool     g_capture_active = false;
static volatile bool     g_capture_done = false;
static volatile uint32_t g_max_conv_counts = 0U;  /* Worst ADC busy-wait, counts.   */

static result_t g_results[ADC_MAX_SETTINGS];
static bool g_sweep_done = false;

/* SAMPLENUM values: 2^n samples averaged. 0..6 = 1..64 samples. */
static const uint8_t g_avg_log2_table[ADC_MAX_SETTINGS] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U };

/* ============================ UART setup ============================= */

/* SERCOM5 is polled; the variant already owns SERCOM5_Handler() for Serial1. */

static void uart_init(void)
{
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3U) | PORT_PMUX_PMUXO(3U);

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST != 0U) { /* Wait while reset synchronizes. */ }
    while (SERCOM5->USART.CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1U) |
                               SERCOM_USART_CTRLA_RXPO(3U) | SERCOM_USART_CTRLA_TXPO(1U);
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB != 0U) { /* Wait for CTRLB. */ }

    SERCOM5->USART.BAUD.reg = UART_BAUD_REG_VALUE;
    SERCOM5->USART.CTRLA.bit.ENABLE = 1U;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE != 0U) { /* Wait for enable. */ }
}

static void uart_putc(char character)
{
    while (SERCOM5->USART.INTFLAG.bit.DRE == 0U) { /* Wait for an empty data register. */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)character;
}

/* ------------------ Redirect C standard output to UART ---------------- */

extern "C" int _write(int file, char *buffer, int length) /* Used by printf(). */
{
    if ((file != 1) && (file != 2)) { errno = EBADF; return -1; }
    if ((buffer == NULL) || (length < 0)) { errno = EINVAL; return -1; }

    for (int index = 0; index < length; ++index) { uart_putc(buffer[index]); }

    while (SERCOM5->USART.INTFLAG.bit.TXC == 0U) { /* Wait for the last character. */ }
    return length;
}

extern "C" int _close(int file) { (void)file; errno = EBADF; return -1; }

extern "C" int _fstat(int file, struct stat *status)
{
    (void)file;
    if (status == NULL) { errno = EINVAL; return -1; }
    status->st_mode = S_IFCHR;
    return 0;
}

extern "C" int _isatty(int file) { return ((file == 1) || (file == 2)) ? 1 : 0; }

extern "C" off_t _lseek(int file, off_t offset, int whence)
{
    (void)file; (void)offset; (void)whence;
    errno = ESPIPE;
    return (off_t)-1;
}

extern "C" int _read(int file, char *buffer, int length)
{
    (void)file; (void)buffer; (void)length;
    return 0;
}

extern "C" void _exit(int status)
{
    (void)status;
    for (;;) { __WFI(); }
}

/* ============================== ADC setup ============================ */

/*
 * The SAMD21 ADC supports hardware sample averaging: AVGCTRL.SAMPLENUM = n
 * accumulates 2^n conversions (1..1024) per result, and needs
 * CTRLB.RESSEL = 16BIT. With ADJRES = 0 no extra division is applied, so the
 * extra bits are kept: the result is the sum of the 2^n conversions for up to
 * 16 samples (16-bit register) and is automatically right-shifted beyond
 * that. The result is therefore (one 12-bit reading) x adc_result_scale().
 */

static inline void adc_wait_sync(void)
{
    while (ADC->STATUS.bit.SYNCBUSY != 0U) { /* Wait for ADC register synchronization. */ }
}

static inline uint32_t adc_result_scale(uint8_t log2n)
{
    return (log2n >= 4U) ? 16UL : (1UL << log2n);
}

/* One conversion (or one full hardware-averaged set), BUSY-WAIT on RESRDY. */
static inline uint16_t adc_convert_blocking(void)
{
    adc_wait_sync();
    ADC->SWTRIG.reg = ADC_SWTRIG_START;                   /* Start conversion. */
    while (ADC->INTFLAG.bit.RESRDY == 0U) { /* Busy-wait until the result is ready. */ }
    return ADC->RESULT.reg;                               /* Read also clears RESRDY. */
}

static void adc_set_averaging(uint8_t log2n)
{
    ADC->CTRLA.bit.ENABLE = 0U;                           /* Settings change while disabled. */
    adc_wait_sync();
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM(log2n) | ADC_AVGCTRL_ADJRES(0U);
    ADC->CTRLA.bit.ENABLE = 1U;
    adc_wait_sync();
    (void)adc_convert_blocking();                         /* Discard the first conversion. */
}

static void adc_init(void)
{
    uint16_t calibration;

    PORT->Group[0].PINCFG[2].reg |= PORT_PINCFG_PMUXEN;   /* A0 = PA02 to peripheral mux. */
    PORT->Group[0].PMUX[1].bit.PMUXE = 0x1U;              /* Function B = analog (ADC).   */

    PM->APBCMASK.reg |= PM_APBCMASK_ADC;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_ADC;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    calibration = ADC->CALIB.reg;                         /* Keep factory calibration across reset. */
    ADC->CTRLA.reg = ADC_CTRLA_SWRST;
    adc_wait_sync();
    while (ADC->CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }
    ADC->CALIB.reg = calibration;

    ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;        /* Ref = VDDANA/2 ...             */
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 |        /* ... with gain 1/2: 0..VDDANA.  */
                         ADC_INPUTCTRL_MUXNEG_GND |
                         ADC_INPUTCTRL_MUXPOS_PIN0;       /* Single-ended input AIN[0].     */
    adc_wait_sync();
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |          /* 48 MHz / 32 = 1.5 MHz ADC clock */
                     ADC_CTRLB_RESSEL_16BIT;              /* 16-bit result (for averaging).  */
    adc_wait_sync();
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(ADC_SAMPLEN_HALF_CYC);

    adc_set_averaging(0U);                                /* Enable ADC, no averaging.       */
}

/* ===================== Timestamp counter (TC4 + TC5) ================= */

/* TC4 (master) + TC5 (slave) form a free-running 32-bit counter at 48 MHz. */
static void tc4_init_timestamp(void)
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC4 | PM_APBCMASK_TC5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY != 0U) { /* Wait for reset. */ }
    while (TC4->COUNT32.CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }

    TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_NFRQ | TC_CTRLA_PRESCALER_DIV1;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY != 0U) { /* Wait for CTRLA. */ }

    /* Continuous read-synchronisation of COUNT (register address 0x10). */
    TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10U);

    TC4->COUNT32.CTRLA.bit.ENABLE = 1U;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY != 0U) { /* Wait for enable. */ }
}

/* ===================== Sampling timer (TC3, 1 kHz) =================== */

/*
 * The ISR is the data-acquisition task. TC3 has the HIGHEST priority so the
 * framework's SysTick handler cannot delay the start of a sample.
 */
extern "C" void TC3_Handler(void)
{
    if (TC3->COUNT16.INTFLAG.bit.MC0 != 0U)
    {
        const uint32_t t_start = TC4->COUNT32.COUNT.reg;  /* Timestamp first: measures entry jitter. */

        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;        /* Write-1-to-clear. */
        ++g_tick_count;

#if CRITICAL_SECTION_DEMO != 0
        TOGGLE_PIN(PIN_A_MASK);                           /* Pin A toggled three times per ISR. */
        TOGGLE_PIN(PIN_A_MASK);
        TOGGLE_PIN(PIN_A_MASK);
#endif

        if (g_capture_active)
        {
            uint32_t index = g_sample_index;
            const uint16_t result = adc_convert_blocking();      /* Busy-wait conversion. */
            const uint32_t conv = TC4->COUNT32.COUNT.reg - t_start;

            TimeBuf[index] = t_start;
            DataBuf[index] = result;
            if (conv > g_max_conv_counts) { g_max_conv_counts = conv; }

            ++index;
            __DMB();                                      /* Buffers are written before the flags change. */
            g_sample_index = index;
            if (index >= NUM_SAMPLES)
            {
                g_capture_active = false;
                g_capture_done = true;
            }
        }
    }
}

static void tc3_init_1khz_tick(void)
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for reset. */ }
    while (TC3->COUNT16.CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV64;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for CTRLA. */ }

    TC3->COUNT16.CC[0].reg = (uint16_t)(TC3_COUNTS_PER_SAMPLE - 1UL); /* CC0 = 749 -> 1 ms. */
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for CC0. */ }

    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 0U);                       /* Highest priority. */
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.bit.ENABLE = 1U;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for enable. */ }
}

static void wait_ms(uint32_t duration_ms)
{
    const uint32_t start = g_tick_count;
    while ((g_tick_count - start) < duration_ms) { __WFI(); }
}

/* ===================== Integer / fixed-point maths =================== */

/* Integer square root of a 64-bit value (bitwise method, no floating point). */
static uint32_t isqrt_u64(uint64_t value)
{
    uint64_t result = 0ULL;
    uint64_t bit = 1ULL << 62;

    while (bit > value) { bit >>= 2; }
    while (bit != 0ULL)
    {
        if (value >= (result + bit))
        {
            value -= (result + bit);
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

/*
 * Fixed-point base-2 logarithm. Input x is a Q8 number (value x 256, x > 0).
 * Returns log2(x / 256) as a signed Q8 number (result x 256).
 * Integer part from the most significant bit; 8 fractional bits by repeated
 * squaring of the normalised mantissa.
 */
static int32_t fixed_log2_q8(uint32_t x_q8)
{
    const int32_t msb = 31 - __builtin_clz(x_q8);
    int32_t result = (msb - 8) * 256;
    uint32_t y = (msb >= 15) ? (x_q8 >> (msb - 15)) : (x_q8 << (15 - msb)); /* Q15 in [1,2). */

    for (int32_t bit = 0; bit < 8; ++bit)
    {
        y = (y * y) >> 15;                                /* Square the mantissa.         */
        if (y >= (2UL << 15))                             /* Square reached 2.0 or more:  */
        {
            result += (128 >> bit);                       /* this fractional bit is 1,    */
            y >>= 1;                                      /* renormalise to [1,2).        */
        }
    }
    return result;
}

static void print_q8(int32_t value_q8)                    /* Prints a Q8 value as x.yyy. */
{
    uint32_t magnitude;

    if (value_q8 < 0) { printf("-"); magnitude = (uint32_t)(-value_q8); }
    else              { magnitude = (uint32_t)value_q8; }

    printf("%lu.%03lu", (unsigned long)(magnitude >> 8),
           (unsigned long)(((magnitude & 0xFFUL) * 1000UL) / 256UL));
}

/* ========================= Analysis of one record ==================== */

static void analyse_timing(result_t *r)
{
    uint32_t max_delta = 0U;
    uint32_t min_delta = 0xFFFFFFFFUL;
    uint32_t missed = 0U;

    for (uint32_t i = 1U; i < NUM_SAMPLES; ++i)
    {
        const uint32_t delta = TimeBuf[i] - TimeBuf[i - 1U]; /* Unsigned: safe across wrap. */
        if (delta > max_delta) { max_delta = delta; }
        if (delta < min_delta) { min_delta = delta; }
        if (delta > MISSED_DEADLINE_COUNTS) { ++missed; }
    }

    r->max_delta = max_delta;
    r->min_delta = min_delta;
    r->jitter = max_delta - min_delta;                    /* Jitter = maxT - minT. */
    r->missed = missed;
    r->max_conv = g_max_conv_counts;
}

static void analyse_data(result_t *r)
{
    const uint64_t n = NUM_SAMPLES;
    uint64_t sum = 0ULL;
    uint64_t sum_sq = 0ULL;

    for (uint32_t i = 0U; i < NUM_SAMPLES; ++i)
    {
        const uint64_t v = DataBuf[i];
        sum += v;
        sum_sq += v * v;
    }

    r->mean_x100 = (uint32_t)((sum * 100ULL) / n);        /* Signal = mean.               */

    /* N^2 x variance = N*sum(x^2) - (sum x)^2 ; scale by 10^4 so sqrt gives std x 100. */
    const uint64_t n2_var = (n * sum_sq) - (sum * sum);
    const uint64_t var_x10000 = (n2_var * 10000ULL) / (n * n);
    r->std_x100 = isqrt_u64(var_x10000);                  /* Noise = standard deviation.  */

    if ((r->std_x100 > 0U) && (r->mean_x100 > 0U))
    {
        r->snr_q8 = (uint32_t)(((uint64_t)r->mean_x100 << 8) / r->std_x100); /* SNR = mean / std. */
        r->enob_q8 = (r->snr_q8 > 0U) ? fixed_log2_q8(r->snr_q8) : 0;        /* ENOB = log2(SNR). */
    }
    else
    {
        r->snr_q8 = 0U;                                   /* Undefined: no measurable noise. */
        r->enob_q8 = 0;
    }
}

/* ============================== Reporting ============================ */

static void print_timing(const result_t *r)
{
    printf("Timing  : maxT = %lu counts (%lu ns), minT = %lu counts (%lu ns)\r\n",
           (unsigned long)r->max_delta, (unsigned long)COUNTS_TO_NS(r->max_delta),
           (unsigned long)r->min_delta, (unsigned long)COUNTS_TO_NS(r->min_delta));
    printf("          jitter = maxT - minT = %lu counts (%lu ns)\r\n",
           (unsigned long)r->jitter, (unsigned long)COUNTS_TO_NS(r->jitter));
    printf("          worst-case ADC busy-wait = %lu ns, missed deadlines = %lu -> %s\r\n",
           (unsigned long)COUNTS_TO_NS(r->max_conv), (unsigned long)r->missed,
           (r->missed == 0U) ? "all 1000 samples collected on time" : "HARD DEADLINE VIOLATED");
}

static void print_statistics(const result_t *r)
{
    const uint32_t full_scale_x100 = ADC_FULL_SCALE * r->scale * 100UL;

    printf("Signal  : mean = %lu.%02lu (result units, full scale %lu)\r\n",
           (unsigned long)(r->mean_x100 / 100UL), (unsigned long)(r->mean_x100 % 100UL),
           (unsigned long)(ADC_FULL_SCALE * r->scale));
    printf("Noise   : std  = %lu.%02lu\r\n",
           (unsigned long)(r->std_x100 / 100UL), (unsigned long)(r->std_x100 % 100UL));

    if (r->snr_q8 > 0U)
    {
        printf("SNR     : "); print_q8((int32_t)r->snr_q8);
        printf("    ENOB = log2(SNR) = "); print_q8(r->enob_q8); printf(" bits\r\n");
    }
    else
    {
        printf("SNR     : undefined (noise below 0.01 result unit, or zero signal)\r\n");
    }

    if (r->mean_x100 < full_scale_x100)
    {
        const uint32_t millivolts = (uint32_t)(((uint64_t)r->mean_x100 * VCC_MV) / full_scale_x100);
        const uint32_t r2_ohms = (uint32_t)(((uint64_t)R_TOP_OHMS * r->mean_x100) /
                                            (full_scale_x100 - r->mean_x100));
        printf("Physical: Vout = %lu mV, R2 = R_TOP x mean / (FS - mean) = %lu ohms\r\n",
               (unsigned long)millivolts, (unsigned long)r2_ohms);
    }
}

/* Probability mass function: number of observations versus observed value. */
static void print_pmf(void)
{
    uint16_t pmf[PMF_BINS];
    char bar[PMF_BAR_WIDTH + 1U];
    uint16_t lo = 0xFFFFU;
    uint16_t hi = 0U;
    uint16_t peak = 0U;

    for (uint32_t i = 0U; i < NUM_SAMPLES; ++i)
    {
        if (DataBuf[i] < lo) { lo = DataBuf[i]; }
        if (DataBuf[i] > hi) { hi = DataBuf[i]; }
    }

    const uint32_t range = (uint32_t)hi - lo + 1U;
    const uint32_t width = (range + PMF_BINS - 1U) / PMF_BINS;   /* Values per row. */
    const uint32_t rows = (range + width - 1U) / width;

    for (uint32_t b = 0U; b < PMF_BINS; ++b) { pmf[b] = 0U; }
    for (uint32_t i = 0U; i < NUM_SAMPLES; ++i) { ++pmf[(DataBuf[i] - lo) / width]; }
    for (uint32_t b = 0U; b < rows; ++b) { if (pmf[b] > peak) { peak = pmf[b]; } }

    printf("PMF     : value%s, count (%lu value(s) per row)\r\n",
           (width > 1U) ? " range" : "", (unsigned long)width);

    for (uint32_t b = 0U; b < rows; ++b)
    {
        const uint32_t first = lo + (b * width);
        const uint32_t last = first + width - 1U;
        const uint32_t length = (((uint32_t)pmf[b] * PMF_BAR_WIDTH) + peak - 1U) / peak;
        uint32_t k;

        for (k = 0U; k < length; ++k) { bar[k] = '#'; }
        bar[k] = '\0';

        if (width > 1U) { printf("  %5lu-%5lu, %4u |%s\r\n", (unsigned long)first, (unsigned long)last, pmf[b], bar); }
        else            { printf("  %5lu, %4u |%s\r\n", (unsigned long)first, pmf[b], bar); }
    }
}

static void print_summary(void)
{
    printf("\r\n==================== SUMMARY (1000 samples per row) ====================\r\n");
    printf("avg | mean      | std     | SNR        | ENOB(bits) | maxT(ns)  | jitter(ns) | missed\r\n");

    for (uint32_t s = 0U; s < ADC_MAX_SETTINGS; ++s)
    {
        const result_t *r = &g_results[s];

        printf("%3lu | %5lu.%02lu | %3lu.%02lu | ", (unsigned long)r->averages,
               (unsigned long)(r->mean_x100 / 100UL), (unsigned long)(r->mean_x100 % 100UL),
               (unsigned long)(r->std_x100 / 100UL), (unsigned long)(r->std_x100 % 100UL));

        if (r->snr_q8 > 0U) { print_q8((int32_t)r->snr_q8); printf(" | "); print_q8(r->enob_q8); }
        else                { printf("undefined | n/a"); }

        printf(" | %9lu | %10lu | %lu\r\n", (unsigned long)COUNTS_TO_NS(r->max_delta),
               (unsigned long)COUNTS_TO_NS(r->jitter), (unsigned long)r->missed);
    }
    printf("========================================================================\r\n");
}

/* ============================ Capture control ======================== */

static void run_capture(uint8_t log2n)
{
    adc_set_averaging(log2n);

    g_sample_index = 0U;
    g_max_conv_counts = 0U;
    g_capture_done = false;
    __DMB();
    g_capture_active = true;                              /* The ISR starts sampling on the next tick. */

    while (!g_capture_done)
    {
#if CRITICAL_SECTION_DEMO != 0
        TOGGLE_PIN(PIN_B_MASK);                           /* Pin B toggled once per main-loop pass. */
#else
        __WFI();
#endif
    }
    __DMB();                                              /* Buffers are complete and safe to read. */
}

static void run_sweep(void)
{
    for (uint32_t s = 0U; s < ADC_MAX_SETTINGS; ++s)
    {
        const uint8_t log2n = g_avg_log2_table[s];
        result_t *r = &g_results[s];

        r->averages = 1UL << log2n;
        r->scale = adc_result_scale(log2n);

        run_capture(log2n);
        analyse_timing(r);
        analyse_data(r);

        printf("\r\n---- Hardware averaging: %lu sample(s) per result (SAMPLENUM = %u) ----\r\n",
               (unsigned long)r->averages, (unsigned int)log2n);
        print_timing(r);
        print_statistics(r);
        print_pmf();
    }
}

/* ======================= Arduino-framework entry points ============== */

extern "C" void setup(void)
{
    (void)setvbuf(stdout, NULL, _IONBF, 0);
    (void)setvbuf(stderr, NULL, _IONBF, 0);

    uart_init();
    adc_init();
    tc4_init_timestamp();

#if CRITICAL_SECTION_DEMO != 0
    PORT->Group[0].DIRSET.reg = PIN_A_MASK | PIN_B_MASK;  /* Pins A and B as outputs. */
    PORT->Group[0].OUTCLR.reg = PIN_A_MASK | PIN_B_MASK;
#endif

    tc3_init_1khz_tick();
    __enable_irq();

    printf("\r\nECNG 3006 - Data acquisition: 1 kHz ADC, jitter, SNR/ENOB, PMF\r\n");
    printf("A0 (PA02), %lu samples per record, critical-section demo mode = %d\r\n",
           (unsigned long)NUM_SAMPLES, (int)CRITICAL_SECTION_DEMO);
    printf("Starting in %lu s (press reset to repeat)...\r\n", (unsigned long)(START_DELAY_MS / 1000UL));
}

extern "C" void loop(void)
{
    if (!g_sweep_done)
    {
        wait_ms(START_DELAY_MS);
        run_sweep();
        print_summary();
        g_sweep_done = true;
    }
    __WFI();
}