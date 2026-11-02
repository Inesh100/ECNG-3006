/*
 * ECNG 3006 - Question 7: Serial-RX Debouncing Finite State Machine (main.cpp)
 * Platform   : Arduino MKR Zero (ATSAMD21G18A)
 * Student    : Nicholas Ramnarine
 * Student ID : 816041866
 *
 * Compiler   : PlatformIO ARM GCC toolchain (C-style code built as C++)
 * System tick: TC3 hardware timer interrupt, nominal 1 ms
 * Serial     : SERCOM5 (PB22 TX / PB23 RX), 115200 baud, polled receive
 * Linkage    : extern "C" is used on setup(), loop(), TC3_Handler() and the
 *              Newlib stubs so the framework/vector table find the C symbols.
 *
 * This application uses SAMD21 registers directly (no Arduino.h, no Arduino
 * API calls). The Arduino framework supplies startup and calls setup()/loop().
 *
 * FSM IMPLEMENTATION STRATEGY - SWITCH-CASE (Williams, Ch. 6)
 * -----------------------------------------------------------
 * 1. The FSM has four states:
 *      STATE_OFF           LED off, waiting for the valid character
 *      STATE_ON            LED on,  waiting for the valid character
 *      STATE_OFF_DEBOUNCE  LED off, just switched off, ignoring valid chars
 *      STATE_ON_DEBOUNCE   LED on,  just switched on,  ignoring valid chars
 * 2. Only ONE character (FSM_VALID_CHAR) is valid. Every other character is
 *    invalid and never changes the state or the debounce counter.
 * 3. Incoming serial data is pre-processed by read_event(), which collapses
 *    everything received during one tick into a single FSM event
 *    (EVENT_NONE / EVENT_INVALID_CHAR / EVENT_VALID_CHAR).
 * 4. The TC3 ISR only increments a tick counter. loop() runs exactly one
 *    fsm_step() per tick, so FSM transitions happen only on a system tick.
 * 5. fsm_step() is a switch on the current state. Each case tests the event,
 *    performs the actions (LED, counter) and assigns the next state.
 * 6. The 500 ms debounce uses a tick counter. It is cleared when a debounce
 *    state is entered, and again whenever the valid character arrives while
 *    in a debounce state (that character is ignored). When the counter
 *    reaches DEBOUNCE_TICKS (500 x 1 ms) the FSM returns to the matching
 *    idle state, ready to accept the next valid character.
 */

#include <sam.h>          /* SAMD21 register definitions and CMSIS functions. */
#include <stdint.h>       /* Fixed-width integer types.                       */
#include <stdio.h>        /* printf() and standard-output functions.          */
#include <errno.h>        /* errno codes for the low-level I/O stubs.         */
#include <sys/stat.h>     /* struct stat and S_IFCHR.                         */
#include <sys/types.h>    /* off_t for _lseek().                              */

/* ---------------------- Configuration constants ---------------------- */

#define CPU_CLOCK_HZ        (48000000UL)  /* GCLK0 frequency: 48 MHz.            */
#define TIMER_PRESCALER     (64UL)        /* TC3 clock divider.                  */
#define TIMER_TICKS_PER_MS  (CPU_CLOCK_HZ / TIMER_PRESCALER / 1000UL) /* 750.    */
#define UART_BAUD_REG_VALUE (65326U)      /* SERCOM5 BAUD value: 115200 @ 48 MHz. */
#define UART_ERROR_MASK     ((uint16_t)(SERCOM_USART_STATUS_PERR | \
                                        SERCOM_USART_STATUS_FERR | \
                                        SERCOM_USART_STATUS_BUFOVF))

#define FSM_VALID_CHAR      ('t')         /* The single valid character.         */
#define DEBOUNCE_TICKS      (500U)        /* 500 ticks x 1 ms = 500 ms window.   */

/* ----------------------- FSM types and variables ---------------------- */

typedef enum
{
    STATE_OFF = 0,        /* LED off, idle.                                  */
    STATE_OFF_DEBOUNCE,   /* LED off, debouncing after an ON -> OFF change. */
    STATE_ON,             /* LED on, idle.                                   */
    STATE_ON_DEBOUNCE     /* LED on, debouncing after an OFF -> ON change.  */
} fsm_state_t;

typedef enum
{
    EVENT_NONE = 0,       /* No character arrived during this tick.          */
    EVENT_INVALID_CHAR,   /* A character other than FSM_VALID_CHAR arrived.  */
    EVENT_VALID_CHAR      /* FSM_VALID_CHAR arrived.                         */
} fsm_event_t;

static volatile uint32_t g_tick_count = 0U;    /* Incremented by the TC3 ISR.      */
static uint32_t g_ticks_handled = 0U;          /* Ticks already given to the FSM.  */
static fsm_state_t g_state = STATE_OFF;        /* Current FSM state.               */
static uint16_t g_debounce_ticks = 0U;         /* Ticks elapsed in debounce state. */

/* ========================== LED configuration ======================== */

static void led_init(void) /* Configures the MKR Zero built-in LED (PB08). */
{
    PORT->Group[1].DIRSET.reg = PORT_PB08;  /* PB08 is an output.            */
    PORT->Group[1].OUTCLR.reg = PORT_PB08;  /* LED off = initial FSM state.  */
}

static void led_set(bool on) /* Drives the LED on or off. */
{
    if (on) { PORT->Group[1].OUTSET.reg = PORT_PB08; }
    else    { PORT->Group[1].OUTCLR.reg = PORT_PB08; }
}

/* ============================ UART setup ============================= */

/*
 * SERCOM5 is used in polling mode. The MKR Zero variant already defines
 * SERCOM5_Handler() for Serial1, so this program does not define another.
 */

static void uart_init(void) /* Configures SERCOM5 for UART transmit and receive. */
{
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN; /* PB22 (TX) to peripheral mux. */
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN; /* PB23 (RX) to peripheral mux. */
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3U) | PORT_PMUX_PMUXO(3U); /* Function D. */

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5; /* Enable the SERCOM5 bus clock. */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST; /* Software reset. */
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST != 0U) { /* Wait while reset synchronizes. */ }
    while (SERCOM5->USART.CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }

    /* USART mode, LSB first, RX on PAD3, TX on PAD2. */
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1U) |
                               SERCOM_USART_CTRLA_RXPO(3U) | SERCOM_USART_CTRLA_TXPO(1U);
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB != 0U) { /* Wait for CTRLB. */ }

    SERCOM5->USART.BAUD.reg = UART_BAUD_REG_VALUE; /* Nominal 115200 baud. */
    SERCOM5->USART.CTRLA.bit.ENABLE = 1U;          /* Enable the USART.    */
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE != 0U) { /* Wait for enable. */ }
}

static void uart_putc(char character) /* Transmits one character. */
{
    while (SERCOM5->USART.INTFLAG.bit.DRE == 0U) { /* Wait for an empty data register. */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)character;
}

/*
 * Non-blocking serial receive.
 * Returns false immediately if no character has arrived (RXC flag clear).
 * Otherwise stores the character in *character and returns true. A character
 * received with a parity, framing or overflow error is reported as '\0',
 * which can never equal FSM_VALID_CHAR, so it is treated as invalid.
 */
static bool uart_try_getc(char *character)
{
    uint16_t status;
    uint16_t data;

    if (SERCOM5->USART.INTFLAG.bit.RXC == 0U) { return false; } /* Nothing received. */

    status = SERCOM5->USART.STATUS.reg; /* Read status before data: errors belong to this character. */
    data   = SERCOM5->USART.DATA.reg;   /* Reading DATA also clears the RXC flag.                    */

    if ((status & UART_ERROR_MASK) != 0U)
    {
        SERCOM5->USART.STATUS.reg = (uint16_t)(status & UART_ERROR_MASK); /* Write-1-to-clear. */
        *character = '\0';
    }
    else
    {
        *character = (char)(data & 0xFFU);
    }
    return true;
}

/* ------------------ Redirect C standard output to UART ---------------- */

extern "C" int _write(int file, char *buffer, int length) /* Used by printf(). */
{
    if ((file != 1) && (file != 2)) { errno = EBADF; return -1; }
    if ((buffer == NULL) || (length < 0)) { errno = EINVAL; return -1; }

    for (int index = 0; index < length; ++index) { uart_putc(buffer[index]); }

    while (SERCOM5->USART.INTFLAG.bit.TXC == 0U) { /* Wait for the last character to finish. */ }
    return length;
}

/* Stubs that let Newlib link when no filesystem is present. */

extern "C" int _close(int file) { (void)file; errno = EBADF; return -1; }

extern "C" int _fstat(int file, struct stat *status)
{
    (void)file;
    if (status == NULL) { errno = EINVAL; return -1; }
    status->st_mode = S_IFCHR; /* Treat the UART as a character device. */
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
    return 0; /* stdin is not routed through Newlib; RX uses uart_try_getc(). */
}

extern "C" void _exit(int status)
{
    (void)status;
    for (;;) { __WFI(); }
}

/* ======================= Hardware timer (system tick) ================ */

/*
 * TC3 uses GCLK0 / 64 = 750,000 counts per second.
 * 750 counts = 1 ms, so CC0 = 749 (counter restarts on each match).
 */

extern "C" void TC3_Handler(void) /* Name must match the TC3 vector-table entry. */
{
    if (TC3->COUNT16.INTFLAG.bit.MC0 != 0U)
    {
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0; /* Write-1-to-clear. */
        ++g_tick_count;                            /* One more system tick (1 ms). */
    }
}

static void tc3_init_1ms_tick(void) /* Configures TC3 for a 1 ms periodic interrupt. */
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for clock synchronization. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for reset. */ }
    while (TC3->COUNT16.CTRLA.bit.SWRST != 0U) { /* Wait until reset bit clears. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV64;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for CTRLA. */ }

    TC3->COUNT16.CC[0].reg = (uint16_t)(TIMER_TICKS_PER_MS - 1UL); /* CC0 = 749. */
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for CC0. */ }

    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;   /* Clear any stale flag.    */
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0; /* Enable the MC0 interrupt. */

    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 3U);
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.bit.ENABLE = 1U; /* Start the counter. */
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for enable. */ }
}

/* ===================== Finite state machine (switch-case) ============= */

/*
 * Event pre-processing: drain every character received since the last tick
 * and reduce it to ONE event. A valid character takes priority over invalid
 * ones, so a burst of characters can never cause more than one transition.
 */
static fsm_event_t read_event(void)
{
    fsm_event_t event = EVENT_NONE;
    char received;

    while (uart_try_getc(&received))
    {
        if (received == FSM_VALID_CHAR)  { event = EVENT_VALID_CHAR; }
        else if (event == EVENT_NONE)    { event = EVENT_INVALID_CHAR; }
    }
    return event;
}

/* One FSM step. Called once per system tick with the event for that tick. */
static void fsm_step(fsm_event_t event)
{
    /* Invalid characters never cause a transition in any state. */
    if (event == EVENT_INVALID_CHAR)
    {
        printf("[%lu ms] invalid character ignored\r\n", (unsigned long)g_tick_count);
    }

    switch (g_state)
    {
        case STATE_OFF:
            if (event == EVENT_VALID_CHAR)
            {
                led_set(true);                 /* Action: LED on.            */
                g_debounce_ticks = 0U;         /* Start the 500 ms window.   */
                g_state = STATE_ON_DEBOUNCE;   /* Next state.                */
                printf("[%lu ms] '%c' accepted: OFF -> ON\r\n",
                       (unsigned long)g_tick_count, FSM_VALID_CHAR);
            }
            break;

        case STATE_ON:
            if (event == EVENT_VALID_CHAR)
            {
                led_set(false);                /* Action: LED off.           */
                g_debounce_ticks = 0U;
                g_state = STATE_OFF_DEBOUNCE;
                printf("[%lu ms] '%c' accepted: ON -> OFF\r\n",
                       (unsigned long)g_tick_count, FSM_VALID_CHAR);
            }
            break;

        case STATE_ON_DEBOUNCE:
            if (event == EVENT_VALID_CHAR)
            {
                g_debounce_ticks = 0U;         /* Same character inside window: ignore, restart. */
                printf("[%lu ms] '%c' ignored (within 500 ms)\r\n",
                       (unsigned long)g_tick_count, FSM_VALID_CHAR);
            }
            else
            {
                ++g_debounce_ticks;
                if (g_debounce_ticks >= DEBOUNCE_TICKS)
                {
                    g_state = STATE_ON;        /* Window closed: ready again. */
                    printf("[%lu ms] debounce complete, state ON\r\n", (unsigned long)g_tick_count);
                }
            }
            break;

        case STATE_OFF_DEBOUNCE:
            if (event == EVENT_VALID_CHAR)
            {
                g_debounce_ticks = 0U;
                printf("[%lu ms] '%c' ignored (within 500 ms)\r\n",
                       (unsigned long)g_tick_count, FSM_VALID_CHAR);
            }
            else
            {
                ++g_debounce_ticks;
                if (g_debounce_ticks >= DEBOUNCE_TICKS)
                {
                    g_state = STATE_OFF;
                    printf("[%lu ms] debounce complete, state OFF\r\n", (unsigned long)g_tick_count);
                }
            }
            break;

        default: /* Unreachable unless the state variable is corrupted: recover safely. */
            led_set(false);
            g_debounce_ticks = 0U;
            g_state = STATE_OFF;
            break;
    }
}

/* ======================= Arduino-framework entry points ============== */

/*
 * The framework supplies main(), which calls setup() once and then calls
 * loop() repeatedly. This program therefore does not define main().
 */

extern "C" void setup(void)
{
    (void)setvbuf(stdout, NULL, _IONBF, 0); /* Send printf output immediately. */
    (void)setvbuf(stderr, NULL, _IONBF, 0);

    uart_init();
    led_init();
    tc3_init_1ms_tick();
    __enable_irq();

    printf("\r\nECNG 3006 - Q7 Serial Debounce FSM (switch-case)\r\n");
    printf("Valid character: '%c'   Debounce window: %u ms   LED starts OFF\r\n\r\n",
           FSM_VALID_CHAR, (unsigned int)DEBOUNCE_TICKS);
}

extern "C" void loop(void)
{
    if (g_ticks_handled == g_tick_count) /* No new tick yet: sleep until the next interrupt. */
    {
        __WFI();
        return;
    }

    ++g_ticks_handled;       /* Consume exactly one tick (none are lost if printf runs long). */
    fsm_step(read_event());  /* Pre-process serial data into one event, then step the FSM.    */
}