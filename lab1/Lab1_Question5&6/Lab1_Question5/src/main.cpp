
/*
 * ECNG 3006 - Demonstration Cyclic Executive
 * Platform: Arduino MKR Zero (ATSAMD21G18A)
 * Student: Nicholas Ramnarine
 * Student ID: 816041866
 *
 * Compiler: PlatformIO ARM GCC toolchain
 * Timing: TC3 hardware timer interrupt every nominal 1 ms
 * Printing: C library printf(), retargeted to SERCOM5 UART
 *
 * This application uses SAMD21 registers directly.
 * It does not include Arduino.h or call Arduino API functions.
 * The Arduino framework supplies startup and calls setup() and loop().
 */

#include <sam.h>          // SAMD21 register definitions and CMSIS functions.
#include <stdint.h>       // Provides fixed-width integer types.
#include <stdio.h>        // Provides printf() and standard-output functions.
#include <errno.h>        // Provides errno error codes for low-level I/O.
#include <sys/stat.h>     // Provides struct stat and S_IFCHR.
#include <sys/types.h>    // Provides the off_t type for _lseek().

/* ---------------------- Configuration constants ---------------------- */

#define CPU_CLOCK_HZ          (48000000UL) // Assumed GCLK0 frequency: 48 MHz.
#define TIMER_PRESCALER       (64UL)       // TC3 timer clock divider.
#define TIMER_TICKS_PER_MS    (CPU_CLOCK_HZ / TIMER_PRESCALER / 1000UL) // 750 counts per ms.
#define UART_BAUD_REG_VALUE   (63019U)     // SERCOM5 BAUD value for 115200 baud at 48 MHz.
#define SLOT_COUNT            (5U)         // Number of task slots per major cycle.
#define SLOT_DURATION_MS      (1000ULL)    // Each slot is scheduled one second apart.
#define MAJOR_CYCLE_MS         ((uint64_t)SLOT_COUNT * SLOT_DURATION_MS) // Five-second cycle.

/* ---------------------- Software time structure ---------------------- */

typedef struct
{
    uint32_t seconds;       // Whole seconds since the program started.
    uint16_t milliseconds;  // Milliseconds within the current second.
} time_structure_t;         // Names the structure type used for timekeeping.

static volatile time_structure_t g_system_time = {0U, 0U}; // Shared time updated by the ISR.
static uint64_t g_cycle_start_ms = 0ULL; // Absolute start time of the current major cycle.
static uint32_t g_cycle_number = 0U;     // Counts completed major cycles.

/* ========================== LED configuration ======================== */

static void led_init(void) // Configures the MKR Zero's built-in LED.
{
    PORT->Group[1].DIRSET.reg = PORT_PB08; // Set PB08 as an output pin.
    PORT->Group[1].OUTCLR.reg = PORT_PB08; // Start with the LED output LOW.
}

/* ============================ UART setup ============================= */

/*
 * SERCOM5 transmits in polling mode rather than using a custom UART ISR.
 * The MKR Zero Arduino variant already defines SERCOM5_Handler() for Serial1.
 * This program therefore does not define another SERCOM5_Handler().
 */

static void uart_init(void) // Configures SERCOM5 for UART transmission.
{
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN; // Enable peripheral mux for PB22/TX.
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN; // Enable peripheral mux for PB23/RX.
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3U) | PORT_PMUX_PMUXO(3U); // Select peripheral function D.

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5; // Enable the SERCOM5 peripheral bus clock.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE; // Route GCLK0 to SERCOM5.
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for the clock configuration to synchronize. */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST; // Request a software reset of SERCOM5.
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST != 0U) { /* Wait while the reset synchronizes. */ }
    while (SERCOM5->USART.CTRLA.bit.SWRST != 0U) { /* Wait until the software-reset bit clears. */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1U) | SERCOM_USART_CTRLA_RXPO(3U) | SERCOM_USART_CTRLA_TXPO(1U); // Select USART mode, LSB-first, RX PAD3 and TX PAD2.
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN; // Enable USART transmitter and receiver.
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB != 0U) { /* Wait for CTRLB to synchronize. */ }

    SERCOM5->USART.BAUD.reg = UART_BAUD_REG_VALUE; // Configure the baud rate for nominal 115200 baud.
    SERCOM5->USART.CTRLA.bit.ENABLE = 1U; // Enable the SERCOM5 USART peripheral.
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE != 0U) { /* Wait for USART enable synchronization. */ }
}

static void uart_putc(char character) // Transmits one character using the UART data register.
{
    while (SERCOM5->USART.INTFLAG.bit.DRE == 0U) { /* Wait until the transmit data register is empty. */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)character; // Place the next character into the data register.
}

/* ------------------ Redirect C standard output to UART ---------------- */

/*
 * Newlib/newlib-nano calls _write() when printf() sends output.
 * The C-linkage declaration ensures that the runtime can find this symbol.
 */

extern "C" int _write(int file, char *buffer, int length) // Low-level output routine used by printf().
{
    if ((file != 1) && (file != 2)) { errno = EBADF; return -1; } // Accept stdout (1) and stderr (2) only.
    if ((buffer == NULL) || (length < 0)) { errno = EINVAL; return -1; } // Reject invalid output arguments.

    for (int index = 0; index < length; ++index) { uart_putc(buffer[index]); } // Send each byte over the UART.

    while (SERCOM5->USART.INTFLAG.bit.TXC == 0U) { /* Wait until the last transmitted character finishes. */ }
    return length; // Report the number of characters successfully accepted for transmission.
}

/* These C-linkage stubs support Newlib when no filesystem is present. */

extern "C" int _close(int file) // Implements close() for an unsupported file descriptor.
{
    (void)file;                 // Mark the argument as intentionally unused.
    errno = EBADF;             // Report a bad file descriptor.
    return -1;                 // Indicate that closing failed.
}

extern "C" int _fstat(int file, struct stat *status) // Supplies basic stream information to Newlib.
{
    (void)file;                                     // The device has no real filesystem.
    if (status == NULL) { errno = EINVAL; return -1; } // Reject a null status pointer.
    status->st_mode = S_IFCHR;                      // Mark standard output as a character device.
    return 0;                                       // Report success.
}

extern "C" int _isatty(int file) // Reports whether a descriptor represents a terminal.
{
    return ((file == 1) || (file == 2)) ? 1 : 0; // Identify stdout and stderr as terminal-like streams.
}

extern "C" off_t _lseek(int file, off_t offset, int whence) // Handles file-position requests.
{
    (void)file;                       // No file is associated with the UART.
    (void)offset;                     // Seeking is not supported.
    (void)whence;                     // The seek mode is not used.
    errno = ESPIPE;                   // Report that seeking is invalid for this stream.
    return (off_t)-1;                 // Return the standard failure value.
}

extern "C" int _read(int file, char *buffer, int length) // Handles standard-input requests.
{
    (void)file;                       // No input descriptor is implemented.
    (void)buffer;                     // No input buffer will be filled.
    (void)length;                     // No input length is required.
    return 0;                         // Report end-of-file because UART input is not implemented.
}

extern "C" void _exit(int status) // Provides a terminal state if the C runtime exits.
{
    (void)status;                     // Ignore the exit status on this embedded target.
    for (;;) { __WFI(); }             // Remain in a low-power wait state indefinitely.
}

/* ======================= Hardware timer and time ===================== */

/*
 * TC3 uses GCLK0 / 64 = 750,000 timer counts per second.
 * 750 counts produce a nominal 1 ms interval, so CC0 = 749.
 */

extern "C" void TC3_Handler(void) // Exact C-linkage symbol expected by the TC3 vector-table entry.
{
    if (TC3->COUNT16.INTFLAG.bit.MC0 != 0U) // Check whether the match/compare-zero flag is set.
    {
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0; // Clear the write-one-to-clear interrupt flag.

        ++g_system_time.milliseconds; // Advance the software time by one millisecond.
        if (g_system_time.milliseconds >= 1000U) // Check whether one full second has elapsed.
        {
            g_system_time.milliseconds = 0U; // Reset the within-second millisecond count.
            ++g_system_time.seconds;         // Advance the whole-seconds count.
        }
    }
}

static void tc3_init_1ms_tick(void) // Configures TC3 to generate periodic 1 ms interrupts.
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC3; // Enable the TC3 peripheral bus clock.

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3; // Route GCLK0 to the TC3/TCC2 clock channel.
    while (GCLK->STATUS.bit.SYNCBUSY != 0U) { /* Wait for generic-clock synchronization. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST; // Reset the TC3 counter peripheral.
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for reset synchronization. */ }
    while (TC3->COUNT16.CTRLA.bit.SWRST != 0U) { /* Wait until the reset bit clears. */ }

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV64; // Select 16-bit mode, match-frequency waveform and divide-by-64.
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for the CTRLA write to synchronize. */ }

    TC3->COUNT16.CC[0].reg = (uint16_t)(TIMER_TICKS_PER_MS - 1UL); // Set CC0 to 749 for a 1 ms period.
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for compare-register synchronization. */ }

    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0; // Clear any stale compare-match flag.
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0; // Enable the TC3 match/compare-zero interrupt.

    NVIC_ClearPendingIRQ(TC3_IRQn); // Clear any previously pending TC3 interrupt in the NVIC.
    NVIC_SetPriority(TC3_IRQn, 3U); // Assign the TC3 interrupt priority.
    NVIC_EnableIRQ(TC3_IRQn); // Enable the TC3 interrupt at the NVIC.

    TC3->COUNT16.CTRLA.bit.ENABLE = 1U; // Start the TC3 counter.
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY != 0U) { /* Wait for the timer-enable write to synchronize. */ }
}

static time_structure_t time_now(void) // Returns a consistent snapshot of the ISR-updated time.
{
    time_structure_t snapshot;                 // Local copy of the current time.
    uint32_t previous_primask = __get_PRIMASK(); // Remember whether global interrupts were already disabled.

    __disable_irq();                           // Prevent the timer ISR from modifying the fields during the copy.
    snapshot.seconds = g_system_time.seconds;  // Copy the seconds field.
    snapshot.milliseconds = g_system_time.milliseconds; // Copy the milliseconds field.

    if ((previous_primask & 1U) == 0U) { __enable_irq(); } // Re-enable interrupts only if they were enabled before.
    return snapshot;                           // Return the consistent time snapshot.
}

static uint64_t time_now_ms(void) // Converts the time structure into total milliseconds since startup.
{
    const time_structure_t snapshot = time_now(); // Read both time fields consistently.
    return ((uint64_t)snapshot.seconds * 1000ULL) + (uint64_t)snapshot.milliseconds; // Convert seconds and milliseconds to one value.
}

static void sleep_ms(uint32_t duration_ms) // Waits for a relative duration measured by the hardware timer.
{
    const uint64_t deadline_ms = time_now_ms() + (uint64_t)duration_ms; // Calculate the desired wake-up time.

    while (time_now_ms() < deadline_ms) { __WFI(); } // Wait for interrupts until the deadline has been reached.
}

static void sleep_until_ms(uint64_t deadline_ms) // Waits until an absolute uptime deadline.
{
    while (time_now_ms() < deadline_ms) { __WFI(); } // Sleep until the TC3 ISR advances the time to the deadline.
}

/* ======================== Cyclic executive tasks ===================== */

static void task_slot_0(void) // Task executed in slot zero of each major cycle.
{
    const time_structure_t now = time_now(); // Read the current uptime.
    printf("[slot 0] cycle start; uptime %lu.%03u s\r\n", (unsigned long)now.seconds, (unsigned int)now.milliseconds); // Print the current time.
}

static void task_slot_1(void) // Example task assigned to the second slot.
{
    printf("[slot 1] periodic input/update task\r\n"); // Print the slot-one message.
}

static void task_slot_2(void) // Example task assigned to the third slot.
{
    printf("[slot 2] data-processing task\r\n"); // Print the slot-two message.
}

static void task_slot_3(void) // Example task assigned to the fourth slot.
{
    printf("[slot 3] status/report task\r\n"); // Print the slot-three message.
}

static void task_slot_4_burn(void) // Demonstrates work that occupies time until the cycle boundary.
{
    const uint64_t burn_start_ms = time_now_ms(); // Record when this demonstration begins.
    const uint64_t cycle_deadline_ms = g_cycle_start_ms + MAJOR_CYCLE_MS; // Calculate the end of the five-second cycle.

    printf("[slot 4] demonstration task: waiting for the next five-second boundary\r\n"); // Announce the final slot.

    while (time_now_ms() < cycle_deadline_ms) { __asm volatile ("nop"); } // Busy-wait while the TC3 ISR continues updating time.

    printf("[slot 4] elapsed wait: %lu ms\r\n", (unsigned long)(time_now_ms() - burn_start_ms)); // Report how long the task waited.
}

typedef void (*cyclic_task_t)(void); // Defines a pointer type for functions with no arguments or return value.

static const cyclic_task_t g_task_table[SLOT_COUNT] = // Stores the tasks in their scheduled order.
{
    task_slot_0, // Run the cycle-start message in slot zero.
    task_slot_1, // Run the periodic input/update example in slot one.
    task_slot_2, // Run the data-processing example in slot two.
    task_slot_3, // Run the status/report example in slot three.
    task_slot_4_burn // Run the cycle-boundary demonstration in slot four.
};

/* ======================= Arduino-framework entry points ============== */

/*
 * PlatformIO's Arduino framework supplies main(), which calls setup() once
 * and then calls loop() repeatedly. This application therefore does not
 * define its own main() and does not include Arduino.h.
 */

extern "C"{

void setup(void) // Called once by the framework after its startup initialization.
{
    (void)setvbuf(stdout, NULL, _IONBF, 0); // Disable stdout buffering so printf output is sent promptly.
    (void)setvbuf(stderr, NULL, _IONBF, 0); // Disable stderr buffering for prompt error output.

    uart_init(); // Configure SERCOM5 before the first printf call.
    led_init(); // Configure PB08 as the built-in LED output.
    tc3_init_1ms_tick(); // Start the timer that updates the software clock.
    __enable_irq(); // Ensure global interrupts are enabled.

    sleep_ms(10U); // Allow a connected serial terminal time to open.

    printf("\r\nECNG 3006 - Cyclic Executive Demonstration\r\n"); // Print the application heading.
    printf("MKR Zero / ATSAMD21G18A, nominal TC3 tick = 1 ms\r\n"); // Report the target and tick period.
    printf("Five slots per major cycle; major cycle = 5 seconds\r\n\r\n"); // Describe the repeating schedule.

    g_cycle_start_ms = time_now_ms(); // Record the absolute start time of the first major cycle.
    g_cycle_number = 0U; // Start the cycle count at zero.
}

void loop(void) // Called by the framework; this implementation runs the cyclic executive continuously.
{
    for (;;) // Keep executing major cycles indefinitely.
    {
        for (uint32_t slot = 0U; slot < SLOT_COUNT; ++slot) // Visit each slot in the task table in order.
        {
            const uint64_t slot_deadline_ms = g_cycle_start_ms + ((uint64_t)slot * SLOT_DURATION_MS); // Calculate this slot's absolute start time.

            sleep_until_ms(slot_deadline_ms); // Wait until the scheduled start time, unless already late.

            PORT->Group[1].OUTTGL.reg = PORT_PB08; // Toggle the built-in LED once at the start of this slot.

            printf("[cycle %lu] ", (unsigned long)g_cycle_number); // Print the current major-cycle number.
            g_task_table[slot](); // Execute the task assigned to this slot.
        }

        sleep_until_ms(g_cycle_start_ms + MAJOR_CYCLE_MS); // Wait until the scheduled end of the five-second cycle.
        g_cycle_start_ms += MAJOR_CYCLE_MS; // Advance the schedule by one full cycle without accumulating task delays.
        ++g_cycle_number; // Increment the major-cycle counter.
    }
}
}


