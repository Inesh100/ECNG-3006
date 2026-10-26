#include <sam.h>
#include <FreeRTOS_SAMD21.h>
#include <stdbool.h>

// =====================================================================
//  Interrupt-driven design
//
//   TC3 timer ISR (every 500 ms)  -> toggles LED PB08 directly
//                                 -> notifies the print task
//   print task                    -> sleeps until notified, then queues text
//   SERCOM5 DRE ISR               -> shifts queued bytes out the UART
//
//  Nothing polls or delays: the task blocks on a notification, and the
//  UART/LED are serviced entirely from interrupts.
// =====================================================================

#define UART_TX_QUEUE_LEN 128

static QueueHandle_t txQueue;
static TaskHandle_t  printTaskHandle;

// ---------- Raw SERCOM5 UART, interrupt-driven TX ----------
// MKR Zero: pin 14 = TX/PB22, pin 13 = RX/PB23
static void uart_init(void) {
    // Route PB22/PB23 to SERCOM5 (peripheral function D = 3)
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);

    // Power and clock SERCOM5 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    while (GCLK->STATUS.bit.SYNCBUSY);   // one-time init wait, hardware-bounded

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST);

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_MODE(1) |
                               SERCOM_USART_CTRLA_RXPO(3) |
                               SERCOM_USART_CTRLA_TXPO(1);
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB);

    SERCOM5->USART.BAUD.reg = 63019;            // 115200 baud @ 48 MHz
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE);

    // TX queue + interrupt (DRE stays disabled until there is data)
    txQueue = xQueueCreate(UART_TX_QUEUE_LEN, sizeof(uint8_t));
    NVIC_SetPriority(SERCOM5_IRQn, 2);
    NVIC_EnableIRQ(SERCOM5_IRQn);
}

// Task context only (blocks if the queue is full)
static void uart_putc(char c) {
    uint8_t b = (uint8_t)c;
    xQueueSend(txQueue, &b, portMAX_DELAY);
    SERCOM5->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;   // kick the ISR
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_putu(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do { buf[--i] = '0' + (v % 10); v /= 10; } while (v);
    uart_puts(&buf[i]);
}

extern "C" void SERCOM5_Handler(void) {
    if (SERCOM5->USART.INTFLAG.bit.DRE && SERCOM5->USART.INTENSET.bit.DRE) {
        uint8_t c;
        BaseType_t woken = pdFALSE;

        if (xQueueReceiveFromISR(txQueue, &c, &woken) == pdTRUE) {
            SERCOM5->USART.DATA.reg = c;                                // clears DRE
        } else {
            SERCOM5->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;    // queue empty
        }

        portYIELD_FROM_ISR(woken);
    }
}

// ---------- TC3 500 ms timer: toggles the LED from the ISR ----------
// GCLK0 48 MHz / 1024 prescaler = 46875 Hz
// Match-frequency mode: period = CC0 + 1 counts = 23437 counts ~= 500 ms
static void led_timer_init(void) {
    // LED on PB08 as output
    PORT->Group[1].DIRSET.reg = PORT_PB08;

    // Power and clock TC3 from GCLK0
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    while (TC3->COUNT16.CTRLA.bit.SWRST);

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                             TC_CTRLA_WAVEGEN_MFRQ |
                             TC_CTRLA_PRESCALER_DIV1024;
    TC3->COUNT16.CC[0].reg = 23436;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;      // interrupt on CC0 match
    NVIC_SetPriority(TC3_IRQn, 2);
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

extern "C" void TC3_Handler(void) {
    if (TC3->COUNT16.INTFLAG.bit.MC0) {
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;    // clear the flag

        PORT->Group[1].OUTTGL.reg = PORT_PB08;        // toggle LED in the ISR

        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(printTaskHandle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// ---------- Task: sleeps until the timer ISR wakes it ----------
void vPrintTask(void *pvParameters) {
    uint32_t count = 0;

    // Start the timer only once the task handle is valid
    led_timer_init();

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      // blocked, 0% CPU, until ISR

        uart_puts("toggle ");
        uart_putu(count++);
        uart_puts("\r\n");
    }
}

int main(void) {
    SystemInit();
    uart_init();

    xTaskCreate(
        vPrintTask,       // Task function
        "Printer",        // Name tag
        256,              // Stack depth (words)
        NULL,             // Task inputs
        1,                // Priority
        &printTaskHandle  // Handle (needed by the timer ISR)
    );

    vTaskStartScheduler();

    while (1) {
        // Safety trap
    }
    return 0;
}

// Retain framework linkages to keep compilation targets happy
extern "C" {
  void setup() {}
  void loop() {}
}