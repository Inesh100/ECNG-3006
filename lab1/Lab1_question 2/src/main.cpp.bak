#include <sam.h>
#include <FreeRTOS_SAMD21.h>

// ---------- Raw SERCOM5 UART (MKR Zero: pin 14 = TX/PB22, pin 13 = RX/PB23) ----------
static void uart_init(void) {
    // Route PB22/PB23 to SERCOM5 (peripheral function D = 3)
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);

    // Power and clock SERCOM5 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Reset, then configure: internal clock, TX on PAD2, RX on PAD3, LSB first
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
}

static void uart_puts(const char *s) {
    while (*s) {
        while (!SERCOM5->USART.INTFLAG.bit.DRE);
        SERCOM5->USART.DATA.reg = (uint8_t)*s++;
    }
}

static void uart_putu(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do { buf[--i] = '0' + (v % 10); v /= 10; } while (v);
    uart_puts(&buf[i]);
}


// Task function that flips raw hardware pin bits using the SAMD21 Port Controller
void vBlinkTask(void *pvParameters) {
    // 1. Configure Pin PB08 (Built-in LED) as a digital OUTPUT
    // PORT->Group[1] refers to PORT B. (Group[0] would be PORT A)
    PORT->Group[1].DIRSET.reg = PORT_PB08;
    uint32_t count = 0;

    while (1) {
        // 2. Toggle the pin state by writing a 1 to the OUTTGL register
        PORT->Group[1].OUTTGL.reg = PORT_PB08;


        uart_puts("toggle ");
        uart_putu(count++);
        uart_puts("\r\n");
        
        // 3. Keep the task asleep for 500ms non-blocking
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms delay
    }
}

int main(void) {
    // Initialize standard microcontroller system clocks (48MHz layout)
    SystemInit();
    uart_init();
    
    // Create the blinking thread loop
    xTaskCreate(
        vBlinkTask,       // Task function
        "Blinker",        // Name tag
        128,              // Stack depth 
        NULL,             // Task inputs
        1,                // Execution priority level
        NULL              // Handle reference

    );
    
    // Boot the FreeRTOS Scheduler engine execution path
    vTaskStartScheduler();
    
    while (1) {
        // Safety lock trap loop
    }
    return 0;
}

// Retain underlying framework linkages to keep compilation targets happy
extern "C" {
  void setup() {
    // Left completely blank
  }
  void loop() {
    // Left completely blank
  }
}