#include <sam.h>
#include <FreeRTOS_SAMD21.h>


// Task function that flips raw hardware pin bits using the SAMD21 Port Controller
void vBlinkTask(void *pvParameters) {
    // 1. Configure Pin PB08 (Built-in LED) as a digital OUTPUT
    // PORT->Group[1] refers to PORT B. (Group[0] would be PORT A)
    PORT->Group[1].DIRSET.reg = PORT_PB08;

    while (1) {
        // 2. Toggle the pin state by writing a 1 to the OUTTGL register
        PORT->Group[1].OUTTGL.reg = PORT_PB08;
        
        // 3. Keep the task asleep for 500ms non-blocking
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms delay
    }
}

int main(void) {
    // Initialize standard microcontroller system clocks (48MHz layout)
    SystemInit();
    
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
    main();
  }
  void loop() {
    // Left completely blank
  }
}

