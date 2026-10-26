
#include <unity.h>

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// ================================================================
// Native Linux simulation of the MKR Zero interrupt-driven design
//
// Simulates:
//   TC3 timer ISR       -> toggles LED state and gives notification
//   Print task          -> consumes notification and queues a message
//   SERCOM5 DRE ISR     -> transmits one queued byte per invocation
//
// This does NOT access SAMD21 hardware or run FreeRTOS.
// Each simulated TC3 ISR call represents one timer event.
// ================================================================

#define UART_TX_QUEUE_LEN 128
#define UART_OUTPUT_LEN   512

typedef struct
{
    // Simulated LED connected to PB08
    bool led_state;

    // Simulated FreeRTOS task notification count
    uint32_t notifications;

    // Number printed in "toggle N"
    uint32_t print_count;

    // Simulated UART transmit queue
    char tx_queue[UART_TX_QUEUE_LEN];
    size_t queue_head;
    size_t queue_tail;
    size_t queue_count;

    // Bytes transmitted by the simulated UART ISR
    char uart_output[UART_OUTPUT_LEN];
    size_t output_length;

} NativeModel;

static NativeModel model;

// -------------------- Simulated TC3 timer ISR --------------------

static void native_tc3_isr(void)
{
    // Equivalent to toggling PB08 using PORT.OUTTGL
    model.led_state = !model.led_state;

    // Equivalent to vTaskNotifyGiveFromISR()
    model.notifications++;
}

// -------------------- Simulated UART queue -----------------------

static bool native_uart_puts(const char *text)
{
    while (*text != '\0')
    {
        // The embedded implementation would block if its queue
        // were full. This simulation reports failure instead.
        if (model.queue_count >= UART_TX_QUEUE_LEN)
        {
            return false;
        }

        model.tx_queue[model.queue_tail] = *text++;

        model.queue_tail =
            (model.queue_tail + 1U) % UART_TX_QUEUE_LEN;

        model.queue_count++;
    }

    return true;
}

// -------------------- Simulated SERCOM5 DRE ISR ------------------

static void native_sercom5_dre_isr(void)
{
    // No data to transmit: equivalent to the interrupt being
    // disabled when the FreeRTOS queue becomes empty.
    if (model.queue_count == 0U)
    {
        return;
    }

    // Transmit one byte, as the hardware UART ISR would.
    char byte = model.tx_queue[model.queue_head];

    model.queue_head =
        (model.queue_head + 1U) % UART_TX_QUEUE_LEN;

    model.queue_count--;

    // Record the transmitted byte for testing.
    if (model.output_length + 1U < UART_OUTPUT_LEN)
    {
        model.uart_output[model.output_length++] = byte;
        model.uart_output[model.output_length] = '\0';
    }
}

// Simulate the UART repeatedly interrupting until its queue is empty.
static void native_uart_process_all(void)
{
    while (model.queue_count > 0U)
    {
        native_sercom5_dre_isr();
    }
}

// -------------------- Simulated print task -----------------------

static bool native_print_task_once(void)
{
    // Equivalent to ulTaskNotifyTake(pdTRUE, portMAX_DELAY).
    // In this model, false means the task would remain blocked.
    if (model.notifications == 0U)
    {
        return false;
    }

    // pdTRUE clears the accumulated notification count on exit.
    model.notifications = 0U;

    char message[32];

    uint32_t current_count = model.print_count++;

    (void)snprintf(
        message,
        sizeof(message),
        "toggle %lu\r\n",
        (unsigned long)current_count
    );

    return native_uart_puts(message);
}

// -------------------- Unity test setup ---------------------------

void setUp(void)
{
    memset(&model, 0, sizeof(model));
}

void tearDown(void)
{
}

// Test 1: timer ISR toggles the LED and generates notifications.
void test_timer_isr_toggles_led_and_notifies(void)
{
    TEST_ASSERT_FALSE(model.led_state);
    TEST_ASSERT_EQUAL_UINT32(0U, model.notifications);

    native_tc3_isr();

    TEST_ASSERT_TRUE(model.led_state);
    TEST_ASSERT_EQUAL_UINT32(1U, model.notifications);

    native_tc3_isr();

    TEST_ASSERT_FALSE(model.led_state);
    TEST_ASSERT_EQUAL_UINT32(2U, model.notifications);
}

// Test 2: print task has no work before a notification.
void test_print_task_waits_without_notification(void)
{
    TEST_ASSERT_FALSE(native_print_task_once());
    TEST_ASSERT_EQUAL_UINT32(0U, model.print_count);
    TEST_ASSERT_EQUAL_UINT32(0U, model.queue_count);
    TEST_ASSERT_EQUAL_STRING("", model.uart_output);
}

// Test 3: one timer event produces the expected UART message.
void test_one_tick_prints_toggle_zero(void)
{
    native_tc3_isr();

    TEST_ASSERT_TRUE(native_print_task_once());

    TEST_ASSERT_EQUAL_UINT32(0U, model.notifications);
    TEST_ASSERT_EQUAL_UINT32(10U, model.queue_count);

    // Drain the queued UART bytes via simulated DRE interrupts.
    native_uart_process_all();

    TEST_ASSERT_EQUAL_STRING("toggle 0\r\n", model.uart_output);
    TEST_ASSERT_EQUAL_UINT32(0U, model.queue_count);
    TEST_ASSERT_EQUAL_UINT32(1U, model.print_count);
}

// Test 4: subsequent timer events increment the message number.
void test_second_tick_prints_toggle_one(void)
{
    native_tc3_isr();
    TEST_ASSERT_TRUE(native_print_task_once());
    native_uart_process_all();

    native_tc3_isr();
    TEST_ASSERT_TRUE(native_print_task_once());
    native_uart_process_all();

    TEST_ASSERT_EQUAL_STRING(
        "toggle 0\r\ntoggle 1\r\n",
        model.uart_output
    );

    TEST_ASSERT_EQUAL_UINT32(2U, model.print_count);
}

// Test 5: accumulated notifications are cleared by one task take.
void test_multiple_pending_notifications_are_cleared(void)
{
    native_tc3_isr();
    native_tc3_isr();

    TEST_ASSERT_EQUAL_UINT32(2U, model.notifications);

    TEST_ASSERT_TRUE(native_print_task_once());

    // pdTRUE clears the accumulated notification count.
    TEST_ASSERT_EQUAL_UINT32(0U, model.notifications);

    native_uart_process_all();

    // The example task prints once per notification take, not
    // once for every accumulated notification.
    TEST_ASSERT_EQUAL_STRING("toggle 0\r\n", model.uart_output);
}

// -------------------- Native test runner ------------------------

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_timer_isr_toggles_led_and_notifies);
    RUN_TEST(test_print_task_waits_without_notification);
    RUN_TEST(test_one_tick_prints_toggle_zero);
    RUN_TEST(test_second_tick_prints_toggle_one);
    RUN_TEST(test_multiple_pending_notifications_are_cleared);

    return UNITY_END();
}
