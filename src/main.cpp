#include "mbed.h"
#include "IO_handler.h"
#include "realtime_thread.h"

using namespace std::chrono;

float Ts = 1.0f / 1e2f;                    // 10 kHz, 20 kHz was not working with GPA + UART communication

int main()
{
    // Input/Output handling
    IO_handler io_handler(Ts);

    // Communication
    BufferedSerial uart_serial(USBTX, USBRX, 115200);                   // leave this blocking!
    //uart_comm_thread_send uart_com_send(&io_handler, &uart_serial, .01f); // send communication thread
    //uart_comm_thread_receive uart_com_receive(&uart_serial, .01f);      // receive communication thread

    // Real-Time Thread
    realtime_thread rt_thread(&io_handler, Ts);

    Timer servo_timer;
    servo_timer.start();
    bool toggle_position = false;

    while (true) {
        if (servo_timer.elapsed_time() >= 5s) {
            servo_timer.reset();
            toggle_position = !toggle_position;

            float target_d0 = toggle_position ? (io_handler.servo_start_degrees + 20.0f) : io_handler.servo_start_degrees;
            float target_d1 = toggle_position ? (io_handler.servo_start_d1_degrees + 20.0f) : io_handler.servo_start_d1_degrees;

            io_handler.set_servo_position(0, (180.0f - target_d0) / 180.0f);
            io_handler.set_servo_position(1, target_d1 / 180.0f);
        }

        // Print channel data every 500ms
        printf("valid: %d, period: %ld, failsafe: %d\n", io_handler.rc.m_sbus.isPkgValid(), io_handler.rc.m_sbus.getPeriod(), io_handler.rc.m_sbus.isFailSafe());
        printf("ch0: %d, ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch7: %d, ch8: %d, ch9: %d\n",
                   io_handler.rc.m_sbus.getChannel(0),
                   io_handler.rc.m_sbus.getChannel(1),
                   io_handler.rc.m_sbus.getChannel(2),
                   io_handler.rc.m_sbus.getChannel(3),
                   io_handler.rc.m_sbus.getChannel(4),
                   io_handler.rc.m_sbus.getChannel(5),
                   io_handler.rc.m_sbus.getChannel(6),
                   io_handler.rc.m_sbus.getChannel(7),
                   io_handler.rc.m_sbus.getChannel(8),
                   io_handler.rc.m_sbus.getChannel(9));
        printf("lost: %d\r\n", io_handler.rc.m_sbus.getNumOfLostFrames());

        ThisThread::sleep_for(500ms);
    }
}

