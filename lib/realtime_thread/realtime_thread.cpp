#include "realtime_thread.h"

#include <chrono>
#include <cstdint>


using namespace Eigen;
using namespace std::chrono;

// contructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io_handler, float Ts)
    : m_Thread(osPriorityHigh1, 1024)
{
    m_Ts = Ts;                 // the sampling time
    m_IO_handler = io_handler; // a pointer to the io handler
    m_Timer.reset();
    m_Timer.start();
    start_loop(); // start the realtime loop
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        m_IO_handler->read_rc();
    // temporarily disable button-based servo switching
    // m_IO_handler->process_servo_buttons(m_IO_handler->ch_pm1[4], m_IO_handler->ch_pm1[7]);
        //m_IO_handler->enable_motors(true);
        //m_IO_handler->set_motor_voltage(0, 5.0f * m_IO_handler->ch_pm1[0]);
        //m_IO_handler->set_motor_voltage(1, 5.0f * m_IO_handler->ch_pm1[1]);

        // control DC motors with channel 0 as forward/backward and channel 1 as left/right steering
        m_IO_handler->drive(m_IO_handler->ch_pm1[0], m_IO_handler->ch_pm1[1]);
    }
}

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}
