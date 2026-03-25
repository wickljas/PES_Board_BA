#include "IO_handler.h"
#include "config.h"

#define PI 3.1415927
// constructors


// Deconstructor
IO_handler::IO_handler(float Ts):rc(PA_1), motor_M1(PB_PWM_M1,12),
                motor_M2(PB_PWM_M2,12),enable_motor_driver(PB_ENABLE_DCMOTORS),
                servo_D0(PB_D0)
    
 /*: di1(.0005,Ts),di2(.0005,Ts),counter1(PA_6, PC_7),
   indexpulse1(PA_8),index1(counter1,indexpulse1), counter2(PB_6, PB_7),indexpulse2(PB_4),index2(counter2,indexpulse2),
   i_enable(PC_4),laser_on(PB_0), i_des1(PA_5),i_des2(PA_4),pwm_out1(PB_10),pwm_out2(PA_15)*/
{
    val2pm1.setup(192,1712,-1.0f,1.0f);

    // Use standard positional servo range
    servo_D0.calibratePulseMinMax(0.0150f, 0.1150f);
    servo_D0.setMaxVelocity(1.5f);    // Max speed: adjust this value (higher = faster)
    servo_D0.setMaxAcceleration(3.0f);
    servo_D0.enable(0.0f); // startup at 0°

    // toggle engine for channel7 button behavior
    servo_toggle_state = false;
    servo_button_last = false;
    servo_button_initialized = false;
}
IO_handler::~IO_handler() {} 

void IO_handler::read_encoders_calc_speed(void)
{
    
}
void IO_handler::read_rc()
{
    rc.processReceivedData();
    rc.update();
    map_channels();
}

void IO_handler::enable_motors(bool enable)
{
    enable_motor_driver = enable;
}
void IO_handler::set_motor_voltage(uint8_t m,float voltage)
{
    if (m == 0) {
        motor_M1.setVoltage(voltage);
    } else if (m == 1) {
        motor_M2.setVoltage(voltage);
    }
}

void IO_handler::enable_servo(bool enable)
{
    if (enable) {
        if (!servo_D0.isEnabled()) {
            servo_D0.enable(0.5f);
        }
    } else {
        servo_D0.disable();
    }
}

void IO_handler::set_servo_speed(float speed_pm1)
{
    if (speed_pm1 > 1.0f) speed_pm1 = 1.0f;
    if (speed_pm1 < -1.0f) speed_pm1 = -1.0f;

    // deadband around center to avoid jitter when RC is near neutral
    const float deadband = 0.05f;
    if (fabs(speed_pm1) < deadband) {
        speed_pm1 = 0.0f;
    }

    float norm = 0.5f + 0.5f * speed_pm1;

    if (!servo_D0.isEnabled()) {
        servo_D0.enable(norm);
    } else {
        servo_D0.setPulseWidth(norm);
    }
}

void IO_handler::set_servo_position(float normalized)
{
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < 0.0f) normalized = 0.0f;

    if (!servo_D0.isEnabled()) {
        servo_D0.enable(normalized);
    } else {
        servo_D0.setPulseWidth(normalized);
    }
}

void IO_handler::process_servo_button(float ch7_pm1)
{
    bool current = ch7_pm1 > 0.0f; // channel mapped -1..1: false=192, true=1792

    if (!servo_button_initialized) {
        servo_button_last = current;
        servo_button_initialized = true;
        return;
    }

    if (current != servo_button_last) {
        servo_toggle_state = !servo_toggle_state;
        float target = servo_toggle_state ? 1.0f : 0.0f;
        set_servo_position(target);
    }

    servo_button_last = current;
}

void IO_handler::map_channels()
{
    ch_pm1[0] = val2pm1(rc.m_sbus.getChannel(0));
    ch_pm1[1] = val2pm1(rc.m_sbus.getChannel(1));
    ch_pm1[2] = val2pm1(rc.m_sbus.getChannel(2));
    ch_pm1[3] = val2pm1(rc.m_sbus.getChannel(3));
    ch_pm1[4] = val2pm1(rc.m_sbus.getChannel(4));
    ch_pm1[5] = val2pm1(rc.m_sbus.getChannel(5));
    ch_pm1[6] = val2pm1(rc.m_sbus.getChannel(6));
    ch_pm1[7] = val2pm1(rc.m_sbus.getChannel(7));
    ch_pm1[8] = val2pm1(rc.m_sbus.getChannel(8));
    ch_pm1[9] = val2pm1(rc.m_sbus.getChannel(9));
}   
