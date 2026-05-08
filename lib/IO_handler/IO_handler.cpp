#include "IO_handler.h"
#include "config.h"

#define PI 3.1415927

static constexpr float SERVO_POSITION_SAFE_MIN = 0.02f;
static constexpr float SERVO_POSITION_SAFE_MAX = 0.98f;
static constexpr float SERVO_POSITION_30 = 30.0f / 180.0f;
static constexpr float SERVO_POSITION_60 = 60.0f / 180.0f;

static float clamp_servo_position(float normalized)
{
    if (normalized < SERVO_POSITION_SAFE_MIN) return SERVO_POSITION_SAFE_MIN;
    if (normalized > SERVO_POSITION_SAFE_MAX) return SERVO_POSITION_SAFE_MAX;
    return normalized;
}

// constructors


// Deconstructor
IO_handler::IO_handler(float Ts):rc(PA_3), motor_M1(PB_PWM_M1,12),
                motor_M2(PB_PWM_M2,12),enable_motor_driver(PB_ENABLE_DCMOTORS),
                servo_D0(PB_D0), servo_D1(PB_D1)
    
 /*: di1(.0005,Ts),di2(.0005,Ts),counter1(PA_6, PC_7),
   indexpulse1(PA_8),index1(counter1,indexpulse1), counter2(PB_6, PB_7),indexpulse2(PB_4),index2(counter2,indexpulse2),
   i_enable(PC_4),laser_on(PB_0), i_des1(PA_5),i_des2(PA_4),pwm_out1(PB_10),pwm_out2(PA_15)*/
{
    val2pm1.setup(192,1712,-1.0f,1.0f);

    // Use standard positional servo range
    servo_D0.calibratePulseMinMax(0.0150f, 0.1150f); // BL815H: original range
    servo_D0.setMaxVelocity(0.375f);    // Max speed: 1/4 of original (higher = faster)
    servo_D0.setMaxAcceleration(0.75f);
    servo_D1.calibratePulseMinMax(0.05f, 0.1f); // HS-0385TH: 1ms to 2ms
    servo_D1.setMaxVelocity(0.1f);     // Slower for analog servo
    servo_D1.setMaxAcceleration(0.2f);

    servo_current_position0 = clamp_servo_position((180.0f - servo_start_degrees) / 180.0f); // reversed for D0
    servo_current_position1 = clamp_servo_position(servo_start_d1_degrees / 180.0f); // start at servo_start_degrees movement
    servo_D0.enable(servo_current_position0);
    servo_D1.enable(servo_current_position1);

    // disable motors on startup
    enable_motors(false);

    // servo toggle states for channel 4 and 7
    servo_toggle_state4 = true;
    servo_toggle_state7 = true;
    servo_button_last4 = false;
    servo_button_last7 = false;
    servo_button_initialized4 = false;
    servo_button_initialized7 = false;
    m_Ts = Ts; // store sampling time
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

void IO_handler::drive(float forward_pm1, float steer_pm1)
{
    if (forward_pm1 > 1.0f) forward_pm1 = 1.0f;
    if (forward_pm1 < -1.0f) forward_pm1 = -1.0f;
    if (steer_pm1 > 1.0f) steer_pm1 = 1.0f;
    if (steer_pm1 < -1.0f) steer_pm1 = -1.0f;

    // Deadband around neutral so the motors stay still when the sticks are centered
    const float deadband = 0.05f;
    if (fabs(forward_pm1) < deadband) forward_pm1 = 0.0f;
    if (fabs(steer_pm1) < deadband) steer_pm1 = 0.0f;

    const float VMAX = 12.0f;
    float base = forward_pm1 * VMAX;
    float turn = steer_pm1 * VMAX;

    float left_voltage = base + turn;
    float right_voltage = base - turn;

    if (left_voltage > VMAX) left_voltage = VMAX;
    if (left_voltage < -VMAX) left_voltage = -VMAX;
    if (right_voltage > VMAX) right_voltage = VMAX;
    if (right_voltage < -VMAX) right_voltage = -VMAX;

    if (left_voltage == 0.0f && right_voltage == 0.0f) {
        set_motor_voltage(0, 0.0f);
        set_motor_voltage(1, 0.0f);
        enable_motors(false);
    } else {
        enable_motors(true);
        // Compensate for different gear ratios: M1 is 78:1, M2 is 100:1
        // M1 runs faster with same voltage, so reduce its voltage by factor 78/100
        set_motor_voltage(0, left_voltage * 0.78f);
        set_motor_voltage(1, right_voltage);
    }
}

void IO_handler::enable_servo(bool enable)
{
    if (enable) {
        if (!servo_D0.isEnabled()) {
            servo_D0.enable(servo_current_position0);
        }
        if (!servo_D1.isEnabled()) {
            servo_D1.enable(servo_current_position1);
        }
    } else {
        servo_D0.disable();
        servo_D1.disable();
    }
}

void IO_handler::set_servo_position(uint8_t servo_id, float normalized)
{
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < 0.0f) normalized = 0.0f;
    normalized = clamp_servo_position(normalized);

    Servo *target_servo = (servo_id == 1) ? &servo_D1 : &servo_D0;
    float *target_position = (servo_id == 1) ? &servo_current_position1 : &servo_current_position0;
    *target_position = normalized;

    if (!target_servo->isEnabled()) {
        target_servo->enable(normalized);
    } else {
        target_servo->setPulseWidth(normalized);
    }
}

void IO_handler::set_servo_positions(float normalized0, float normalized1)
{
    set_servo_position(0, normalized0);
    set_servo_position(1, normalized1);
}

void IO_handler::set_servo_position(float normalized)
{
    set_servo_positions(normalized, normalized);
}

void IO_handler::set_servo_degrees(float degrees)
{
    float normalized = clamp_servo_position(degrees / 180.0f);
    set_servo_position(normalized);
}

void IO_handler::process_servo_buttons(float ch4_pm1, float ch7_pm1)
{
    const float threshold = 0.2f; // hysteresis threshold to prevent flickering
    bool button4 = ch4_pm1 > threshold;
    if (!servo_button_initialized4) {
        servo_button_last4 = button4;
        servo_button_initialized4 = true;
    }
    if (button4 && !servo_button_last4) {
        servo_toggle_state4 = !servo_toggle_state4;
        float deg4 = servo_toggle_state4 ? 104.0f : 84.0f;
        set_servo_position(0, (180.0f - deg4) / 180.0f); // reversed for D0
    }
    servo_button_last4 = button4;

    bool button7 = ch7_pm1 > threshold;
    if (!servo_button_initialized7) {
        servo_button_last7 = button7;
        servo_button_initialized7 = true;
    }
    if (button7 && !servo_button_last7) {
        servo_toggle_state7 = !servo_toggle_state7;
        float deg7 = servo_toggle_state7 ? 60.0f : 30.0f;
        set_servo_position(1, deg7 / 180.0f);
    }
    servo_button_last7 = button7;
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
    servo_current_position0 = clamp_servo_position(norm);

    if (!servo_D0.isEnabled()) {
        servo_D0.enable(servo_current_position0);
    } else {
        servo_D0.setPulseWidth(servo_current_position0);
    }
}

void IO_handler::process_servo_button(float ch7_pm1)
{
    bool mode7 = ch7_pm1 > 0.0f;
    set_servo_position(1, mode7 ? SERVO_POSITION_60 : SERVO_POSITION_30);
}

void IO_handler::update_servo_position_joystick(float ch_joystick_pm1)
{
    // Constrain joystick input to [-1, 1]
    if (ch_joystick_pm1 > 1.0f) ch_joystick_pm1 = 1.0f;
    if (ch_joystick_pm1 < -1.0f) ch_joystick_pm1 = -1.0f;

    // Deadband around center to avoid jitter when joystick is near neutral
    const float deadband = 0.05f;
    if (fabs(ch_joystick_pm1) < deadband) {
        ch_joystick_pm1 = 0.0f;
    }

    // Increment/decrement servo position based on joystick input
    // Positive joystick (up) -> increase position
    // Negative joystick (down) -> decrease position
    // The rate of change is proportional to the joystick deflection (integrate velocity)
    const float position_rate = 0.375f; // normalized position change per second at full stick (1/4 speed)
    servo_current_position0 += ch_joystick_pm1 * position_rate * m_Ts;

    // Constrain position to [0, 1]
    if (servo_current_position0 > 1.0f) servo_current_position0 = 1.0f;
    if (servo_current_position0 < 0.0f) servo_current_position0 = 0.0f;

    servo_current_position0 = clamp_servo_position(servo_current_position0);

    // Keep servo enabled and set to current position
    if (!servo_D0.isEnabled()) {
        servo_D0.enable(servo_current_position0);
    } else {
        servo_D0.setPulseWidth(servo_current_position0);
    }
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
