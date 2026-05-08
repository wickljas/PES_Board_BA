#pragma once
/* class IO_handler
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "Enc_unwrap_scale.h"
#include "LinearCharacteristics.h"
#include "FastPWM.h"
#include "IIRfilter.h"
#include "RC.h"
#include "Motor.h"
#include "Servo.h"

class IO_handler
{
public:
    IO_handler(float Ts);        // default constructor
    virtual ~IO_handler();   // deconstructor
    void read_encoders_calc_speed(void);       // read both encoders and calculate speeds
    void enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    void set_motor_voltage(uint8_t,float);
    void drive(float forward_pm1, float steer_pm1); // forward/back and left/right steering

    void enable_servo(bool enable); // enable/disable servo output
    void set_servo_speed(float speed_pm1); // speed in [-1,+1], -1=backward, +1=forward
    void set_servo_position(float normalized); // position in [0,1], 0=0deg, 1=180deg for both servos
    void set_servo_position(uint8_t servo_id, float normalized); // position for a single servo
    void set_servo_positions(float normalized0, float normalized1); // set both servo positions
    void process_servo_buttons(float ch4_pm1, float ch7_pm1); // update servo positions from channel 4 and 7 toggles
    void process_servo_button(float ch7_pm1); // legacy single-button support
    void update_servo_position_joystick(float ch_joystick_pm1); // update servo position from joystick input, maintains position at rest

    void set_servo_degrees(float degrees); // set both servos to position in degrees
    float servo_start_degrees = 83.0f; // starting position in degrees
    float servo_start_d1_degrees = 92.0f;

    void read_rc();
    LinearCharacteristics val2pm1;
    float ch_pm1[10];
    RC rc;
private:
    IIRFilter di1;
    IIRFilter di2;
    Motor motor_M1;
    Motor motor_M2;
    DigitalOut enable_motor_driver;
    Servo servo_D0;
    Servo servo_D1;

    bool servo_toggle_state4;
    bool servo_toggle_state7;
    bool servo_button_last4;
    bool servo_button_last7;
    bool servo_button_initialized4;
    bool servo_button_initialized7;
    float servo_current_position0; // current servo D0 position [0, 1]
    float servo_current_position1; // current servo D1 position [0, 1]
    float m_Ts; // sampling time

    void map_channels();
   // DigitalIn big_button;         // Enable button an backside
    ///------------- Encoder -----------------------
   /* EncoderCounter counter1;    // initialize counter on PA_6 and PC_7
    InterruptIn indexpulse1;
    EncoderCounterIndex index1;   
    // ------------------------------------
    EncoderCounter counter2;    // initialize counter on PB_6 and PB_7
    InterruptIn indexpulse2;
    EncoderCounterIndex index2;    // 

    AnalogOut i_des1;           // desired current values
    AnalogOut i_des2;
    DigitalOut i_enable;
    DigitalOut laser_on;
    FastPWM pwm_out1;
    FastPWM pwm_out2;
    */
    //-------------------------------------
    LinearCharacteristics i2u;
    LinearCharacteristics u2i;
    LinearCharacteristics scale2pwm;
    //Enc_unwrap_scale uw1;
    //Enc_unwrap_scale uw2;
    float la;
};