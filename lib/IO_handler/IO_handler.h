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

class IO_handler
{
public:
    IO_handler(float Ts);        // default constructor
    virtual ~IO_handler();   // deconstructor
    void read_encoders_calc_speed(void);       // read both encoders and calculate speeds
    void enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    void set_motor_voltage(uint8_t,float);
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