#include "IO_handler.h"

#define PI 3.1415927
// constructors


// Deconstructor
IO_handler::IO_handler(float Ts):rc(PA_1), motor_M1(PB_PWM_M1,12),
                motor_M2(PB_PWM_M2,12),enable_motor_driver(PB_ENABLE_DCMOTORS)
    
 /*: di1(.0005,Ts),di2(.0005,Ts),counter1(PA_6, PC_7),
   indexpulse1(PA_8),index1(counter1,indexpulse1), counter2(PB_6, PB_7),indexpulse2(PB_4),index2(counter2,indexpulse2),
   i_enable(PC_4),laser_on(PB_0), i_des1(PA_5),i_des2(PA_4),pwm_out1(PB_10),pwm_out2(PA_15)*/
{
    val2pm1.setup(192,1712,-1.0f,1.0f);
    
}
IO_handler::~IO_handler() {} 

void IO_handler::read_encoders_calc_speed(void)
{
    
}
void IO_handler::read_rc()
{
    rc.processReceivedData();
    const RC::rc_pkg_t rc_pkg = rc.update();
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
