#pragma once

#include <cstdint>
#include <mbed.h>

class Enc_unwrap_scale
{
public:
    Enc_unwrap_scale(){};   // default constructor
    Enc_unwrap_scale(uint32_t T, uint8_t bits,bool changesign);
    
    float operator()(int32_t inc) {
        return doStep(inc);
    }
    
    virtual     ~Enc_unwrap_scale();
    
    float       doStep(int32_t inc);

    void reset_overflows();
private:

        int32_t incPast;
        int32_t max_delta;
        double  inc2rad;
        int32_t incPerRev;
        int32_t num_overflows;
        bool chsgn;
};
