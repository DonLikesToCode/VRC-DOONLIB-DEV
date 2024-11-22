#pragma once

#include "vex.h"
#include <cmath>
#include "DOONLIB/DOON_PID.h"

using namespace doon_pid;

//////////////////////////////////////////////////////////////////////////////////////////////////////

PID::PID(double target, double currentPos, double errorMargin, double integralMargin) : 
    target(target), currentPos(currentPos), e_margin(errorMargin), i_margin(integralMargin),
    kp(0), ki(0), kd(0), intg(0), output(0), prevoutput(0), error(0), preverror(0) {}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void PID::setConsts(double sP, double sI, double sD, double error_margin, double integral_margin, double clamp_domain){
    kp = sP; // proportionate increase to error
    ki = sI; // integral area under the time graph
    kd = sD; // slope of the time graph
    e_margin = error_margin; // margin of error before stopping
    i_margin = integral_margin; // domain/margin required before integral is applied
    c_domain = clamp_domain; // domain motor outputs are clamped to
}


void PID::reset(){
    intg = 0;
    derv = 0;
    output = 0;
    currentPos = 0;
}


double PID::tick(){
    if (std::abs(error) > e_margin)
    {
        error = target - currentPos;
        derv = preverror - error;


        if (error < i_margin && error > -i_margin) {
            intg += error;
        } else {
            intg = 0;
        }


        output = (kp*error) + (ki*intg) + (kd*derv);


    } else {
        output = 0;
    }


    //clamping output values
    if (output > c_domain) output = c_domain;
    if (output < -c_domain) output = -c_domain;


    preverror = error;
    prevoutput = output;
   


    return output;
}


