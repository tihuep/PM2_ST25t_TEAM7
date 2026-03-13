/**
 * @file BasicMovement.h
 * @brief This file defines the BasicMovement class.
 * @author langhma2
 */

#ifndef BASIC_MOVEMENT_H_
#define BASIC_MOVEMENT_H_

#include "mbed.h"
#include "FastPWM.h"
#include "DCMotor.h"

class BasicMovement
{
public:
    explicit BasicMovement(float speed, bool direction);
    virtual ~BasicMovement();

    void Forward(float speed);
    void Backward(float speed);
    void fullTurn(float speed, bool direction);
    void halfTurn(float speed, bool direction);
    void quarterTurn(float speed, bool direction);
    void stop();

private:
    // private member variables and functions can be declared here
};
#endif /* BASIC_MOVEMENT_H_ */