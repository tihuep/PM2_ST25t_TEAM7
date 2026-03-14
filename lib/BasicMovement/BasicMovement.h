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

enum TurnDirection {
    CW,
    CCW
};

class BasicMovement
{
public:

    BasicMovement(DCMotor& motor1, DCMotor& motor2);
    virtual ~BasicMovement();

    void Forward(float speed);
    void Backward(float speed);
    void fullTurn(TurnDirection direction);
    void halfTurn(TurnDirection direction);
    void quarterTurn(TurnDirection direction);
    void stop();

private:
    DCMotor& motor_M1;
    DCMotor& motor_M2;
};

#endif /* BASIC_MOVEMENT_H_ */