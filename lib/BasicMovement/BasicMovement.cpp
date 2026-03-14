#include "BasicMovement.h"

// Constructor
BasicMovement::BasicMovement(DCMotor& motor1, DCMotor& motor2)
    : motor_M1(motor1), motor_M2(motor2)
{
}

// Destructor
BasicMovement::~BasicMovement()
{
    // free resources if necessary
}

void BasicMovement::Forward(float speed)
{
    // code to move forward at the given speed
    motor_M1.setVelocity(motor_M1.getMaxVelocity() * speed); //Für Test nur 0.5
    motor_M2.setVelocity(motor_M2.getMaxVelocity() * speed);
}

void BasicMovement::Backward(float speed)
{
    // code to move backward at the given speed
    motor_M1.setVelocity(motor_M1.getMaxVelocity() * -speed); //Für Test nur 0.5
    motor_M2.setVelocity(motor_M2.getMaxVelocity() * -speed);
}

void BasicMovement::fullTurn(TurnDirection direction)
{
    // code to perform a full turn at the given speed and direction
    if(direction==TurnDirection::CW) {   // Clockwise
        motor_M1.setRotation(1.0f);             //Für Test nur 0.5
        motor_M2.setRotation(-1.0f);
    } else {                                    // Counterclockwise
        motor_M1.setRotation(-1.0f);            //Für Test nur 0.5
        motor_M2.setRotation(1.0f);
    }
}

void BasicMovement::halfTurn(TurnDirection direction)
{
    if(direction == CW) {
        motor_M1.setRotation(0.5f);
        motor_M2.setRotation(-0.5f);
    }
    else {
        motor_M1.setRotation(-0.5f);
        motor_M2.setRotation(0.5f);
    }
}

void BasicMovement::quarterTurn(TurnDirection direction)
{
    // code to perform a quarter turn at the given speed and direction
        if(direction == CW) {
        motor_M1.setRotation(0.25f);
        motor_M2.setRotation(-0.25f);
    }
    else {
        motor_M1.setRotation(-0.25f);
        motor_M2.setRotation(0.25f);
    }
}

void BasicMovement::stop()
{
    // code to stop the movement
    motor_M1.setVelocity(0.0f);
    motor_M2.setVelocity(0.0f);
}