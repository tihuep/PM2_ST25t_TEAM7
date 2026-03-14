#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"
#include "Servo.h"
#include "WS2812SPI.h"
#include "BasicMovement.h"

#define NUM_LEDS 8

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

//-----------------------------------------------------------------------------------------------------------------------------------------                                        
//LEDs
    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// DCMotors

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                    // 6.0f V if you only use one battery pack
    const float gear_ratio = 78.125f; // gear ratio
    const float kn = 180.0f / 12.0f;  // motor constant [rpm/V]

    // motor M1
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M1.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);

    // motor M2
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

    BasicMovement basic_movement(motor_M1, motor_M2); // create BasicMovement object to easily command the robot to move forward, backward and turn
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Servos
    Servo servo_Low_D0(PB_D0);
    Servo servo_High_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // servo Low: Insert servo name e.g. Futaba S3003
    float servo_Low_D0_ang_min = 0.0150f; // carefull, these values might differ from servo to servo
    float servo_Low_D0_ang_max = 0.1150f;
    //Servo High: Insert servo name e.g. Futaba S3003
    float servo_High_D1_ang_min = 0.0325f;
    float servo_High_D1_ang_max = 0.1175f;

    //To be calibrated
    servo_Low_D0.calibratePulseMinMax(servo_Low_D0_ang_min, servo_Low_D0_ang_max);
    servo_High_D1.calibratePulseMinMax(servo_High_D1_ang_min, servo_High_D1_ang_max);

    // default acceleration of the servo motion profile is 1.0e6f
    //enable if blocks fall off
    //servo_Low_D0.setMaxAcceleration(0.3f);
    //servo_High_D1.setMaxAcceleration(0.3f);
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Ultrasonic Sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Line Array Sensor


//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Color Sensor


//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// RGB LED strip
    WS2812SPI rgbleds(D11, NUM_LEDS); // MOSI pin, number of LEDs

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// enum
    // set up states for state machine
    enum RobotState {
        INITIAL,
        SLEEP,
        FORWARD,
        BACKWARD,
        HALF_TURN,
        FULL_TURN,
        POSITIONING,
        PICK_UP,
        DROP_OFF,
        FINISHED,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    /*enum TurnDirection {
        CW,
        CCW
    } turn_direction = TurnDirection::CW;*/

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// other variables
    bool package_height = 0; // 0 -> low, 1 -> high

//-----------------------------------------------------------------------------------------------------------------------------------------

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---

            //read ultrasonic sensor once per cycle, further code should use the value of us_distance_cm, so that the ultrasonic sensor is not read multiple times per cycle
            const float us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f)
                us_distance_cm = us_distance_cm_candidate;

            // enable the servos
            if (!servo_Low_D0.isEnabled())
                servo_Low_D0.enable();
            if (!servo_High_D1.isEnabled())
                servo_High_D1.enable();

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::SLEEP: {
                    // wait for the signal from the user, so to run the process
                    // that is triggered by clicking the mechanical button
                    // then go to the FORWARD state
                    //if (mechanical_button.read())
                        robot_state = RobotState::FINISHED; //FOR TEST ONLY, CHANGE TO Linefollow or smth

                    break;
                }
                /*case RobotState::FORWARD: {
                    motor_M1.setVelocity(motor_M1.getMaxVelocity() * 0.5f); //Für Test nur 0.5
                    motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5f);

                    //IF um Linie zu erkennen und State zu wechseln

                    break;
                }
                case RobotState::BACKWARD: {
                    motor_M1.setVelocity(motor_M1.getMaxVelocity() * -0.5f); //Für Test nur 0.5
                    motor_M2.setVelocity(motor_M2.getMaxVelocity() * -0.5f);

                    //IF um Linie zu erkennen und State zu wechseln

                    break;
                }
                case RobotState::HALF_TURN: {
                    if(turn_direction == TurnDirection::CW) {   // Clockwise
                        motor_M1.setRotation(0.5f);             //Für Test nur 0.5
                        motor_M2.setRotation(-0.5f);
                    } else {                                    // Counterclockwise
                        motor_M1.setRotation(-0.5f);            //Für Test nur 0.5
                        motor_M2.setRotation(0.5f);
                    }

                    break;
                }
                case RobotState::FULL_TURN: {
                    if(turn_direction == TurnDirection::CW) {   // Clockwise
                        motor_M1.setRotation(1.0f);             //Für Test nur 0.5
                        motor_M2.setRotation(-1.0f);
                    } else {                                    // Counterclockwise
                        motor_M1.setRotation(-1.0f);            //Für Test nur 0.5
                        motor_M2.setRotation(1.0f);
                    }

                    break;
                }*/
                case RobotState::POSITIONING: {
                    
                    
                    break;
                }
                case RobotState::PICK_UP: {
                    if(package_height == 0) {       // low
                        //Rotate arm out
                        servo_Low_D0.setPulseWidth(1.0f); // Mechanically mount arm correctly

                        //Delay if necessary
                        //This_thread::sleep_for(chrono::milliseconds(1000)); // adjust the delay time as needed

                        //Rotate arm in
                        servo_Low_D0.setPulseWidth(0.0f); // Mechanically mount arm correctly

                    } else {                        // high
                        //Rotate arm out
                        servo_High_D1.setPulseWidth(1.0f); // Mechanically mount arm correctly

                        //Delay if necessary
                        //This_thread::sleep_for(chrono::milliseconds(1000)); // adjust the delay time as needed

                        //Rotate arm in
                        servo_High_D1.setPulseWidth(0.0f); // Mechanically mount arm correctly

                    }                 
                    break;
                }
                case RobotState::DROP_OFF: {
                    if(package_height == 0) {       // low
                        //Rotate arm out
                        servo_Low_D0.setPulseWidth(1.0f); // Mechanically mount arm correctly

                        //Delay if necessary
                        //This_thread::sleep_for(chrono::milliseconds(1000)); // adjust the delay time as needed

                        //Rotate arm in
                        servo_Low_D0.setPulseWidth(0.0f); // Mechanically mount arm correctly

                    } else {                        // high
                        //Rotate arm out
                        servo_High_D1.setPulseWidth(1.0f); // Mechanically mount arm correctly

                        //Delay if necessary
                        //This_thread::sleep_for(chrono::milliseconds(1000)); // adjust the delay time as needed

                        //Rotate arm in
                        servo_High_D1.setPulseWidth(0.0f); // Mechanically mount arm correctly

                    }                     
                    break;
                }
                case RobotState::FINISHED: {
                    
                    printf("VICTORY\n");
                    basic_movement.fullTurn(TurnDirection::CW); // do a full turn for the victory dance, adjust direction and type of turn as you like 
                    static int hue = 0;
                    rgbleds.setBrightness(127); // set brightness to maximum for the victory dance

                    for (int i = 0; i < NUM_LEDS; i++) {
                        //rgbleds.setPixelColor(i, rand()%256, rand()%256, rand()%256); // random color for each LED, more like disco
                        rgbleds.setPixelColor(i,                                        // rainbow effect, hue changes over time, each LED has a different phase shift
                                             (sin(hue * 0.1f) + 1) * 127,               // red channel
                                             (sin(hue * 0.1f + 2) + 1) * 127,           // green channel
                                             (sin(hue * 0.1f + 4) + 1) * 127);          // blue channel
                    }
                    rgbleds.show();
                    hue++;
                    
                    break;
                }
                case RobotState::EMERGENCY: {

                    static int counter = 0;
                    static bool on = false;
                    rgbleds.setBrightness(127); // set brightness to maximum for the emergency signal

                    counter++;

                    if(counter > 25) // ~500 ms (25 × 20 ms loop)
                    {
                        counter = 0;
                        on = !on;

                        if(on)
                        {
                            for(int i = 0; i < NUM_LEDS; i++)
                            {
                                rgbleds.setPixelColor(i, 255, 0, 0); // red color to indicate emergency
                            }
                        }
                        else
                        {
                            rgbleds.clear();
                        }

                        rgbleds.show();
                    }
                    break;
                }
                default: {

                    break; // do nothing
                }
            }


            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                motor_M1.setMotionPlannerPosition(0.0f);
                motor_M1.setMotionPlannerVelocity(0.0f);
                motor_M1.enableMotionPlanner();
                motor_M2.setMotionPlannerPosition(0.0f);
                motor_M2.setMotionPlannerVelocity(0.0f);
                motor_M2.enableMotionPlanner();
                servo_Low_D0.disable();
                servo_High_D1.disable(); 
                rgbleds.clear();
                rgbleds.show();
                robot_state = RobotState::INITIAL;

            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}