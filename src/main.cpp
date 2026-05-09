#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "LineFollower.h"

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
    const int main_task_period_ms = 10; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

//-----------------------------------------------------------------------------------------------------------------------------------------                                        
//LEDs
    // led on nucleo board
    DigitalOut user_led(LED1);

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// DCMotors

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                    // 6.0f V if you only use one battery pack
    const float gear_ratio = 78.125f; // gear ratio
    const float kn = 180.0f / 12.0f;  // motor constant [rpm/V]

    // motor M1 (right motor)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    //motor_M1.enableMotionPlanner();      //do not use with LineFollower
    // limit max. acceleration to half of the default acceleration
    //motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.1f);

    // motor M2 (left motor)
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    //motor_M2.enableMotionPlanner();       //do not use with LineFollower
    // limit max. acceleration to half of the default acceleration
    //motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.1f);

    //sets initial gwaggling speeds for pick up and drop off action
    float left_speed_gwaggli = -0.1;
    float right_speed_gwaggli = 0.1;

    //velocity factor to control the robot speed in diferent states
    double velocity_factor = 1.0f; //velocity factor for line following, can be reduced if the robot is not able to follow the line at high speed

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Servos
    Servo servo_Arm_D0(PB_D0);
    Servo servo_Truelli_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // servo Low: Insert servo name e.g. Futaba S3003
    float servo_Arm_D0_ang_min = 0.032f;
    float servo_Arm_D0_ang_max = 0.085f;
    //Servo High: Insert servo name e.g. Futaba S3003
    float servo_Truelli_D1_ang_min = 0.031f;
    float servo_Truelli_D1_ang_max = 0.1175f;

    //Apply calibrations
    servo_Arm_D0.calibratePulseMinMax(servo_Arm_D0_ang_min, servo_Arm_D0_ang_max);
    servo_Truelli_D1.calibratePulseMinMax(servo_Truelli_D1_ang_min, servo_Truelli_D1_ang_max);

    // default acceleration of the servo motion profile is 1.0e6f
    //enable if blocks fall off
    //servo_Arm_D0.setMaxAcceleration(1.0f);
    //servo_Truelli_D1.setMaxAcceleration(1.0f);
    //servo_Arm_D0.setMaxVelocity(0.9f); // limit velocity of the servo

    //initializes truelli state
    int truelli_state = 0;

//-----------------------------------------------------------------------------------------------------------------------------------------
// Line Array Sensor
    
    const float d_wheel = 0.05f; // wheel diameter in meters
    const float b_wheel = 0.158f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.08f; // distance from wheel axis to leds on sensor bar / array in meters
    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity()*0.6);

    const float Kp = 1.9f * 2.0f;
    const float Kp_nl = 1.3f * 17.0f;

    lineFollower.setRotationalVelocityControllerGains(Kp, Kp_nl);

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// TCS3200 color sensor
    ColorSensor color_sensor(PA_8);
    color_sensor.switchLed(ON);

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// enum
    // set up states for state machine
    enum RobotState {
        INITIAL,
        LEAVE_GARAGE,
        LINEFOLLOW,
        PICK_DROP,
        CREEP,
        FINISHED,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Variables for packages
    bool package_height = 1; // 0 -> low, 1 -> high
    int color_detected = -1; // 0 -> red, 1 -> blue, 2 -> green, 3 -> yellow

    //array for storing, which packages are picked up
    bool package_storage[4] = {false,false,false,false}; //red, blue, green, yellow

//-----------------------------------------------------------------------------------------------------------------------------------------

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---

            //ONLY FOR CALIBRATION PURPOSES
            //Read out the values for each channel on black and white background
            //Put the values into ColorSensor.cpp m_reference_white and m_reference_black
            //printf("R: %.2f Hz\t G: %.2f Hz\t B: %.2f Hz\t C: %.2f Hz\n", color_sensor.readColor()[0], color_sensor.readColor()[1], color_sensor.readColor()[2], color_sensor.readColor()[3]);

            //Read out and print the color detected by the color sensor
            int color = color_sensor.getColor();
            printf("%s \n", color_sensor.getColorString(color));
            
            // enable the servos
            if (!servo_Arm_D0.isEnabled())
                servo_Arm_D0.enable(0.0f); // enable with 0.0f pulse width, so that the arm is in the initial position
            if (!servo_Truelli_D1.isEnabled())
                servo_Truelli_D1.enable(0.0f); // enable with 0.0f pulse width, so that the arm is in the initial position

            // state machine
            switch (robot_state) {

                case RobotState::INITIAL: {
                    printf("initial\n");

                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    servo_Arm_D0.setPulseWidth(1.0f); //high
                    servo_Truelli_D1.setPulseWidth(0.0f); //high
                    truelli_state = 1; //high

                    //switch to leave garage state
                    robot_state = RobotState::LEAVE_GARAGE;

                    break;
                }

                case RobotState::LEAVE_GARAGE: {
                    printf("leave garage\n");

                    //hardcoded movement to leave the garage
                    motor_M1.setRotation(1.5f); //Right Motor forward one rotation
                    motor_M2.setRotation(0.3f); //Left Motor forward half rotation

                    if(motor_M1.getRotation() >= 1.49f && motor_M2.getRotation() >= 0.29f){ //if movement was completed, 
                        //switch to line following state
                        robot_state = RobotState::LINEFOLLOW; // switch to line following state
                    }

                    break;
                }

                case RobotState::LINEFOLLOW: {
                    printf("linefollow\n");

                    //initialize static counters for velocity factor and cross-line handling
                    static int counter_follow = 0;
                    static int counter_cross_line = 0;
                    static int counter_color = 0;
                    static int invalid_color_counter = 0;

                    //if all packages are either picked up or all are delivered
                    if((package_storage[0] && package_storage[1] && package_storage[2] && package_storage[3]) || (!package_storage[0] && !package_storage[1] && !package_storage[2] && !package_storage[3])){
                        //make robot faster
                        velocity_factor = 1.0f;
                        counter_follow++;

                        //if all packages are picked up, after the tunnel, slow down
                        if(counter_follow > 5500/main_task_period_ms && (package_storage[0] && package_storage[1] && package_storage[2] && package_storage[3])){                   
                            velocity_factor = 0.35f;
                            printf("SLOW AFTER TUNNEL\n");
                        }

                        //if all packages are delivered, at the start, slow down
                        if(counter_follow > 1800/main_task_period_ms && (!package_storage[0] && !package_storage[1] && !package_storage[2] && !package_storage[3])){ // at start
                            velocity_factor = 0.35f;
                        }
                    } else { //if not all packages are picked up or delivered
                        //slow down, to be able to pick up or drop off the packages more reliably
                        velocity_factor = 0.35f;
                        counter_follow = 0;
                    }

                    //normal line following
                    //set motor speed to linefollower calculations
                    motor_M1.setVelocity(lineFollower.getRightWheelVelocity()*velocity_factor); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(lineFollower.getLeftWheelVelocity()*velocity_factor);  // set a desired speed for speed controlled dc motors M2

                    //checks if the line is wider than normal on both sides
                    if (lineFollower.getMeanFourAvgBitsCenter() > 0.60){ // if in avg a bit more than 2 center LEDs are on, we are probably at a cross-line
                        //turn off the motors after a very short time to stop in the middle of the cross-line
                        if (counter_cross_line > 10/main_task_period_ms){
                            motor_M1.setVelocity(0);
                            motor_M2.setVelocity(0);
                            velocity_factor = 1.0f;
                            counter_follow = 0;
                        }else{
                            counter_cross_line++;
                        }
                        
                        //after breaking, print out the color detection counters
                        printf("invalid_color_counter: %d\n", invalid_color_counter);    
                        printf("break %d\n", counter_color);
                        
                        //after a short time, read out the color
                        if (counter_color > 200/main_task_period_ms){
                            //set the positioning variables according to the color
                            switch (color) {
                                case 3: //RED
                                    package_height = 0; //low
                                    color_detected = 0;
                                    robot_state = RobotState::PICK_DROP;
                                    invalid_color_counter = 0;
                                    counter_color = 0;  
                                    break;
                                case 8: //MAGENTA
                                case 7: //BLUE
                                    package_height = 1; //high
                                    color_detected = 1;
                                    robot_state = RobotState::PICK_DROP;
                                    invalid_color_counter = 0;
                                    counter_color = 0;  
                                    break;
                                case 5: //GREEN
                                    package_height = 0; //low
                                    color_detected = 2;
                                    robot_state = RobotState::PICK_DROP;
                                    invalid_color_counter = 0;
                                    counter_color = 0;  
                                    break;
                                case 4: //YELLOW
                                    package_height = 1; //high
                                    color_detected = 3;
                                    robot_state = RobotState::PICK_DROP;
                                    invalid_color_counter = 0;
                                    counter_color = 0;  
                                    break;
                                default: //UNKNOWN, WHITE, BLACK
                                    //If an invalid color was detected for max. 50 times, go to emergency
                                    //If the 50 attempts are not over, try again
                                    if (invalid_color_counter > 50) {
                                        robot_state = RobotState::EMERGENCY;
                                        counter_color = 0;
                                        invalid_color_counter = 0;
                                        printf("default\n");
                                    }
                                    invalid_color_counter++;
                                    break;
                            }
                        }
                        counter_color++;
                    }
                    break;
                }

                case RobotState::PICK_DROP: {
                    printf("pick drop\n");

                    //main counter to handle the timing of the different phases of the pick up and drop off action
                    static int counter = 0;
                    
                    //setting all the time values for the different phases of the pick up and drop off action
                    int truelli_time = 400;
                    int turn_down_time = 400;
                    int gwaggli_time = 700;
                    int turn_up_time = 200;
                    int repeat_time = 4000;

                    //turn the truelli if necessary
                    if (package_height == truelli_state){
                        truelli_time = 0;
                    }else {
                        if (package_height == 0) {
                            servo_Truelli_D1.setPulseWidth(1.0f); //low
                            //set the state of the truelli after all pick drop actions, so that it does not turn again during the pick drop action
                            if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) truelli_state = 0; // low
                        } else {
                            servo_Truelli_D1.setPulseWidth(0.0f); //high
                            //set the state of the truelli after all pick drop actions, so that it does not turn again during the pick drop action
                            if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) truelli_state = 1; //high
                        }
                    }

                    //lower the arm
                    if(counter > (truelli_time)/main_task_period_ms && counter < (truelli_time + turn_down_time)/main_task_period_ms) { 
                        servo_Arm_D0.setPulseWidth(0.0f); //Lower arm
                    }

                    //gwaggli while picking up or dropping off the package
                    if (counter > (truelli_time + turn_down_time)/main_task_period_ms && counter < (truelli_time + turn_down_time + gwaggli_time)/main_task_period_ms) {
                        
                        //Lift the arm slightly while gwaggling after 100ms
                        if (counter > (truelli_time + turn_down_time + 100)/main_task_period_ms){
                            servo_Arm_D0.setPulseWidth(0.03f); //lift sligthly
                        }    
                        
                        //gwaggli (direction changes every 200ms)
                        if ((counter * main_task_period_ms) % 200 == 0) {
                            left_speed_gwaggli *= -1; //invert direction of left speed
                            right_speed_gwaggli *= -1; //invert direction of left speed
                        }
                        motor_M1.setVelocity(motor_M1.getMaxVelocity() * left_speed_gwaggli);
                        motor_M2.setVelocity(motor_M2.getMaxVelocity() * right_speed_gwaggli);

                        //Lower the arm again while gwaggling, 200ms before gwaggling stops
                        if (counter > (truelli_time + turn_down_time + gwaggli_time - 200)/main_task_period_ms){
                            servo_Arm_D0.setPulseWidth(0.0f); //lower arm again
                        }
                    }

                    //lift the arm
                    if (counter > (truelli_time + turn_down_time + gwaggli_time)/main_task_period_ms && counter < (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) {
                        servo_Arm_D0.setPulseWidth(1.0f); //lift arm
                    } 

                    //things to do after the pick up or drop off action
                    if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) {
                        
                        //toggle storage of the package, so that it is now marked as delivered or picked up, depending on the previous state
                        package_storage[color_detected] = !package_storage[color_detected];    
                        
                        if(!package_storage[0] && !package_storage[1] && !package_storage[2] && !package_storage[3]){ //if all packages are marked as delivered
                            //The robot just follows the line for some time, before restarting entirely with the line follow state
                            if (counter < (truelli_time + turn_down_time + gwaggli_time + turn_up_time + repeat_time)/main_task_period_ms){
                                printf("DELIVERY COMPLETE, REPEAT\n");
                                motor_M1.setVelocity(lineFollower.getRightWheelVelocity());
                                motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());

                                //toggle storage of the package back, so that the next iteration is not confused
                                package_storage[color_detected] = !package_storage[color_detected];    

                            }else { //if the delivery was completed, restart entirely with the line follow state, to be able to pick up a package that was maybe missed in the previous round
                                counter = 0;
                                //switch to creep state
                                robot_state = RobotState::LINEFOLLOW;
                            }
                        }else{ //if not all packages are marked as delivered
                            counter = 0;
                            //switch to creep state
                            robot_state = RobotState::CREEP;
                        }

                    } 
                    counter++;
                    printf("counter: %d\n", counter);
                    break;
                }

                case RobotState::CREEP: { //after Package action, creep forward a little bit, before starting regular line following
                    printf("creep\n");
                    
                    static int counter = 0;
                    counter ++;
                    
                    if (counter < 300/main_task_period_ms) {
                        //just drive foward without line detection to avoid confusion due to cross-line
                        motor_M1.setVelocity(motor_M1.getMaxVelocity()*0.4);
                        motor_M2.setVelocity(motor_M2.getMaxVelocity()*0.4);
                    } else if (counter < 900/main_task_period_ms){
                        //set motor speed to linefollower calculations, but at higher speed than linefollow state
                        motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
                    }else if (counter >= 900/main_task_period_ms) {
                        //switch to line follow state after creeping
                        counter = 0;
                        robot_state = RobotState::LINEFOLLOW;
                        break;
                    }                        

                    break;
                }

                case RobotState::FINISHED: { //after finishing the round, do a victory dance with the servos and the motors                    
                    //IS NOT USED, BECAUSE THE ROBOT GOES BACK TO LINEFOLLOW STATE AFTER FINISHING THE PICK DROP ACTION

                    printf("VICTORY\n");    

                    static int counter = 0;
                    counter ++;
                    if (counter < 1500/main_task_period_ms){
                        //first, follow the line normally for some time
                        motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
                    }else if (counter >=1500/main_task_period_ms) {
                        //turn around
                        //counter = 0;
                        motor_M1.setVelocity(motor_M1.getMaxVelocity() * 0.1f);
                        motor_M2.setVelocity(motor_M2.getMaxVelocity() * -0.1f);
                    }
                                        
                    break;
                }

                case RobotState::EMERGENCY: { //In case of an emergency, stop the robot
                    printf("emergency\n");

                    //turn motors off
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);
                    break;
                }
                default: {
                    printf("default\n");
                    break; // do nothing
                }
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
                motor_M1.setMotionPlannerPosition(0.0f);
                motor_M1.setMotionPlannerVelocity(0.0f);
                motor_M1.enableMotionPlanner();
                motor_M2.setMotionPlannerPosition(0.0f);
                motor_M2.setMotionPlannerVelocity(0.0f);
                motor_M2.enableMotionPlanner();
                servo_Arm_D0.disable();
                servo_Truelli_D1.disable(); 
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