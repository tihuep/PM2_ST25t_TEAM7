#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "WS2812SPI.h"
#include "BasicMovement.h"
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

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    //DigitalOut led1(PB_9);
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
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); //Right Motor
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    //motor_M1.enableMotionPlanner();      //do not use with LineFollower
    // limit max. acceleration to half of the default acceleration
    //motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.1f);

    // motor M2
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); //Left Motor
    // limit max. velocity to half physical possible velocity
    //motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    //motor_M2.enableMotionPlanner();       //do not use with LineFollower
    // limit max. acceleration to half of the default acceleration
    //motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.1f);

    //BasicMovement basic_movement(motor_M1, motor_M2); // create BasicMovement object to easily command the robot to move forward, backward and turn
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// Servos
    Servo servo_Arm_D0(PB_D0);
    Servo servo_Truelli_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // servo Low: Insert servo name e.g. Futaba S3003
    float servo_Arm_D0_ang_min = 0.031f; // carefull, these values might differ from servo to servo
    float servo_Arm_D0_ang_max = 0.085f; //equvalent to 0.65f
    //Servo High: Insert servo name e.g. Futaba S3003
    float servo_Truelli_D1_ang_min = 0.031f;
    float servo_Truelli_D1_ang_max = 0.1175f;

    //To be calibrated
    servo_Arm_D0.calibratePulseMinMax(servo_Arm_D0_ang_min, servo_Arm_D0_ang_max);
    servo_Truelli_D1.calibratePulseMinMax(servo_Truelli_D1_ang_min, servo_Truelli_D1_ang_max);

    // default acceleration of the servo motion profile is 1.0e6f
    //enable if blocks fall off
    //servo_Arm_D0.setMaxAcceleration(1.0f);
    //servo_Truelli_D1.setMaxAcceleration(1.0f);
    //servo_Arm_D0.setMaxVelocity(0.9f); // limit velocity of the servo

    float left_speed_gwaggli = 0.07;
    float right_speed_gwaggli = -0.07;

    int truelli_state = 0;

//-----------------------------------------------------------------------------------------------------------------------------------------
// Line Array Sensor
    
    const float d_wheel = 0.05f; // wheel diameter in meters
    const float b_wheel = 0.158f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.08f; // distance from wheel axis to leds on sensor bar / array in meters
    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity()*0.4);

    //const float Kp = 1.0f * 2.0f;
    //const float Kp_nl = 1.0f * 17.0f;

    const float Kp = 1.0f * 2.0f;
    const float Kp_nl = 1.0f * 17.0f;

    lineFollower.setRotationalVelocityControllerGains(Kp, Kp_nl);

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// TCS3200 color sensor
    ColorSensor color_sensor(PA_8);   // creates instance of ColorSensor object with PwmIn at PB_3
    color_sensor.switchLed(ON);

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
        LEAVE_GARAGE,
        LINEFOLLOW,
        PICK_DROP,
        CREEP,
        FINISHED,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
// other variables
    bool package_height = 1; // 0 -> low, 1 -> high
    int color_detected = -1; // 0 -> red, 1 -> blue, 2 -> green, 3 -> yellow

    //array for storing, which packages are picked up
    bool package_storage[4] = {false,false,false,false}; //red, blue, green, yellow
    double velocity_factor = 1.0f; //velocity factor for line following, can be reduced if the robot is not able to follow the line at high speed

//-----------------------------------------------------------------------------------------------------------------------------------------

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---

            //Read out the values for each channel on black and white background
            //Put the values into ColorSensor.cpp m_reference_white and m_reference_black
            //printf("R: %.2f Hz\t G: %.2f Hz\t B: %.2f Hz\t C: %.2f Hz\n", color_sensor.readColor()[0], color_sensor.readColor()[1], color_sensor.readColor()[2], color_sensor.readColor()[3]);
            int color = color_sensor.getColor();
            printf("%s \n", color_sensor.getColorString(color));
            printf("Bit 0:%.2f \n", lineFollower.getAvgBit(0));
            
            // enable the servos
            if (!servo_Arm_D0.isEnabled())
                servo_Arm_D0.enable(0.0f); // enable with 0.0f pulse width, so that the arm is in the initial position, adjust this if necessary
            if (!servo_Truelli_D1.isEnabled())
                servo_Truelli_D1.enable(0.0f); // enable with 0.0f pulse width, so that the arm is in the initial position, adjust this if necessary

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {

                    printf("initial\n");
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    servo_Arm_D0.setPulseWidth(1.0f);
                    servo_Truelli_D1.setPulseWidth(0.0f); //high
                    truelli_state = 1; //high

                    robot_state = RobotState::LEAVE_GARAGE;
                    //robot_state = RobotState::LINEFOLLOW;

                    break;
                }
                case RobotState::LEAVE_GARAGE: {
                    
                    static int counter = 0;
                    printf("%d", counter);

                    //first 500 ms, drive straight forward, no matter what
                    if (counter < 500/main_task_period_ms){
                        motor_M1.setVelocity(0.5);
                        motor_M2.setVelocity(0.5);

                    //until 2000 ms, follow the line, unless more than 3 LEDs are on, if yes, turn slightly left
                    }else if (counter < 4000/main_task_period_ms){       
                        if (lineFollower.getMeanFourAvgBitsCenter() > 0.8){
                            motor_M1.setVelocity(0.5);
                            motor_M2.setVelocity(-0.2);
                        }else{
                            motor_M1.setVelocity(lineFollower.getRightWheelVelocity()*0.7); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(lineFollower.getLeftWheelVelocity()*0.7);  // set a desired speed for speed controlled dc motors M2
                        }

                    //if the 2000ms are over, go to normal operation
                    }else if (counter >= 4000/main_task_period_ms){
                        robot_state = RobotState::LINEFOLLOW;
                    }
                    counter++;
                    
                    break;
                }
                case RobotState::LINEFOLLOW: {
                    
                    printf("linefollow\n");
                    static int counter = 0;
                    static bool velocity_reduced = false;

/*
                    static int counter = 0;
                    static bool on = false;
                    rgbleds.setBrightness(127); // set brightness to maximum for the emergency signal

                    counter++;

                    if(counter > 500/main_task_period_ms) // ~500 ms (25 × 20 ms loop)
                    {
                        counter = 0;
                        on = !on;

                        if(on)
                        {
                            for(int i = 0; i < NUM_LEDS; i++)
                            {
                                rgbleds.setPixelColor(i, 0, 255, 0); // red color to indicate running
                            }
                        }
                        else
                        {
                            rgbleds.clear();
                        }

                        rgbleds.show();
                    }
                    break;
*/
/*
                    //if package_storage is empty and a color has been detected before, switch to FINISHED
                    if (!package_storage[0] && !package_storage[1] && !package_storage[2] && !package_storage[3] 
                            && color_detected < 0){
                        //robot_state = RobotState::FINISHED;
                        break;
                    }
*/
                    if(package_storage[0] && package_storage[1] && package_storage[2] && package_storage[3]){
                        velocity_reduced = true;
                        velocity_factor = 1.0f;
                        counter++;
                        if(counter > 8000/main_task_period_ms){
                            velocity_factor = 0.4f;
                        }
                    } else if(true){
                        velocity_factor = 0.4f;
                    }
                    //set motor speed to linefollower calculations
                    motor_M1.setVelocity(lineFollower.getRightWheelVelocity()*velocity_factor); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(lineFollower.getLeftWheelVelocity()*velocity_factor);  // set a desired speed for speed controlled dc motors M2


                    
                    //checks if the line is wider than normal on both sides
                    //and if color is not UNKNOWN, WHITE or BLACK
                    //to be sure, if we are actually at a cross line with a color
                    //printf("left: %f, right: %f\n", lineFollower.getMeanThreeAvgBitsLeft(), lineFollower.getMeanThreeAvgBitsRight());
                    if (lineFollower.getMeanFourAvgBitsCenter() > 0.8){
                        //turn off the motors
                        motor_M1.setVelocity(0);
                        motor_M2.setVelocity(0);
                        velocity_factor = 1.0f;
                        counter = 0;
                    
                        static int counter_color = 0;
                        
                        printf("break %d\n", counter_color);
                        
                        if (counter_color > 200/main_task_period_ms){
                            //set the positioning variables according to the color
                            switch (color) {
                                case 3: //RED
                                    package_height = 0; //low
                                    color_detected = 0;
                                    break;
                                case 8:
                                case 7: //BLUE
                                    package_height = 1; //high
                                    color_detected = 1;
                                    break;
                                case 5: //GREEN
                                    package_height = 0; //low
                                    color_detected = 2;
                                    break;
                                case 4: //YELLOW
                                    package_height = 1; //high
                                    color_detected = 3;
                                    break;
                                default:
                                    robot_state = RobotState::EMERGENCY;
                                    counter_color = 0;
                                    break;
                            }
                            //switch to servo actions
                            if (robot_state != RobotState::EMERGENCY){
                                robot_state = RobotState::PICK_DROP;
                                counter_color = 0;  
                            }
                        }
                        counter_color++;
                    }

                    break;
                }
                case RobotState::PICK_DROP: {
                    static int counter = 0;
                    
                    int truelli_time = 400;
                    int turn_down_time = 400;
                    int gwaggli_time = 500;
                    int turn_up_time = 200;

                    if (package_height == truelli_state){
                        truelli_time = 0;
                    }else {
                        if (package_height == 0) {
                            servo_Truelli_D1.setPulseWidth(1.0f); //low
                            if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) truelli_state = 0; // low
                        } else {
                            servo_Truelli_D1.setPulseWidth(0.0f); //high
                            if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) truelli_state = 1; //high
                        }
                    }

                    if(counter > (truelli_time)/main_task_period_ms && counter < (truelli_time + turn_down_time)/main_task_period_ms) { 
                        servo_Arm_D0.setPulseWidth(0.0f); //turn down
                    }

                    if (counter > (truelli_time + turn_down_time)/main_task_period_ms && counter < (truelli_time + turn_down_time + gwaggli_time)/main_task_period_ms) {
                        //gwaggli
                        if ((counter*main_task_period_ms) % 200 == 0) {
                            left_speed_gwaggli *= -1;
                            right_speed_gwaggli *= -1;
                        }
                        motor_M1.setVelocity(motor_M1.getMaxVelocity() * left_speed_gwaggli);
                        motor_M2.setVelocity(motor_M2.getMaxVelocity() * right_speed_gwaggli);
                    }

                    if (counter > (truelli_time + turn_down_time + gwaggli_time)/main_task_period_ms && counter < (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) {
                        servo_Arm_D0.setPulseWidth(1.0f); //turn up  

                        motor_M1.setVelocity(0);
                        motor_M2.setVelocity(0);
                    } 

                    if (counter > (truelli_time + turn_down_time + gwaggli_time + turn_up_time)/main_task_period_ms) {
                        counter = 0;

                        if (package_storage[color_detected]){
                            package_storage[color_detected] = !package_storage[color_detected]; //toggle storage of the package
                            if(!package_storage[0] && !package_storage[1] && !package_storage[2] && !package_storage[3]){
                                robot_state = RobotState::FINISHED;
                            } else{
                                robot_state = RobotState::CREEP;
                            }
                        } else{
                            package_storage[color_detected] = !package_storage[color_detected]; //toggle storage of the package
                            //if finished, switch to LINEFOLLOW
                            robot_state = RobotState::CREEP;
                        }
                    } 
                    counter++;
                    break;
                }
                case RobotState::CREEP: {
                    printf("CREEP\n");
                    //After Package action, creep forward a little bit, before starting regular line following
                    static int counter = 0;
                    counter ++;
                    if (counter < 300/main_task_period_ms) {
                        motor_M1.setVelocity(motor_M1.getMaxVelocity()*0.4); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(motor_M2.getMaxVelocity()*0.4);  // set a desired speed for speed controlled dc motors M2
                    } else if (counter < 1300/main_task_period_ms){
                        //set motor speed to linefollower calculations
                        motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
                    }else if (counter >= 1300/main_task_period_ms) {
                        counter = 0;
                        robot_state = RobotState::LINEFOLLOW;
                        break;
                    }                        

                    break;
                }
                case RobotState::FINISHED: {
                    printf("VICTORY\n");                    
                    static int counter = 0;
                    counter ++;
                    if (counter < 3000/main_task_period_ms){
                        //set motor speed to linefollower calculations
                        motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
                    }else if (counter >=3000/main_task_period_ms) {
                        //counter = 0;
                        //motor_M1.setVelocity(motor_M1.getMaxVelocity() * 0.5f);
                        //motor_M2.setVelocity(motor_M2.getMaxVelocity() * -0.5f);
                    }
                    static int hue = 0;
                    rgbleds.setBrightness(60); // set brightness to maximum for the victory dance

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

                    printf("emergency\n");

                    //turn motors off
                    motor_M1.setVelocity(0); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(0);  // set a desired speed for speed controlled dc motors M2
                    

                    static int counter = 0;
                    static bool on = false;
                    rgbleds.setBrightness(127); // set brightness to maximum for the emergency signal

                    counter++;

                    if(counter > 500/main_task_period_ms) // ~500 ms (25 × 20 ms loop)
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
                    printf("default\n");
                    break; // do nothing
                }
            }


            // visual feedback that the main task is executed, setting this once would actually be enough
            //led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                //led1 = 0;
                //basic_movement.stop();
                enable_motors = 0;
                motor_M1.setMotionPlannerPosition(0.0f);
                motor_M1.setMotionPlannerVelocity(0.0f);
                motor_M1.enableMotionPlanner();
                motor_M2.setMotionPlannerPosition(0.0f);
                motor_M2.setMotionPlannerVelocity(0.0f);
                motor_M2.enableMotionPlanner();
                servo_Arm_D0.disable();
                servo_Truelli_D1.disable(); 
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