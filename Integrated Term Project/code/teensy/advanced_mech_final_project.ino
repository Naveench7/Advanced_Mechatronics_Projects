#include <NewPing.h>
#include <FreeRTOS_TEENSY4.h>
#include <Servo.h>

#define SERVO_FORWARD_SPEED 20  // Normalized speed for servo motors while moving forward
#define SERVO_TURN_SPEED 30     // Normalized speed for servo motors while turning
#define CORRECTION_DELTA 10     // Normalized delta for steering correction

#define ARM_PIN 4

#define L_SERVO_PIN 8   // PWM pin for the Left servo motor
#define R_SERVO_PIN 9  // PWM pin for the Right servo motor

#define FRIEND_LED_PIN 11
#define ENEMY_LED_PIN 13

#define LEFT_ACTUATION_SERVO_PIN 7
#define RIGHT_ACTUATION_SERVO_PIN 10

#define ULTRASONIC_TX_PIN 6     // Trigger pin for the Ultrasonic sensor
#define ULTRASONIC_RX_PIN 12    // Echo pin for the Ultrasonic sensor
#define ULTRASONIC_SERVO_PIN 10 // Pin for servo that controls the ultrasonic sensor
#define LED_PIN 13              // Pin for the LED and buzzer
#define OBJECT_MIN_DISTANCE 14 // The nearest an object should be
#define OBSTACLE_DETECT_TIMEOUT 30000

#define NUM_INTERSECTIONS 5   // Total number of intersections
#define INTERSECTION_DETECT_TIMEOUT 2000 // Timeout for intersection detection
// #define DEBUG

#define TURN_DELAY 700  // Tuned value, used to move forward to get the
                        // intersections under the rover
#define NUM_LINE_SENS 4 // Number of sensors
#define NUM_REAR_LINE_SENS 2

const uint8_t line_sensor_pins[NUM_LINE_SENS] = {17, 18, 19, 20}; // EXT interrupt for middle sensors
bool line_detections[NUM_LINE_SENS];    // Holds the senor readings


const uint8_t rear_line_sensor_pins[NUM_REAR_LINE_SENS] = {21, 22};
bool rear_line_detections[NUM_REAR_LINE_SENS];

enum LL_STATE     // Enum for states in the state machine
{
    FW,     // Move forward
    RW,     // Move in reverse
    LEFT,   // Turn Left
    RIGHT,  // Turn Right
    U,      // U Turn
    STOP    // Stop
};
LL_STATE current_ll_state;   // Variable for the state machine

enum STATES     // Enum for states in the state machine
{
    LINE,           // Line follow mode
    INTERSECTION,   // Detected intersection
    OBSTACLE,       // Detected obstacle
    END             // End state
};
STATES current_state;


int forward_delta;   // Computed normalized delta between motor speeds

uint8_t intersection_counter = 0; // Keeps the number of intersections in memory

bool detected_intersection = false;     // Bool that identifies if we detected an intersection
uint64_t previous_intersection_detection_time = millis();   // Time used to check the timeout of intersection
uint8_t ignore_intersection = 1;        // Ignores the next n intersections

Servo left_servo;       // Left servo object
Servo right_servo;      // Right servo 

Servo left_actuation_servo;
Servo right_actuation_servo;
bool servo_activated = false;

void forward();     // Function that moves the car forward
void reverse();     // Function that moves the car backward
void turnLeft();    // Function that turns the car left
void turnRight();   // Function that turns the car right
void turnAround();  // Function that takes a 180 degree turn
void stopMoving();        // Stops all motors

void sense();           // Reads from the line sensors and updates the line detection variable
void blinkLed(bool friendly);        // Sends pulses to LED and buzzer (Used when intersections are detected or if we finish)

inline uint8_t mapServo(int normalized_speed);   // Converts the normalized speed value of the motor to servo library
                                                    // Takes normalized speed from -100(Reverse) to 100(Forward) and converts
                                                    // it to a value between 0 - 180 for the servo library

float distance_cm = 5000;      // Distance sensed from the ultrasonic sensor
uint64_t prev_obstacle_detection_time;

// Defining paths 
LL_STATE path1[] = {LL_STATE::RW, LL_STATE::LEFT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::RW, LL_STATE::RW, LL_STATE::RW, LL_STATE::FW, LL_STATE::LEFT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW};
LL_STATE path2[] = {LL_STATE::RW, LL_STATE::RW, LL_STATE::LEFT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::RIGHT, LL_STATE::RW, LL_STATE::RW, LL_STATE::FW, LL_STATE::LEFT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW};
LL_STATE path3[] = {LL_STATE::RW, LL_STATE::RIGHT, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::RIGHT, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW, LL_STATE::FW};
LL_STATE *path;
uint8_t path_length;
uint8_t path_offset = 0;

// Localization and planning
int intersection_count = 0;
bool obstacle_detected = false;

// Declaring RTOS task functions
static void stateMachineTask        (void* arg);
static void stateLLControlTask      (void* arg);  
static void actuateIndicatorsTask   (void* arg);
static void actuateServoTask        (void* arg);
static void resetServoTask          (void* arg);
static void senseTask               (void* arg);
static void senseDistanceTask       (void* arg);
static void sendTX2Task             (void* arg);
static void recvTX2Task             (void* arg);


void setup()
{
    //Initialize the serial port
    Serial.begin(115200);
    Serial1.begin(115200);

    //Initialize arm pin
    pinMode(ARM_PIN, INPUT_PULLUP);
    pinMode(ENEMY_LED_PIN, OUTPUT);
    pinMode(FRIEND_LED_PIN, OUTPUT);

    delay(3000);
    
    
    // Setup servo motors
    left_servo.attach(L_SERVO_PIN);
    right_servo.attach(R_SERVO_PIN);

    left_actuation_servo.attach(LEFT_ACTUATION_SERVO_PIN);
    right_actuation_servo.attach(RIGHT_ACTUATION_SERVO_PIN);

    right_actuation_servo.write(30);
    left_actuation_servo.write(0);

    // Setup input pins for the IR sensor
    for(int i=0; i<NUM_LINE_SENS; i++)
    {
        pinMode(line_sensor_pins[i], INPUT);
    }

    // Setup pins for ultrasonic sensor
    pinMode(ULTRASONIC_TX_PIN, OUTPUT);
    pinMode(ULTRASONIC_RX_PIN, INPUT);

    while(!Serial1.available())
    {
        Serial.print('.');
        delay(1000);
    }
    while(Serial1.available())
    {
        Serial1.read();
    }
    Serial.println("Starting");
    //Starts all tasks
    portBASE_TYPE llControlTaskVar          = xTaskCreate(stateLLControlTask, NULL, 500, NULL, 2, NULL);
    portBASE_TYPE stateMachineTaskVar       = xTaskCreate(stateMachineTask, NULL, 500, NULL, 2, NULL);
    portBASE_TYPE senseTaskVar              = xTaskCreate(senseTask, NULL, 500, NULL, 2, NULL);
    portBASE_TYPE senseDistanceTaskVar      = xTaskCreate(senseDistanceTask, NULL, 1000, NULL, 2, NULL);
    //portBASE_TYPE sendTX2TaskVar            = xTaskCreate(sendTX2Task, NULL, 500, NULL, 2, NULL);
    portBASE_TYPE recvTX2TaskVar            = xTaskCreate(recvTX2Task, NULL, 1000, NULL, 2, NULL);
    //portBASE_TYPE actuateIndicatorsTaskVar  = xTaskCreate(actuateIndicatorsTask, NULL, 100, NULL, 2, NULL);
    portBASE_TYPE resetServoTaskVar         = xTaskCreate(resetServoTask, NULL, 500, NULL, 2, NULL);
    vTaskStartScheduler();
}

void loop()
{
    
}

static void stateMachineTask(void* arg)
{
    current_state = STATES::LINE;
    while(1)
    {
//       Serial.println("In state machine");
        switch(current_state)
        {
            case STATES::LINE:  // State machine line follower mode  
                if(detected_intersection)  // Checks if intersection is detection
                {
                      current_state = STATES::INTERSECTION; // Switch to intersection mode if its detected
                }
                if(distance_cm < OBJECT_MIN_DISTANCE && millis() - prev_obstacle_detection_time > OBSTACLE_DETECT_TIMEOUT)
                {
                    prev_obstacle_detection_time = millis();
                    current_state = STATES::OBSTACLE;
                }
                break;
            case STATES::INTERSECTION:  // State machine intersection mode
                Serial1.println("Intersection");
                Serial.print("INTERSECTION: ");
                current_state = STATES::LINE;
                intersection_count++;
                Serial.println(intersection_count);
                if(obstacle_detected)
                {
                    current_ll_state = path[path_offset++];
                    Serial.print("New command is ");
                    switch(current_ll_state)
                    {
                        case FW:
                            Serial.println("Forward");
                            break;
                        case RW:
                            Serial.println("Reverse");
                            break;
                        case LEFT:
                            Serial.println("Left");
                            break;
                        case RIGHT:
                            Serial.println("Right");
                            break;
                        case U:
                            Serial.println("U");
                            break;
                    }
                    if(path_offset == path_length)
                    {
                        vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
                        current_ll_state = LL_STATE::STOP;
                    }
                }
                break;
            case STATES::OBSTACLE:  // State machine intersection mode
                Serial1.println("Obstacle");
                Serial.println("Obstacle");
                obstacle_detected = true;
                switch(intersection_count)
                {
                    case 1:
                        path = path1;
                        path_length = sizeof(path1)/sizeof(LL_STATE);
                        Serial.println("Selecting path1");
                        break;
                    case 2:
                        path = path2;
                        path_length = sizeof(path2)/sizeof(LL_STATE);
                        Serial.println("Selecting path2");
                        break;
                    case 4:
                        path = path3;
                        path_length = sizeof(path3)/sizeof(LL_STATE);
                        Serial.println("Selecting path3");
                    default:
                        Serial.println("Somethings wrong ??!!");
                       
                }
                
                Serial.print("Size of path: ");
                Serial.println(path_length);
                current_state = STATES::INTERSECTION;
                break; 
        }
//        Serial.println("End state machine");
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L); // Delay for 30ms   
    }
    
}

static void stateLLControlTask(void* arg)
{
    current_ll_state = LL_STATE::FW;
    while(1)
    {
//        Serial.println("IN ll task");
        switch(current_ll_state)
        {
            case LL_STATE::FW:
                forward();
                break;
            case LL_STATE::RW:
                reverse();
                break;
            case LL_STATE::LEFT:
                turnLeft();
                current_ll_state = FW;
                break;
            case LL_STATE::RIGHT:
                turnRight();
                current_ll_state = FW;
                break;
            case LL_STATE::U:
                turnAround();
                current_ll_state = FW;
                break;
            case LL_STATE::STOP:
                stopMoving();
                break;
        }
//        Serial.println("End ll machine");
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L); // Delay for 10ms   
    }
}


static void actuateIndicatorsTask(void* arg)
{
    while(1)
    {
        vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L); // Delay for 1s  
    }
}

static void actuateServoTask(void* arg)
{
    while(1)
    {
        vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L); // Delay for 1s  
    }
}

static void senseTask(void* arg)
{
    while(true)
    {
        // Runs the sense function at 100Hz
        sense(); // Calls the sense function
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L); // Delay for 10ms
    }
}

static void senseDistanceTask(void* arg)
{
    int loop_freq = 20;
    float loop_period = 1000/(float)loop_freq;
    while(1)
    {
//        Serial.println("In Sense Distance");
        while(current_ll_state != LL_STATE::FW && current_ll_state != LL_STATE::RW)
        {
            vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
        }
        long current_time_sense_distance = millis();
        // Pulses the trigger pin of the ultrasonic sensor
        digitalWrite(ULTRASONIC_TX_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TX_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TX_PIN, LOW);

        uint64_t count = 0;
        while(!digitalRead(ULTRASONIC_RX_PIN));
        while(digitalRead(ULTRASONIC_RX_PIN) && count < 1000)
        {
            count ++;
            vTaskDelay((20L * configTICK_RATE_HZ) / 1000000L);
        }
        distance_cm = (count * 0.006) * 100;//sonar.ping_cm();//0.017 * pulseIn(ULTRASONIC_RX_PIN, HIGH, 10);
        //Serial.print("Distance measured: ");
        //Serial.print(distance_cm);
        //Serial.println("cm");
        
        // Delay for run loop at loop_freq
        long wait_time = loop_period - (millis() - current_time_sense_distance);
//        Serial.print("Waiting for: ");
//        Serial.println(wait_time);
//        Serial.println("End Sense Distance");
        if(wait_time > 0)
            vTaskDelay((wait_time * configTICK_RATE_HZ) / 1000L);   
        else
            vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);   
//        Serial.println("Delay end Distance");
    }
}

static void sendTX2Task(void* arg)
{
    while(1)
    {   
        // Waits for semaphore to be given
        //xSemaphoreTake(send_semaphore, portMAX_DELAY);
        //Serial.println("Found intersection");
        //Serial2.print("Intersection\n");
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L); // Delay for 10ms  
    }
}

static void recvTX2Task(void* arg)
{
    while(1)
    {
//        Serial.println("In recv");
        if(Serial1.available()>2)
        {          
            char detection_msg = Serial1.read();
            Serial.println("Got data");
            Serial.println(detection_msg);
            if(detection_msg == 'X')
            {
                char detection_type = Serial1.read();
                char detection_side = Serial1.read();
                while(current_ll_state != LL_STATE::FW && current_ll_state != LL_STATE::RW);
                LL_STATE prev_command = current_ll_state;
                current_ll_state = LL_STATE::STOP;
                Serial.print(detection_side);
                Serial.print(detection_type);
                vTaskDelay((10L * configTICK_RATE_HZ) / 1000L);
                if(detection_type == 'F')
                {
                    Serial.print("Found friend ");
                    digitalWrite(FRIEND_LED_PIN, HIGH);
                    if(detection_side == 'R')
                    {
                        Serial.println("on the Right side");         
                    }
                    else if(detection_side == 'L')
                    {
                        Serial.println("on the Left side");
                    }
                }
                else if(detection_type == 'E')
                {
                    Serial.print("Found enemy ");
                    digitalWrite(ENEMY_LED_PIN, HIGH);
                    if(detection_side == 'R')
                    {
                        Serial.println("on the Right side");
                        right_actuation_servo.write(100);
                    }
                    else if(detection_side == 'L')
                    {
                        Serial.println("on the Left side");
                        left_actuation_servo.write(70);
                    }
                    servo_activated = true;
                }
                vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L); // Delay for 1s
                digitalWrite(FRIEND_LED_PIN, LOW);
                digitalWrite(ENEMY_LED_PIN, LOW);
                current_ll_state = prev_command;
                
            }
        }
//        Serial.println("End recv");
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L); // Delay for 1s  
    }
}

static void resetServoTask(void* arg)
{
    while(1)
    {
        if(servo_activated)
        {
            vTaskDelay((4500L * configTICK_RATE_HZ) / 1000L); // Delay for 1s
            right_actuation_servo.write(30);
            left_actuation_servo.write(0);
            servo_activated = false;
        }
        vTaskDelay((50L * configTICK_RATE_HZ) / 1000L); // Delay for 1s
    }
    
}


void sense()
{
    uint8_t sum = 0;    // If the sum of all sensors == NUM sensors then we detect an intersection
//    Serial.print("Sense: ");
    for(int i=0; i<NUM_LINE_SENS; i++)  // Iterate over all sensor pins and read them
    {
        line_detections[i] = digitalRead(line_sensor_pins[i]);
//        Serial.print(line_detections[i]);
//        Serial.print(" ");
        sum += line_detections[i];  // Compute the sum 
    }

    for(int i=0; i<NUM_REAR_LINE_SENS; i++)  // Iterate over all sensor pins and read them
    {
        rear_line_detections[i] = digitalRead(rear_line_sensor_pins[i]);
    }
//    Serial.println();

    if(sum == NUM_LINE_SENS && millis() - previous_intersection_detection_time > INTERSECTION_DETECT_TIMEOUT)   // Timeout used to avoid multiple intersection detection
                                                                                                    // while standing on the same one
    {
        Serial.println(millis() - previous_intersection_detection_time);
        previous_intersection_detection_time = millis();
        if(ignore_intersection)             // If ignore intersection flag is detected we dont
        {                                   // update the detected intersection flag.
            ignore_intersection -= 1;       // We also clear the ignore intersection value
            Serial.println("Ignoring this intersection");
        }
        else
        {
            detected_intersection = true;
            Serial.println("Sense found this intersection");
        }
    }
    else
    {
        detected_intersection = false;      // If sum is not 5 then set the detected interserction flag to false
    }

}

/*
  * Computes a normalized delta using the sensor input 
  * If the left sensor detects the line and the right does not the output is -CORRECTION_DELTA
  * If the right sensor detects the line and the left does not the output is CORRECTION_DELTA
  * The correction delta is added to the left servo command and subtracted from the right servo command
  * We then remap the normalized command from domain (-100, 100) to (0, 180) and write to servo
*/
void forward()
{
    forward_delta = (line_detections[1] * CORRECTION_DELTA) - (line_detections[2] * CORRECTION_DELTA);
    uint8_t left_servo_cmd = SERVO_FORWARD_SPEED + forward_delta;
    uint8_t right_servo_cmd = SERVO_FORWARD_SPEED - forward_delta;
    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));
}

// Same as forward but the servo outputs are scaled by a factor of -1
void reverse()
{
    forward_delta = (rear_line_detections[1] * CORRECTION_DELTA) - (rear_line_detections[0] * CORRECTION_DELTA);
    uint8_t left_servo_cmd = SERVO_FORWARD_SPEED - (forward_delta+5); // Offset correction
    uint8_t right_servo_cmd = SERVO_FORWARD_SPEED + (forward_delta+5);
    left_servo.write(mapServo(-left_servo_cmd));
    right_servo.write(mapServo(right_servo_cmd));
}

/*
    * Turning is done using the same command for both motors, this makes the wheels rotate in the opposite directions
    * We turn fast for 700ms and then slow until one of the middle sensors detects the line
    * Once line is detected we stop and return
*/
void turnLeft()
{
    for(int i=0; i<700; i++)
    {
      forward();
      vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    }
    // Turn fast for 700ms
    int left_servo_cmd =  SERVO_TURN_SPEED-15; 
    int right_servo_cmd = SERVO_TURN_SPEED;
    left_servo.write(mapServo(-left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));
    Serial.println("Left: Turning Left");
    vTaskDelay((600L * configTICK_RATE_HZ) / 1000L);
    
    // Slow down the turn speed
    Serial.println("Left: Looking for middle sensors");
    left_servo_cmd =  SERVO_TURN_SPEED/1.5; 
    right_servo_cmd = SERVO_TURN_SPEED/1.5;
    left_servo.write(mapServo(-left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));

    // Sense and turn until one of the sensors detect a line
    while(!(line_detections[1] == 1 || line_detections[2] == 1))
    {
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L);
    }

    // Minor delay to allow the motors to overshoot which brings both sensors on the line
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    Serial.println("Left: Found line");
}

/*
    * Turning is done using the same command for both motors, this makes the wheels rotate in the opposite directions
    * We turn fast for 700ms and then slow until one of the middle sensors detects the line
    * Once line is detected we stop and return
*/
void turnRight()
{
    for(int i=0; i<700; i++)
    {
      forward();
      vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    }
    // Turn fast for 700ms
    int left_servo_cmd = SERVO_TURN_SPEED; 
    int right_servo_cmd = SERVO_TURN_SPEED;
    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(right_servo_cmd));
    Serial.println("Right: Turning Right fast");
    vTaskDelay((900L * configTICK_RATE_HZ) / 1000L);

    // Slow down the turn speed
    left_servo_cmd = SERVO_TURN_SPEED/1.5; 
    right_servo_cmd = SERVO_TURN_SPEED/1.5;
    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(right_servo_cmd));

    while(!(line_detections[1] == 1 || line_detections[2] == 1))
    {
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L);
    }

    // Minor delay to allow the motors to overshoot the target which brings both sensors on the line
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    Serial.println("Right: Found line");
}

/*
    * Turn around will command the bot to go back for 500ms and then turn left
    * Turn left will keep turning until the line is detected by a sensor
    * The line will be behind the bot so it will complete a 180 degree turn using the left command
*/
void turnAround()
{
    // Reverse command
    left_servo.write(mapServo(-30));
    right_servo.write(mapServo(30));
    // Minor delay
    delay(500);
    // Keeps turning left until a line is sensed
    turnLeft();
}

// Sends a neutral command to the servo
void stopMoving()
{
    left_servo.write(90);
    right_servo.write(90);
}

// Returns the remapped normalized speed from (-100, 100) to (0, 180)
inline uint8_t mapServo(int normalized_speed)
{
    if(!digitalRead(ARM_PIN))
        return map(normalized_speed, -100.0, 100.0, 0.0, 180.0);
    else
        return 90;
}
