#include <Servo.h>

#define SERVO_FORWARD_SPEED 20  // Normalized speed for servo motors while moving forward
#define SERVO_TURN_SPEED 30     // Normalized speed for servo motors while turning
#define CORRECTION_DELTA 10     // Normalized delta for steering correction

#define L_SERVO_PIN 9   // PWM pin for the Left servo motor
#define R_SERVO_PIN 11  // PWM pin for the Right servo motor

#define ULTRASONIC_TX_PIN 7     // Trigger pin for the Ultrasonic sensor
#define ULTRASONIC_RX_PIN 12    // Echo pin for the Ultrasonic sensor
#define ULTRASONIC_SERVO_PIN 10 // Pin for servo that controls the ultrasonic sensor
#define LED_PIN 13              // Pin for the LED and buzzer

#define NUM_INTERSECTIONS 5   // Total number of intersections
#define OBJECT_MIN_DISTANCE 5 // The nearest an object should be
#define INTERSECTION_DETECT_TIMEOUT 2000 // Timeout for intersection detection
// #define DEBUG

#define TURN_DELAY 700  // Tuned value, used to move forward to get the
                        // intersections under the rover
#define NUM_LINE_SENS 5 // Number of sensors

const uint8_t line_sensor_pins[NUM_LINE_SENS] = {5, 2, 4, 3, 6}; // EXT interrupt for middle sensors
bool line_detections[NUM_LINE_SENS];    // Holds the senor readings

enum STATES     // Enum for states in the state machine
{
    LINE,           // Line follow mode
    INTERSECTION,   // Detected intersection
    DETECT_OBJ,     // Looking for object
    OBJ_A,          // Found object in A
    OBJ_B,          // Found object in B
    OBJ_A_B,        // Found object in A and B
    END             // End state
};

STATES current_state;   // Variable for the state machine

int8_t forward_delta;   // Computed normalized delta between motor speeds

uint8_t intersection_counter = 0; // Keeps the number of intersections in memory

bool detected_intersection = false;     // Bool that identifies if we detected an intersection
uint16_t previous_intersection_detection_time = millis();   // Time used to check the timeout of intersection
uint8_t ignore_intersection = 1;        // Ignores the next n intersections

Servo left_servo;       // Left servo object
Servo right_servo;      // Right servo object
Servo ultrasonic_servo; // Ultrasonic sensor's servo object

bool object_a, object_b;    // Bool that gets set if objects are detected in side A and B
float distance_cm;      // Distance sensed from the ultrasonic sensor

inline uint8_t mapServo(int8_t normalized_speed);   // Converts the normalized speed value of the motor to servo library
                                                    // Takes normalized speed from -100(Reverse) to 100(Forward) and converts
                                                    // it to a value between 0 - 180 for the servo library

void forward();     // Function that moves the car forward
void reverse();     // Function that moves the car backward
void turnLeft();    // Function that turns the car left
void turnRight();   // Function that turns the car right
void turnAround();  // Function that takes a 180 degree turn
void stop();        // Stops all motors

// void leftSideCommands();
// void rightSideCommands();

void sense();           // Reads from the line sensors and updates the line detection variable
float senseDistance();  // Reads the distance to the nearest object using the ultrasonic sensor

void blinkLed();        // Sends pulses to LED and buzzer (Used when intersections are detected or if we finish)
void pulseUltrasonic(); // Used to send the pulse that starts the ultrasonic sensor reading

bool detectObject();    // Returns true if an object is present in front

void setup()
{
    // Setup servo motors
    left_servo.attach(L_SERVO_PIN);
    right_servo.attach(R_SERVO_PIN);

    // Setup input pins for the IR sensor
    for(int i=0; i<NUM_LINE_SENS; i++)
    {
        pinMode(line_sensor_pins[i], INPUT);
    }
    pinMode(4, INPUT_PULLUP);   // Decided not to use the middle sensor. Always set to high

    // ToDo: Add initialization of the ultrasonic sensor here
    ultrasonic_servo.attach(ULTRASONIC_SERVO_PIN);
    pinMode(ULTRASONIC_TX_PIN, OUTPUT);
    pinMode(ULTRASONIC_RX_PIN, INPUT);
    digitalWrite(ULTRASONIC_TX_PIN, LOW);

    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize the state machine to line follow mode
    current_state = STATES::LINE;

    //Serial port init
    Serial.begin(9600);

    // Used as a start flag. The robot wont start unitl A0 is connected to ground pin
    pinMode(A0, INPUT_PULLUP);
    while(digitalRead(A0))
    {
      delay(100);
    }
    delay(2000); // Delay for quality of life
}


void loop()
{
#ifdef DEBUG    
    // Used for testing different modules individually
    // This part wont be included by the compiler if we dont define DEBUG
    ultrasonic_servo.write(0);
    delay(1000);
    object_a = detectObject();
      
    ultrasonic_servo.write(180);
    delay(1000);
    object_b = detectObject();

    ultrasonic_servo.write(90);
    delay(1000);

    Serial.print("Debug: ");
    Serial.print(object_a);
    Serial.print(", ");
    Serial.println(object_b);
#endif    


    switch (current_state)
    {
    case STATES::LINE:  // State machine line follower mode  
        sense();        // Get sensor input
        if(intersection_counter == NUM_INTERSECTIONS)   // Switch to the end state if intersection counter
                                                        // equals the number of intersections defined above
        {
          current_state = STATES::END;                  
        }
        else if(detected_intersection)  // Checks if intersection is detection
        {
              current_state = STATES::INTERSECTION; // Switch to intersection mode if its detected
              intersection_counter++;               // Increment the intersection counter
        }
        forward();  // Move forward
        break;
    case STATES::INTERSECTION:  // State machine intersection mode
        Serial.println("State machine: Intersection");
        stop();             // Once intersection has been seen we need to stop
        blinkLed();         // Pulse the LED and buzzer
        forward();          // Move forward so that the intersection is under the bot and not under the sensor
        delay(TURN_DELAY);  // Delay and stop
        stop();             // Reliable using delays
        current_state = STATES::DETECT_OBJ; // Go to object detect mode
        break; 
    case STATES::DETECT_OBJ:
        Serial.println("State machine: Object detect mode");
        ultrasonic_servo.write(0);      // Look Right with the servo motor
        delay(1000);                    // Delay to make sure the senor reading is triggered once the servo completes the action
        object_a = detectObject();      // Check if object is present in A
        
        ultrasonic_servo.write(180);    // Look left with the servo motor
        delay(1000);                    // Same as above
        object_b = detectObject();      // Check for object B using ultrasonic sensor

        ultrasonic_servo.write(90);     // Bring the sensor back to the middle positon
        delay(1000);                    // Delay to make sure the sensor is back in positon before continuing

        if(object_a && object_b)                // If both objects are present
        {
            current_state = STATES::OBJ_A_B;    // Trasnition to state OBJ_A_B
            Serial.println("Transition: OBJ A B");
        }
        else if(object_a)                   // If object A only
        {
            current_state = STATES::OBJ_A;  // Transition to OBJ_A
            Serial.println("Transition: OBJ A");
        }
        else if(object_b)
        {
            current_state = STATES::OBJ_B;  // If object B only
            Serial.println("Transition: OBJ B");
        }
        else    // IF no object is present transition to line follow
        {
            Serial.println("Transition: Line Follow");
            current_state = STATES::LINE;
        }
        delay(1000);
        break;
    case STATES::OBJ_A_B:
        Serial.println("State machine: Object AB");
        objectABCommands();             // Function that holds the procedure if object is present in lanes A and B
        current_state = STATES::LINE;   // Transition back to line follow mode when object A and B mode is finished
        break;
    case STATES::OBJ_A:
        Serial.println("State machine: Object A");
        objectACommands();              // Function that holds the procedure if object is present in lane A
        current_state = STATES::LINE;   // Transition back to line follow mode when object A mode is finished            
        break;
    case STATES::OBJ_B:
        Serial.println("State machine: Object B");
        objectBCommands();               // Function that holds the procedure if object is present in lane B 
        current_state = STATES::LINE;    // Transition back to line follow mode when object B mode is finished    
        break;

    case STATES::END:               // End state
        stop();
        ultrasonic_servo.write(90);     // Bring the servo back to the middle position
        delay(1000);                    // Wait for servo to arrive
        bool object_c = detectObject(); // Detect if final object is present in the last spot
        if(object_c)
        {
            while(senseDistance() > OBJECT_MIN_DISTANCE)
            {
                sense();    // If object is present here then sense and move forward
                forward();  // until the bot is near it
            }
            // The demonstration is over
            stop();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            while(true)
                delay(10000);
        }
        else
        {
            for(int i=0; i<200; i++)
            {
              sense();      // If object is not present then
              forward();    // Go forward for 2 seconds and stop
              delay(10);
            }
            // The demonstration is over
            stop();
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            delay(1000);
            blinkLed();
            while(true)
                delay(10000);
        }
    default:
        break;
    }
}

void sense()
{
    uint8_t sum = 0;    // If the sum of all sensors == NUM sensors then we detect an intersection
//    Serial.print("Sense: ")
    for(int i=0; i<NUM_LINE_SENS; i++)  // Iterate over all sensor pins and read them
    {
        line_detections[i] = digitalRead(line_sensor_pins[i]);
//        Serial.print(line_detections[i]);
//        Serial.print(" ");
        sum += line_detections[i];  // Compute the sum 
    }
//    Serial.println();

    if(sum == NUM_LINE_SENS && millis() - previous_intersection_detection_time > INTERSECTION_DETECT_TIMEOUT)   // Timeout used to avoid multiple intersection detection
                                                                                                    // while standing on the same one
    {
        previous_intersection_detection_time = millis();
        if(ignore_intersection)             // If ignore intersection flag is detected we dont
        {                                   // update the detected intersection flag.
            ignore_intersection -= 1;       // We also clear the ignore intersection value
        }
        else
        {
            detected_intersection = true;
        }
    }
    else
    {
        detected_intersection = false;      // If sum is not 5 then set the detected interserction flag to false
    }

}

float senseDistance()
{
    pulseUltrasonic();  // Pulses the trigger pin of the ultrasonic sensor
    long duration =     // Computes the distance to the nearest target in cm using the pulsein command
    distance_cm = 0.017 * pulseIn(ULTRASONIC_RX_PIN, HIGH, 500000);
    return distance_cm; // Returns the distance in cm
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
    forward_delta = (line_detections[1] * CORRECTION_DELTA) - (line_detections[3] * CORRECTION_DELTA);
    int left_servo_cmd = SERVO_FORWARD_SPEED + forward_delta;
    int right_servo_cmd = SERVO_FORWARD_SPEED - forward_delta;

    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));
}

// Same as forward but the servo outputs are scaled by a factor of -1
void reverse()
{
    forward_delta = (line_detections[1] * CORRECTION_DELTA) - (line_detections[3] * CORRECTION_DELTA);
    uint8_t left_servo_cmd = SERVO_FORWARD_SPEED + forward_delta;
    uint8_t right_servo_cmd = SERVO_FORWARD_SPEED - forward_delta;

    left_servo.write(mapServo(-SERVO_FORWARD_SPEED));
    right_servo.write(mapServo(SERVO_FORWARD_SPEED));
}

/*
    * Turning is done using the same command for both motors, this makes the wheels rotate in the opposite directions
    * We turn fast for 700ms and then slow until one of the middle sensors detects the line
    * Once line is detected we stop and return
*/
void turnLeft()
{
    // Turn fast for 700ms
    int8_t left_servo_cmd =  SERVO_TURN_SPEED; 
    int8_t right_servo_cmd = SERVO_TURN_SPEED;
    left_servo.write(mapServo(-left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));
    Serial.println("Left: Turning Left");
    delay(700);
    
    // Slow down the turn speed
    Serial.println("Left: Looking for middle sensors");
    left_servo_cmd =  SERVO_TURN_SPEED/2.0; 
    right_servo_cmd = SERVO_TURN_SPEED/2.0;
    left_servo.write(mapServo(-left_servo_cmd));
    right_servo.write(mapServo(-right_servo_cmd));

    // Sense and turn until one of the sensors detect a line
    sense();
    while(!(line_detections[1] == 1 || line_detections[3] == 1))
    {
        sense();
        Serial.print(line_detections[1]);
        Serial.print("  ");
        Serial.println(line_detections[3]);
    }

    // Minor delay to allow the motors to overshoot which brings both sensors on the line
    delay(100);
    Serial.println("Left: Found line");
    stop();
}

/*
    * Turning is done using the same command for both motors, this makes the wheels rotate in the opposite directions
    * We turn fast for 700ms and then slow until one of the middle sensors detects the line
    * Once line is detected we stop and return
*/
void turnRight()
{
    // Turn fast for 700ms
    int8_t left_servo_cmd = SERVO_TURN_SPEED; 
    int8_t right_servo_cmd = SERVO_TURN_SPEED;
    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(right_servo_cmd));
    Serial.println("Right: Turning Right fast");
    delay(700);

    // Slow down the turn speed
    left_servo_cmd = SERVO_TURN_SPEED/2.0; 
    right_servo_cmd = SERVO_TURN_SPEED/2.0;
    left_servo.write(mapServo(left_servo_cmd));
    right_servo.write(mapServo(right_servo_cmd));

    // Sense and turn until one of the sensors detects a line
    sense();
    while(!(line_detections[1] == 1 && line_detections[2] == 1 && line_detections[3] == 1))
    {
        sense();
    }

    // Minor delay to allow the motors to overshoot the target which brings both sensors on the line
    delay(100);
    Serial.println("Right: Found line");
    stop();
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
    stop();
    // Keeps turning left until a line is sensed
    turnLeft();
}

// Sends a neutral command to the servo
void stop()
{
    left_servo.write(90);
    right_servo.write(90);
}

/*
    * Runs if an object is present on both A and B side
    * Turns left initially and then keeps going unitl the ultrasonic sensor detects the object is close
    * Once in front, stop and then reverse and turn 180 degrees, then keep going forward until the object
    * in side A is found. Then we reverse and then go forward until we detect an intersection. If found
    * we go forward a little more so that the bot is right under the intersection and then turns right
    * to reorient itself back in the straight line 
*/
void objectABCommands()
{
    // Turns left to get into side B 
    Serial.println("AB: Turn Left");
    turnLeft();

    // Go forward until the sensor returns a distance lower than the threshold 
    Serial.println("AB: Sense distance");
    while(senseDistance() > 8)
    {   
        sense();
        forward();
        delay(10);
    }

    // Stop and wait as asked in the problem
    stop();
    delay(3000);

    // Turn around
    Serial.println("AB: Turn around");
    turnAround();
    
    // Go forward until the next object is close
    while(senseDistance() > 8)
    {
        sense();
        forward();
        delay(10);
    }

    // Stop and wait again
    stop();
    delay(3000);

    // Turn around
    turnAround();

    // Keep going unitl an intersection is detected
    while(!detected_intersection)
    {
        sense();
        forward();
    }

    // Once detected we go forward for 700ms to get the intersection under the 
    for(int i=0; i<70; i++)
    {
      sense();
      forward();
      delay(10);
    }

    turnRight();

}

/*
    * Runs if an object is present on side A
    * Turns right initially and then keeps going unitl the ultrasonic sensor detects the object is close
    * Once in front, stop and then reverse and turn 180 degrees, then keep going forward until we detect 
    * an intersection. If found we go forward a little more so that the bot is right under the 
    * intersection and then turns right to reorient itself back in the straight line 
*/
void objectACommands()
{
    // Turns right to face side A
    turnRight();

    // Keep going until you reach close to the object
    while(senseDistance() > 8)
    {
        sense();
        forward();
        delay(10);
    }

    // Stop and wait
    stop();
    delay(3000);

    // Turn around
    turnAround();
    
    // Go forward until an intersection is found
    while(!detected_intersection)
    {
        sense();
        forward();
    }

    // Once detected we go forward for 700ms to get the intersection under the
    for(int i=0; i<70; i++)
    {
      sense();
      forward();
      delay(10);
    }

    // Turn right to face the next intersection
    turnRight();

}

/*
    * Runs if an object is present on side B
    * Turns left initially and then keeps going unitl the ultrasonic sensor detects the object is close
    * Once in front, stop and then reverse and turn 180 degrees, then keep going forward until we detect 
    * an intersection. If found we go forward a little more so that the bot is right under the 
    * intersection and then turns left to reorient itself back in the straight line 
*/
void objectBCommands()
{
    Serial.println("B: Turn Left");
    turnLeft();

    Serial.println("B: Sense distance");
    while(senseDistance() > 8)
    {
        
        sense();
        forward();
        delay(10);
    }

    stop();
    delay(3000);

    Serial.println("B: Turn around");
    turnAround();

    Serial.println("B: Looking for intersection");
    while(!detected_intersection)
    {
        sense();
        forward();
    }

    Serial.println("B: Found intersection");
    for(int i=0; i<70; i++)
    {
      sense();
      forward();
      delay(10);
    }

    Serial.println("B: Exit B");
    turnLeft();

}

// Returns a true if an object is present within 60cm
bool detectObject()
{
    float dist = senseDistance();
    Serial.println(dist);
    if(dist < 60)
    {
        return true;
    }
    return false;
}

// Returns the remapped normalized speed from (-100, 100) to (0, 180)
inline uint8_t mapServo(int8_t normalized_speed)
{
    return map(normalized_speed, -100.0, 100.0, 0.0, 180.0);
}

// Pulses the ultrasonic sensor
void pulseUltrasonic()
{
    digitalWrite(ULTRASONIC_TX_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TX_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TX_PIN, LOW);
}

// Pulses the LED and buzzer
void blinkLed()
 {
    for(int i=0; i<2; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}
