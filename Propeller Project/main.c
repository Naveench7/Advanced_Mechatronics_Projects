#include "servo.h"
#include "utils.h" // Header files with constants, enums and utility functions

void initialize(); // Initializations

// Low Level Controller initializations
static volatile LL_CONTROL_MODE g_drive_mode; // Enum for drive mode
static volatile int forward_delta; // Correction delta
void llControl(void* data); // Function that does low level control
void goForward(); // Function to move forward
void goReverse(); // Function to move in reverse facing forward
void turnLeft(); // Function to turn left
void turnRight(); // Function to turn right
void stopMoving(); // Function to stop moving


// Line Sensor perception intitializations
static volatile int line_detections[NUM_LINE_SENS];
void sense(void* data);
static volatile bool intersection_detected, ignore_intersection; // Flags for intersection logic

// Ultrasonic Sensor perception
static volatile bool obstacle_detected; // Obstacle detection flag
static volatile bool target_detected; // Target detection flag
static volatile int num_targets_detected; // Counter for number of detected targets
static volatile bool turn_servo; // Flag to signify servo rotation
static volatile float distance_cm; // Distance mesasured by ultrasonic sensor
void senseRange(void * data); // Function handling ultrasonic sensor subsystem

// State machine initializations
static volatile STATES g_current_state; // Current state enum
static volatile unsigned char search_index; // Waypoint index
void stateMachine(void* data); // State machine function

// Localization inits
static volatile int intersection_count_horz = 0; // Current Y coordinate
static volatile int intersection_count_vert = 0; // Current X coordinate
static volatile int direction_horz = 1; // Current unit velocity vector Y component
static volatile int direction_vert = 0; // Current unit velocity vector X component
static volatile int current_yaw = 90; // Current yaw

// Buzzer 
void buzzerControl(void* data);

unsigned int stack1[100];  // for llcontrol
unsigned int stack2[150];  // for sense
unsigned int stack3[601]; // for senseRange
unsigned int stack4[1400];  // for state_machine
unsigned int stack5[90];  // for buzzer_control

// serial *term;


int main()                                    // Main function
{
  simpleterm_close();
  initialize();  // Initialize variables
  cogstart(&llControl, NULL, stack1, sizeof(stack1)); // Start low level control cog
  cogstart(&sense, NULL, stack2, sizeof(stack2)); // Start line sensing cog
  cogstart(&senseRange, NULL, stack3, sizeof(stack3)); // Start ultrasonic subsystem cog
  cogstart(&stateMachine, NULL, stack4, sizeof(stack4)); // Start state machine cog
  cogstart(&buzzerControl, NULL, stack5, sizeof(stack5)); // Start buzzer control cog
}

void initialize()
{
  g_drive_mode = FW; // Set initial drive mode to move forward
  ignore_intersection = true; // Ignore the 1st intersection
}

void llControl(void* data)
{
  
  //term = serial_open(31, 30, 0, 115200);
  while(1)
  {
    switch(g_drive_mode) // State machine for the low level control
    {
      case FW:
        goForward();
        break;
      case RW:
        goReverse();
        break;
      case U:
        current_yaw += 180;
        turnLeft();
        g_drive_mode = FW;
        break;
      case L:
        current_yaw += 90;
        turnLeft();
        g_drive_mode = FW;
        break;
      case R:
        current_yaw -= 90;
        turnRight();
        g_drive_mode = FW;
        break;
      case STOP:
        for(int i=0; i<75; i++)
        {
          goForward();
          pause(1);
        }
        stopMoving();
        while(1);
    }
    pause(10);    
  }  
}

// Drive forward for 1 timeframe
void goForward()
{ 
    forward_delta = (line_detections[1] * CORRECTION_DELTA) - (line_detections[2] * CORRECTION_DELTA); // Compute correction for each motor
    int left_servo_cmd = mapServo((SERVO_FORWARD_SPEED + LEFT_OFFSET) + forward_delta); // Compute signal width for left and right motors
    int right_servo_cmd = mapServo(-(SERVO_FORWARD_SPEED + RIGHT_OFFSET - forward_delta)); 
    
    //dprint(term, "Forward mode L: %d R:%d Delta:%d\n", left_servo_cmd, right_servo_cmd, forward_delta);
    servo_set(L_SERVO_PIN, left_servo_cmd);
    servo_set(R_SERVO_PIN, right_servo_cmd);
}

// Reverse is not used anywhere
void goReverse()
{
    forward_delta = (line_detections[1] * CORRECTION_DELTA) - (line_detections[2] * CORRECTION_DELTA); // Compute correction for each motor
    int left_servo_cmd = SERVO_FORWARD_SPEED + forward_delta;
    int right_servo_cmd = SERVO_FORWARD_SPEED - forward_delta;

    servo_speed(L_SERVO_PIN, -left_servo_cmd);
    servo_speed(R_SERVO_PIN, right_servo_cmd);
}

// Stop moving
void stopMoving()
{
    servo_speed(L_SERVO_PIN, mapServo(0));
    servo_speed(R_SERVO_PIN, mapServo(0));
}

// Turn left 90 degrees
void turnLeft()
{
    // Move forward to get the intersection under the bot
    for(int i=0; i<75; i++)
    {
      goForward();
      pause(1);
    }
    
    // Turn fast for a short amount of time
    int8_t left_servo_cmd =  SERVO_TURN_SPEED; 
    int8_t right_servo_cmd = SERVO_TURN_SPEED;
    servo_angle(L_SERVO_PIN, mapServoAngle(-left_servo_cmd));
    servo_angle(R_SERVO_PIN, mapServoAngle(-right_servo_cmd+RIGHT_OFFSET));
    pause(TURN_DELAY);
    
    // Slow down the turn speed until you have feedback from the line sensor
    left_servo_cmd =  SERVO_TURN_SPEED/2.0; 
    right_servo_cmd = SERVO_TURN_SPEED/2.0;
    servo_angle(L_SERVO_PIN, mapServoAngle(-left_servo_cmd));
    servo_angle(R_SERVO_PIN, mapServoAngle(-right_servo_cmd+RIGHT_OFFSET));

    while(!(line_detections[1] == 1 || line_detections[2] == 1))
    {
    }
    
    // Minor delay to allow the motors to overshoot the target which brings both sensors on the line
    pause(5);
}

// Turn right 90 degrees
void turnRight()
{
    // Move forward to get the intersection under the bot
    for(int i=0; i<75; i++)
    {
      goForward();
      pause(1);
    }
    
    // Turn fast for a short amount of time
    int8_t left_servo_cmd = SERVO_TURN_SPEED; 
    int8_t right_servo_cmd = SERVO_TURN_SPEED;
    servo_angle(L_SERVO_PIN, mapServoAngle(left_servo_cmd));
    servo_angle(R_SERVO_PIN, mapServoAngle(right_servo_cmd-RIGHT_OFFSET));
    pause(TURN_DELAY);
    
    // Slow down the turn speed
    left_servo_cmd = SERVO_TURN_SPEED/1.5; 
    right_servo_cmd = SERVO_TURN_SPEED/1.5;
    servo_angle(L_SERVO_PIN, mapServoAngle(left_servo_cmd));
    servo_angle(R_SERVO_PIN, mapServoAngle(right_servo_cmd-RIGHT_OFFSET));
    // Sense and turn until one of the sensors detects a line
    while(!(line_detections[1] == 1 || line_detections[2] == 1))
    {
    }

    // Minor delay to allow the motors to overshoot the target which brings both sensors on the line
    pause(5);
}

void sense(void* data)
{
  //term = serial_open(31, 30, 0, 115200);
  for(int i=0; i<NUM_LINE_SENS; i++)
  {
    set_direction(line_sensor_pins[i], 0); // Setting all line sensor pins as input
  }    
  int count = 0;
  while(1)
  {
    int sum = 0; // Sum for intersection detection
    count++; // Count for timeout detection
    for(int i=0; i<NUM_LINE_SENS; i++)
    {
      line_detections[i] = input(line_sensor_pins[i]); // Reading all input pins
      sum += line_detections[i]; // Updating the sum
    }
    if(sum == NUM_LINE_SENS && count > INTERSECTION_DETECT_TIMEOUT) 
    {
      count = 0; // Clear count for each intersection
      if(ignore_intersection)
      {
        ignore_intersection = false; 
        intersection_detected = false;
        //dprint(term, "Ignoring\n");
      }      
      else
      {
        intersection_detected = true;
        //dprint(term, "Intersection");
      }      
    }
    else
    {
      intersection_detected = false;
    }
    pause(1); // Pause used for timing
  }
}

void senseRange(void* data)
{
  //term = serial_open(31, 30, 0, 115200);
  while(1)
  {                
    // Turn the servo if the object is detected and turn flag is set
    if(turn_servo)
    {
      pause(1000);
      servo_angle(ULTRASONIC_SERVO_PIN, 1900);
    }
    else
    {
      servo_angle(ULTRASONIC_SERVO_PIN, 950);
    }
    
    // Pulse the ultrasonic sensor
    low(ULTRASONIC_TX_PIN);
    usleep(10);
    high(ULTRASONIC_TX_PIN);
    usleep(30);
    low(ULTRASONIC_TX_PIN);
    
    // Compute the distance using pulse in
    distance_cm = 0.017 * pulse_in(ULTRASONIC_RX_PIN, 1);
    
    // Check for obstacle only if servo is facing forward
    if(distance_cm < OBSTACLE_DETECTION_THRESHOLD && !turn_servo)
    {
      // If obstacle is detected update your current location 
      direction_vert = cos(current_yaw * PI / 180.0);
      direction_horz = sin(current_yaw * PI / 180.0);
      intersection_count_horz += direction_horz;
      intersection_count_vert += direction_vert;
      
      // Check for the location of the obstacle
      switch(intersection_count_horz)
      {
        case 5: // If its in i5 then update the seach route
          search_index = 2;
          search_vert[2] = 0;
          search_horz[2] = 4;
        case 3: // For the other 2 we use the same route
        case 2:
        default:
          break;
      }      
      g_drive_mode = U; // Take a U turn as soon as the obstacle is detected
      obstacle_detected = true;
      turn_servo = true;
    }
    
    // Check for the target only if we are not in the middle intersection
    if(distance_cm < TARGET_DETECTION_THRESHOLD && turn_servo && intersection_count_vert!=0)
    {
      num_targets_detected ++; // Update the target detection counter
      pause(2000); // Pause to not trigger it multiple times on the same target
    }      
    //dprint(term, "Distance: %f\n", distance_cm);   
    pause(40); // Pause used for printing 
  }    
  
}

void stateMachine(void* data)
{
  //term = serial_open(31, 30, 0, 115200);
  while (true)
  {  
    switch(g_current_state)
    {
      case STATES_INIT:
        //dprint(term, "Init state\n");
        g_current_state = STATES_LINE_FOLLOW; // Move to line follow state on init
        break;
      case STATES_LINE_FOLLOW: // Does nothing unless intersection is detected or all targets are found
        //  dprint(term, "Line Follow\n");
        if(intersection_detected)
        {
          g_current_state = STATES_INTERSECTION;
        }     
        if(num_targets_detected == TARGET_COUNT)
        {
          g_current_state = STATES_END;
          break;
        }          
        break;
      case STATES_INTERSECTION:
        
        //dprint(term, "Intersection state Yaw: %d\n", current_yaw);
        
        // Compute the current direction using rotation matrix
        direction_vert = cos(current_yaw * PI / 180.0);
        direction_horz = sin(current_yaw * PI / 180.0);
        
        // Update the current location using the direction
        intersection_count_horz += direction_horz;
        intersection_count_vert += direction_vert;
        //dprint(term, "Current Direction Vert: %d Horz: %d Obstacle:\n", direction_vert, direction_horz);
        //dprint(term, "Current Intersection Vert: %d Horz: %d\n", intersection_count_vert, intersection_count_horz);
        pause(10);
        //toggle(26);
        
        // After obstacle detection, we need to follow waypoints
        if(obstacle_detected)
        {
          
          // Find the next target waypoint
          int target_horz = search_horz[search_index];
          int target_vert = search_vert[search_index];
          
          // Compute the error in position
          int err_horz_world = target_horz - intersection_count_horz;
          int err_vert_world = target_vert - intersection_count_vert;
          
          // If error is 0 then update to the next waypoint
          if(err_horz_world == 0 && err_vert_world == 0)
          {
            //dprint(term, "Reached Target\n");
            pause(10);
            search_index += 1; // Update search index
            // Get new target
            target_horz = search_horz[search_index]; 
            target_vert = search_vert[search_index];

            // Compute error in map frame
            err_horz_world = target_horz - intersection_count_horz;
            err_vert_world = target_vert - intersection_count_vert;
            
            // Convert error to body frame using rotation matrix
            int err_x_body = err_vert_world * cos(current_yaw * PI / 180.0) + err_horz_world * sin(current_yaw * PI / 180.0);
            int err_y_body = -err_vert_world * sin(current_yaw * PI / 180.0) + err_horz_world * cos(current_yaw * PI / 180.0);
            
            //dprint(term, "Current Target Vert: %d Horz: %d\n", target_vert, target_horz);
            //dprint(term, "Current Err Body Vert: %d Horz: %d\n",err_x_body, err_y_body);

            // Check if we need to turn using body error
            if(err_y_body > 0)
            {
              //dprint(term, "Sending L\n");
              g_drive_mode = L;
            }
            if(err_y_body < 0)
            {
              //dprint(term, "Sending R\n");
              g_drive_mode = R;
            }          
          
          }                    
        } 
        g_current_state = STATES_LINE_FOLLOW;       
        break;
      case STATES_END:
        g_drive_mode = STOP; // Stop when it reaches the end       
        break;
        
      default: // Default should be line follow mode
        g_current_state = STATES_LINE_FOLLOW;
        break;
    }
    pause(1);
    
  }      
}

// Buzzer control
void buzzerControl(void* data)
{
  // Setting the actuator pins to output
  set_direction(BUZZER_PIN, 1);
  set_directions(27, 26, 0b11);
  
  // Initial blink during start
  pause(100);
  low(27);
  pause(100);
  low(26);
  pause(100);
  
  while(1)
  {
    // Play the buzzer if intersection is detected
    if(intersection_detected)
    {
      freqout(BUZZER_PIN, 500, 6000);
      pause(500);
      freqout(BUZZER_PIN, 500, 4000);
    }
    
    // Turn on 1 LED if 1 object is detected
    if(num_targets_detected == 1)
    {
      high(26);
    }
    
    // Turn on both LEDs and play buzzer if both targets are detected
    if(num_targets_detected == 2)
    {
      high(26);
      high(27);
      for(int i=0; i<10; i++)
      {
        freqout(BUZZER_PIN, 100, 2000);
        freqout(BUZZER_PIN, 500, 4000);
      }        
    }          
    usleep(200);
  }      
}      