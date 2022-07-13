#include <stdbool.h>

#define PI 3.14159265359 // PI defenition for sin and cos

#define SERVO_FORWARD_SPEED 40  // Normalized speed for servo motors while moving forward
#define SERVO_TURN_SPEED 100     // Normalized speed for servo motors while turning
#define CORRECTION_DELTA 29     // Normalized delta for steering correction
#define RIGHT_OFFSET     10     // Offset for right servo (Used because 2 servos have different speeds on the same signal)
#define LEFT_OFFSET      -2     // Offset for left servo (Used because 2 servos have different speeds on the same signal)

#define L_SERVO_PIN 14   // PWM pin for the Left servo motor
#define R_SERVO_PIN 12  // PWM pin for the Right servo motor

#define ULTRASONIC_TX_PIN 0     // Trigger pin for the Ultrasonic sensor
#define ULTRASONIC_RX_PIN 1    // Echo pin for the Ultrasonic sensor
#define ULTRASONIC_SERVO_PIN 15 // Pin for servo that controls the ultrasonic sensor

#define NUM_INTERSECTIONS 5   // Total number of intersections
#define OBSTACLE_DETECTION_THRESHOLD 14 // The nearest an obstacle should be
#define TARGET_DETECTION_THRESHOLD 14 // The nearest the target should be should be
#define INTERSECTION_DETECT_TIMEOUT 1000 // Timeout for intersection detection

#define TARGET_COUNT 2 // Max target count

#define TURN_DELAY 400  // Tuned value, used to move forward to get the
                        // intersections under the rover
#define NUM_LINE_SENS 4 // Number of sensors

#define BUZZER_PIN 3 // Pin for the buzzer

enum LL_CONTROL_MODE
{
  FW, // Move forward
  RW, // Reverse
  L, // Turn left 90 degrees
  R, // Turn right 90 degrees
  U, // U turn
  STOP // Stop moving
};

enum STATES
{
  STATES_INIT, // Initial state
  STATES_LINE_FOLLOW, // Line follow state
  STATES_INTERSECTION, // Intersection state
  STATES_END // End state
};

// Line sensor pins
const uint8_t line_sensor_pins[NUM_LINE_SENS] = {5, 6, 7, 9}; 

// Pre configured waypoints
int search_vert[] = {0, -1, -1, 1, 1, -1, -1};
int search_horz[] = {1,  1,  4, 4, 1,  1,  5};

// Arduino map function using float
float mapF(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino map function using int
int mapI(int x, int in_min, int in_max, int out_min, int out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Servo pulse width mapping
int mapServo(int input)
{
  return mapI(input, -100, 100, 1000, 2000);
}

// Servo angle mapping
int mapServoAngle(int input)
{
  return mapI(input, -100, 100, 0, 1800);
}


