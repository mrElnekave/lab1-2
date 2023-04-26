#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0;     // To keep track of when we last commanded the motors
C610Bus<CAN2> teensy_bus;  // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5;  // Wait for 0.005s between motor updates. This is 200Hz.
// const int LOOP_DELAY_MILLIS = 250;  // Wait for 0.005s between motor updates. This is 4Hz.

const float m1_offset = 0.0;
const float m2_offset = 0.0;

// Step 5. Implement your own PD controller here.
float pd_control(float pos,
                 float vel,
                 float target,
                 float Kp,
                 float Kd) {
    // pos is 0-pi
    // target is 0-pi
    float tau = Kp * (target - pos) + Kd * (-vel);
    // Serial.printf("\ntau: %f\n", tau);
    return tau;
}

void sanitize_current_command(float &command,
                              float pos,
                              float vel,
                              float max_current = 2000,
                              float max_pos = 3.141,
                              float max_vel = 30,
                              float reduction_factor = 0.1) {
    /* Sanitize current command to make it safer.

    Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
    Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
    */
    command = command > max_current ? max_current : command;
    command = command < -max_current ? -max_current : command;
    if (pos > max_pos || pos < -max_pos) {
        Serial.println("ERROR: Actuator position outside of allowed bounds.");
    }
    if (vel > max_vel || vel < -max_vel) {
        command = 0;
        Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
    }
}

// This code waits for the user to type s before executing code.
void setup() {
    // flush teensy's buffer so we can add new stuff.
    while (Serial.available()) {
        Serial.read();
    }
    long last_print = millis();
    while (true) {
        char c = Serial.read();
        if (c == 's') {
            Serial.println("Starting code.");
            break;
        }
        if (millis() - last_print > 2000) {
            Serial.println("Press s to start.");
            last_print = millis();
        }
    }
}

void stop_if_serial() {
    if (Serial.available()) {
        if (Serial.read() == 's') {
            teensy_bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
            Serial.println("Stopping.");
            while (true) {
                continue;
            }
        }
    }
}

void sanitize_all(float &m0_current,
                  float &m1_current,
                  float &m2_current,
                  float m0_pos,
                  float m0_vel,
                  float m1_pos,
                  float m1_vel,
                  float m2_pos,
                  float m2_vel) {
    sanitize_current_command(m0_current, m0_pos, m0_vel);
    sanitize_current_command(m1_current, m1_pos, m1_vel);
    sanitize_current_command(m2_current, m2_pos, m2_vel);
}

void loop() {
    teensy_bus.PollCAN();  // Check for messages from the motors.

    long now = millis();

    stop_if_serial();

    /*
        if (now - last_command >= LOOP_DELAY_MILLIS) {
            float m0_pos = teensy_bus.Get(0).Position();  // Get the shaft position of motor 0 in radians.
            float m0_vel = teensy_bus.Get(0).Velocity();  // Get the shaft velocity of motor 0 in radians/sec.
            Serial.print("m0_pos: ");
            Serial.print(m0_pos);
            Serial.print("\tm0_vel: ");
            Serial.print(m0_vel);

            float m0_current = 0.0;
            float m1_current = 0.0;
            float m2_current = 0.0;

            // Step 8. Change the target position to something periodic
            float time = millis() / 1000.0;  // millis() returns the time in milliseconds since start of program

            // Step 5. Your PD controller is run here.
            float Kp = 1000;                        // steer strength
            float Kd = 100;                         // resistance to turning
            float target_position = 2 * sin(time);  // modify in step 8
            m0_current = pd_control(m0_pos, m0_vel, target_position, Kp, Kd);

            // Step 4. Uncomment for bang-bang control. Comment out again before Step 5.
            // if (m0_pos < 0) {
            //     m0_current = 800;
            // } else {
            //     m0_current = -800;
            // }

            // Step 10. Program periodic motion for all three motors.

            // Step 9. Program PID control for the two other motors.
            float m1_pos = teensy_bus.Get(1).Position();
            float m1_vel = teensy_bus.Get(1).Velocity();
            float m2_pos = teensy_bus.Get(2).Position();
            float m2_vel = teensy_bus.Get(2).Velocity();
            Serial.print("\tm1_pos: ");
            Serial.print(m1_pos);
            Serial.print("\tm1_vel: ");
            Serial.print(m1_vel);
            Serial.print("\tm2_pos: ");
            Serial.print(m2_pos);
            Serial.print("\tm2_vel: ");
            Serial.print(m2_vel);
            // m1_current = YOUR PID CODE
            // m2_current = YOUR PID CODE

            // Sanitizes your computed current commands to make the robot safer.
            sanitize_current_command(m0_current, m0_pos, m0_vel);
            sanitize_current_command(m1_current, m1_pos, m1_vel);
            sanitize_current_command(m2_current, m2_pos, m2_vel);
            // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
            // This actually gives the currents to the motors.

            teensy_bus.CommandTorques(m0_current, m1_current, m2_current, 0, C610Subbus::kIDZeroToThree);
            // Once you motors with ID=4 to 7, use this command
            // bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDFourToSeven);

            last_command = now;
            Serial.println();
        }

    */
    if (now - last_command >= LOOP_DELAY_MILLIS) {
        float bottom_motor = teensy_bus.Get(0).Position();
        float bottom_motor_vel = teensy_bus.Get(0).Velocity();
        float middle_motor = teensy_bus.Get(1).Position();
        float middle_motor_vel = teensy_bus.Get(1).Velocity();
        float top_motor = teensy_bus.Get(2).Position();
        float top_motor_vel = teensy_bus.Get(2).Velocity();

        // do the same for motor 2
        float bottom_motor2 = teensy_bus.Get(4).Position();
        float middle_motor2 = teensy_bus.Get(5).Position();
        float top_motor2 = teensy_bus.Get(3).Position();
        float top_motor2_vel = teensy_bus.Get(3).Velocity();

        float time = millis() / 1000.0;
        float target_position = (float)(3.0 / 2.0) * sin(time);
        Serial.printf("target_position: %f, time: %f\n", target_position, time);

        float Kp = 2000;
        float Kd = 200;

        float bottom_motor_current = pd_control(bottom_motor, bottom_motor_vel, bottom_motor2, Kp, Kd);
        float middle_motor_current = pd_control(middle_motor, middle_motor_vel, middle_motor2, Kp, Kd);
        float top_motor_current = pd_control(top_motor, top_motor_vel, top_motor2, Kp, Kd);
        float top_motor_current2 = 0;  // pd_control(top_motor2, top_motor2_vel, target_position, Kp, Kd);

        // sanitize
        sanitize_all(bottom_motor_current, middle_motor_current, top_motor_current, bottom_motor, bottom_motor_vel,
                     middle_motor, middle_motor_vel, top_motor, top_motor_vel);

        // now send the current to the motor
        teensy_bus.CommandTorques(bottom_motor_current, middle_motor_current, top_motor_current, top_motor_current2, C610Subbus::kIDZeroToThree);
        // teensy_bus.CommandTorques(500, 0, 0, 0, C610Subbus::kIDFourToSeven);

        // print all motor 1 positions
        Serial.printf("bottom motor 1 position: %f\n", bottom_motor);
        Serial.printf("middle motor 1 position: %f\n", middle_motor);
        Serial.printf("top motor 1 position: %f\n", top_motor);

        // timing
        last_command = now;
        Serial.println();
    }
}