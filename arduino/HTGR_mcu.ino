#include <Arduino.h>
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

// Define stepper motor connections and steps per revolution:
#define M1_V_DIR 8
#define M1_V_STEP 9
#define M2_H_DIR 10
#define M2_H_STEP 11
#define DELAY 30
#define DELAY_V_FAST 20
#define DELAY_H_FAST 12
#define V_UNIT_Turn 23
#define H_UNIT_Turn 100
#define NUMBER_OF_MOTION 20

#define MAX_RAIL_H 79                  // 79
#define MAX_RAIL_V 51                  // 51
#define HOME_POSITION_V 0              // from the top
#define HOME_POSITION_H 14.25          // from the left

#define VERTICAL_FORCE_THR 20          // Set vertical force threshold

#define M1_MICRO_STEPS 32              // 1.8 degrees per step (32/6400)
#define M2_MICRO_STEPS 32              // 1.8 degrees per step (32/6400)
#define STEPS_ATTENUATION 0.25         // Attenuate ratio for H movement
#define STEPS_COMPENSTATION 1.4        // Compensation ratio for Vertical rod bending
#define SCALE_COEF_A 1                 // 1 step ~= 0.5 kg

// Variables to control force data sending frequency
unsigned long lastForceSendTime = 0;
const unsigned long forceSendInterval = 50; // Send force data every 100 ms

bool handleEmergencyCommands() {
  while (Serial.available()) {
    int code = Serial.parseInt();
    int case_num = Serial.parseInt();
    if (code == 3) {
      Serial.print("case_num: ");
      Serial.print(case_num);
      if (case_num == 1) {
        Serial.println(" Emergency Stop command received. Stop the operation.");
        clearSerialBuffer();
        return true;
      } else if (case_num == 2) {
        Serial.println(" Emergency Release command received. Release the force.");
        v_release(MAX_RAIL_V / 2, DELAY_V_FAST);
        clearSerialBuffer();
        return true;
      }
    }
  }
  return false;
}

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void motor_run(int step_pin, int delay_t) {
  digitalWrite(step_pin, HIGH);       // Set the 'step' pin on
  delayMicroseconds(delay_t);         // Delay in 'speed' microseconds
  digitalWrite(step_pin, LOW);        // Set the 'step' pin off
  delayMicroseconds(delay_t);         // Delay in 'speed' microseconds
}

bool move_motor_EM(int direction_pin, int step_pin, long steps, int delay_t) {
  digitalWrite(direction_pin, (steps > 0) ? LOW : HIGH);
  for (int i = 0; i < abs(steps); i++) {
    motor_run(step_pin, delay_t);
  }
  return false;
}

void v_release(long z_travel_gap, int delay_t) {
  // 100 means 1 mm movement of the stepper motor with 32/6400 setting with Linear actuator
  long steps = z_travel_gap * V_UNIT_Turn * M1_MICRO_STEPS;
  move_motor_EM(M1_V_DIR, M1_V_STEP, -steps, delay_t);
}

bool move_motor(int direction_pin, int step_pin, long steps, int delay_t) {
  digitalWrite(direction_pin, (steps > 0) ? LOW : HIGH);
  lastForceSendTime = millis(); // Reset the timer
  for (int i = 0; i < abs(steps); i++) {
    motor_run(step_pin, delay_t);

    // Check for emergency commands
    if (handleEmergencyCommands()) {
      return true;
    }

//    // Send force data periodically
//    unsigned long currentTime = millis();
//    if (currentTime - lastForceSendTime >= forceSendInterval) {
//      send_force_Data();
//      lastForceSendTime = currentTime;
//    }
  }
  return false;
}

void send_force_Data(){
  long current_scale = scale.get_units(1); // Read force in grams
  Serial.print("Force:");
  Serial.println(current_scale); // Example output: "Force:15000"
  delay(10);
}

//#3-2) Rubbing
bool rubbing(int n_iteration, int h_moving_gap) {
  // h_moving_gap is mm scale, initial point is 1 mm for one side
  int steps = h_moving_gap * H_UNIT_Turn / 2 * M2_MICRO_STEPS * STEPS_COMPENSTATION;
  int EM_flag = 0;

  for (int i = 0; i < n_iteration; i++) {
    EM_flag = move_motor(M2_H_DIR, M2_H_STEP, -steps, DELAY);
    if (EM_flag) {
      return true;
    }
    EM_flag = move_motor(M2_H_DIR, M2_H_STEP, 2 * steps, DELAY);
    if (EM_flag) {
      return true;
    }
    EM_flag = move_motor(M2_H_DIR, M2_H_STEP, -steps, DELAY);
    if (EM_flag) {
      return true;
    }
  }
  return false;
}

//#3-3) Stamping
bool stamping(int stamping_gap) {
  // stamping_gap is mm scale
  // 100 means 1 mm movement of the stepper motor with 32/6400 setting with Linear actuator
  int steps = stamping_gap * V_UNIT_Turn * M1_MICRO_STEPS;
  int EM_flag = 0;
  int slow_stamping_delay = DELAY * 8;
  EM_flag = move_motor(M1_V_DIR, M1_V_STEP, -steps, slow_stamping_delay);
  if (EM_flag) {
    return true;
  }

  // Wait for spores going down to bottom during 2 sec
  delay(2000);

  EM_flag = move_motor(M1_V_DIR, M1_V_STEP, steps, slow_stamping_delay);
  if (EM_flag) {
    return true;
  }
  delay(1000);

  return false;
}

//#4-1) Move Z direction
bool v_unit_moving(long z_travel_gap, bool z_moving_direction, int delay_t) {
  // z_travel_gap is mm scale
  // 100 means 1 mm movement of the stepper motor with 32/6400 setting with Linear actuator
  long steps = z_travel_gap * V_UNIT_Turn * M1_MICRO_STEPS;
  int EM_flag = 0;

  // Reset force send timer
  lastForceSendTime = millis();

  if (z_moving_direction == 1) {                         // going up
    EM_flag = move_motor(M1_V_DIR, M1_V_STEP, -steps, delay_t);
    if (EM_flag) {
      return true;
    }
    send_force_Data();

  } else if (z_moving_direction == 0) {                  // going down
    EM_flag = move_motor(M1_V_DIR, M1_V_STEP, steps, delay_t);
    if (EM_flag) {
      return true;
    }
    send_force_Data();
  }
  return false;
}

//#4-2) Move Y direction
bool h_unit_moving(long y_travel_gap, bool y_moving_direction, int delay_t) {
  // y_travel_gap is mm scale
  // 100 means 1 mm movement of the stepper motor with 32/6400 setting with Linear actuator
  long steps = y_travel_gap * H_UNIT_Turn * (M2_MICRO_STEPS * STEPS_ATTENUATION);
  int EM_flag = 0;

  // Reset force send timer
  lastForceSendTime = millis();

  if (y_moving_direction) {                            // going right
    EM_flag = move_motor(M2_H_DIR, M2_H_STEP, -steps, delay_t);
    if (EM_flag) {
      return true;
    }
    send_force_Data();

  } else {                                             // going left
    EM_flag = move_motor(M2_H_DIR, M2_H_STEP, steps, delay_t);
    if (EM_flag) {
      return true;
    }
    send_force_Data();
  }
  return false;
}

// Run Guilong's Motion
void run_G_motion(long v_force, int num_iter, int num_cycles, long program_motion_code, int stamping_gap, int travel_distance) {
  long current_scale = 0;
  long v_force_gram = 0;
  int EM_flag = 0;
  Serial.println(current_scale);

  long initial_steps = V_UNIT_Turn * M1_MICRO_STEPS * 30;
  EM_flag = move_motor(M1_V_DIR, M1_V_STEP, initial_steps, DELAY);

  v_force_gram = v_force * 1000;
  while (current_scale < v_force_gram) {
    if (current_scale < 1000) {
      long steps = V_UNIT_Turn * M1_MICRO_STEPS / 16;
      EM_flag = move_motor(M1_V_DIR, M1_V_STEP, steps, DELAY);
      if (EM_flag) {
        clearSerialBuffer();
        return;
      }
      current_scale = scale.get_units(1);
      send_force_Data();

      if (current_scale >= v_force_gram) break;
    } else {
      long steps = SCALE_COEF_A;
      EM_flag = move_motor(M1_V_DIR, M1_V_STEP, steps, DELAY);
      if (EM_flag) {
        clearSerialBuffer();
        return;
      }
      current_scale = scale.get_units(1);
      send_force_Data();

      if (current_scale >= v_force_gram) break;
    }
  }

  while (num_cycles >= 1) {
    current_scale = scale.get_units(3);
    send_force_Data();

    long motion_code = program_motion_code;
    int motion_array[NUMBER_OF_MOTION];
    int motion_count = 0;
    while (motion_code > 100) {
      motion_array[motion_count] = motion_code % 10;
      motion_code /= 10;
      motion_count++;
    }

    for (int i = motion_count - 1; i >= 0; i--) {
      if (motion_array[i] == 7) {
        current_scale = scale.get_units(3);
        send_force_Data();
        EM_flag = rubbing(num_iter, travel_distance);
        if (EM_flag) {
          clearSerialBuffer();
          return;
        }
      } else if (motion_array[i] == 8) {
        current_scale = scale.get_units(3);
        send_force_Data();
        EM_flag = stamping(stamping_gap);
        if (EM_flag) {
          clearSerialBuffer();
          return;
        }
      } else {
        continue;
      }
    }
    num_cycles -= 1;
  }
  EM_flag = v_unit_moving(MAX_RAIL_V, 1, DELAY_V_FAST);
  if (EM_flag) {
    clearSerialBuffer();
    return;
  }
  return;
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  setupScale();
  setupMotors();
}

void setupScale() {
  // Initialize the HX711 scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(18.11);  // Adjust this scale factor to your specific load cell
  scale.tare();  // Reset the scale to 0
}

void setupMotors() {
  pinMode(M1_V_DIR, OUTPUT);
  pinMode(M1_V_STEP, OUTPUT);
  pinMode(M2_H_DIR, OUTPUT);
  pinMode(M2_H_STEP, OUTPUT);
}

void loop() {
  long program_motion_code;
  int stamping_gap;
  int travel_distance;

  if (Serial.available()) {
    int code = Serial.parseInt();
    int case_num = Serial.parseInt();
    long param1 = Serial.parseInt();
    long param2 = Serial.parseInt();
    long param3 = Serial.parseInt();
    long param4 = Serial.parseInt();
    int param5 = Serial.parseInt();
    int param6 = Serial.parseInt();
    Serial.flush();

    long v_force = param1;
    long num_iter = param2;
    long num_cycles = param3;
    int EM_flag = 0;

    if (code == 0) {
      // Main functions
      switch (case_num) {
        case 0:
          // Set sample loading position(going up -> right)
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Set sample loading position(going up -> right)");
          Serial.flush();
          clearSerialBuffer();
          // Up
          EM_flag = v_unit_moving(MAX_RAIL_V, 1, DELAY_V_FAST);
          if (EM_flag) {
            return;
          }

          // Right
          EM_flag = h_unit_moving((MAX_RAIL_H - HOME_POSITION_H) / STEPS_ATTENUATION, 0, DELAY_H_FAST);
          if (EM_flag) {
            return;
          }
          break;

        case 1:
          // Set home position(going up -> left -> right -> down)
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Set home position(going up -> left -> right -> down)");
          Serial.flush();
          clearSerialBuffer();
          // Up
          EM_flag = v_unit_moving(MAX_RAIL_V, 1, DELAY_V_FAST);
          if (EM_flag) {
            return;
          }

          // Left
          EM_flag = h_unit_moving(MAX_RAIL_H / STEPS_ATTENUATION, 1, DELAY_H_FAST);
          if (EM_flag) {
            return;
          }

          // Right
          EM_flag = h_unit_moving(HOME_POSITION_H / STEPS_ATTENUATION, 0, DELAY_H_FAST);
          if (EM_flag) {
            return;
          }

          // Down
          EM_flag = v_unit_moving(HOME_POSITION_V, 0, DELAY_V_FAST);
          if (EM_flag) {
            return;
          }
          break;

        case 2:
          // Run Guilong's motion
          program_motion_code = param4;
          stamping_gap = param5;
          travel_distance = param6;

          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.print(" Program: ");
          Serial.println(program_motion_code);
          Serial.flush();
          clearSerialBuffer();
          run_G_motion(v_force, num_iter, num_cycles, program_motion_code, stamping_gap, travel_distance);
          clearSerialBuffer();
          break;

        default:
          Serial.println("Wrong command received in Code 0! Check the code");
          Serial.flush();
          clearSerialBuffer();
          break;
      }
    } else if (code == 1) {
      // Manual mode tab functions
      switch (case_num) {
        case 0:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Going max up");
          Serial.flush();
          clearSerialBuffer();
          v_unit_moving(MAX_RAIL_V, 1, DELAY_V_FAST);
          break;

        case 1:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Unit movement check, going up");
          Serial.flush();
          clearSerialBuffer();
          v_unit_moving(param1, 1, DELAY*8);
          break;

        case 2:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Unit movement check, going down");
          Serial.flush();
          clearSerialBuffer();
          v_unit_moving(param1, 0, DELAY*8);
          break;

        case 3:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Unit movement check, step left");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving(param1, 1, DELAY);
          break;

        case 4:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Unit movement check, step right");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving(param1, 0, DELAY);
          break;

        case 5:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Stamping Test");
          Serial.flush();
          clearSerialBuffer();
          stamping(param1);
          break;

        case 6:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Rubbing Test 10 iterations");
          Serial.flush();
          clearSerialBuffer();
          rubbing(10, param1);
          break;

        case 7:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Max Left");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving(MAX_RAIL_H / STEPS_ATTENUATION, 1, DELAY_H_FAST);
          break;

        case 8:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Max Right");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving(MAX_RAIL_H / STEPS_ATTENUATION, 0, DELAY_H_FAST);
          break;

        case 9:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Rubbing Test 60 iterations (1 min)");
          Serial.flush();
          clearSerialBuffer();
          rubbing(60, param1);
          break;

        case 10:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Rubbing Test 120 iterations (2 min)");
          Serial.flush();
          clearSerialBuffer();
          rubbing(120, param1);
          break;

        case 11:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Align from Left");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving(HOME_POSITION_H / STEPS_ATTENUATION, 0, DELAY_H_FAST);
          break;

        case 12:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Align from Right");
          Serial.flush();
          clearSerialBuffer();
          h_unit_moving((MAX_RAIL_H - HOME_POSITION_H) / STEPS_ATTENUATION, 1, DELAY_H_FAST);
          break;

        default:
          Serial.print("Case_num: ");
          Serial.print(case_num);
          Serial.println(" Wrong command received in Code 1! Check the code");
          Serial.flush();
          clearSerialBuffer();
          break;
      }
    } else {
      Serial.println("Wrong command received! Check the code");
      Serial.flush();
      clearSerialBuffer();
    }
  }
}