#include <HX711.h>
//#include <Wire.h>

// Button setup variables
#define BUT_PORT 0                          // Button port

// Load cell setup variables
HX711 leftLoadCell;
#define LEFT_LOAD_CELL_PORT 17              // Left load cell port
#define LEFT_LOAD_CELL_CLK  16              // Left load cell clock
HX711 rightLoadCell;
#define RIGHT_LOAD_CELL_PORT  18            // Right load cell port
#define RIGHT_LOAD_CELL_CLK  19             // Right load cell clock

#define LOAD_CELL_OFFSET  50682624          // Load cell offset
#define LOAD_CELL_DIVIDER  5895655          // Load cell divider

// Mauch setup variables
#define MAUCH_VOLTAGE_LINE  22              // Mauch voltage line port
#define MAUCH_CURRENT_LINE  23              // Mauch current line port
#define NO_BITS  10                         // Bits of precision
#define VOLTAGE_DIV  26.65421               // Voltage divider used by Mauch power module
#define AMP_PER_VOLT  30.06317              // Amps per volt of Mauch power module current line
#define SCALED_MAX  3.3                     // Maximum current and voltage are scaled to

// Torque sensor variables
HX711 torqueSensor;
#define TORQUE_PORT  25                     // Torque sensor port
#define TORQUE_CLK  24                      // Torque sensor clock
#define TORQUE_OFFSET  50682624             // Load cell offset
#define TORQUE_DIVIDER  5895655             // Load cell divider

// ESC control variables
#define PWM_MAX  2000                       // Max pulse width in ms
#define PWM_MIN  1000                       // Min pulse width in ms
#define DEFAULT_STEP 10                     // Default step value
int currPWM = 0;                            // Initial PWM value

// Test mode variables
// Manual
int stepAmount = 10;

// Ramp
int rampTarget = 2000;

// Hold
int holdVal = 0;

// Output values
int butPress = 0;                           // Button state variable
float leftLoad;                             // Left load sensor value
float rightLoad;                            // Right load sensor value
float torque;                               // Torque sensor value
float mauchVoltage;                         // Mauch measured voltage
float mauchCurrent;                         // Mauch measured current

// Timer
elapsedMillis sinceBegin = 0;

// Button setup
void buttonSetup() {
    pinMode(BUT_PORT, INPUT);
    attachInterrupt();
}

// Mauch port setup
void mauchPortSetup() {
    pinMode(MAUCH_VOLTAGE_LINE, INPUT);
    pinMode(MAUCH_CURRENT_LINE, INPUT);
}
// Mapping for analog read
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Obtaining voltage consumed by Mauch power module
float getMauchVoltages() {
    int mauchScaledVoltage = analogRead(MAUCH_VOLTAGE_LINE);
    return floatMap(mauchScaledVoltage, 0, 2^NO_BITS-1, 0, SCALED_MAX*VOLTAGE_DIV);
}

// Obtaining current consumed by Mauch power module
float getMauchCurrent() {
    int mauchScaledCurrent = analogRead(MAUCH_CURRENT_LINE);
    return floatMap(mauchScaledCurrent, 0, 2^NO_BITS-1, 0, SCALED_MAX*AMP_PER_VOLT);
}
// Load cell setup
void loadCellSetup() {
    // Left load cell
    leftLoadCell.begin(LEFT_LOAD_CELL_PORT, LEFT_LOAD_CELL_CLK);
    leftLoadCell.set_scale(LOAD_CELL_DIVIDER);
    leftLoadCell.set_offset(LOAD_CELL_OFFSET);

    // Right load cell
    rightLoadCell.begin(RIGHT_LOAD_CELL_PORT, RIGHT_LOAD_CELL_CLK);
    rightLoadCell.set_scale(LOAD_CELL_DIVIDER);
    rightLoadCell.set_offset(LOAD_CELL_OFFSET);
}

// Left load cell data
float getLeftLoadCell() {
    leftLoad = leftLoadCell.get_units(10);
}

// Right load cell data
float getRightLoadCell() {
    rightLoad = rightLoadCell.get_units(10);
}

// Torque sensor setup
void torquePortSetup() {
    torqueSensor.begin(TORQUE_PORT, TORQUE_CLK);
    torqueSensor.set_scale(TORQUE_DIVIDER);
    torqueSensor.set_offset(TORQUE_OFFSET);
}

// Torque sensor data
float getTorque() {
    torque = torqueSensor.get_units(10);
}

// ESC setup, negative stepVal to step down
elapsedMillis stepTime;
float escStepFunc(int targetPWM, int stepVal) {
    if (stepTime >= 100) {
        if (stepVal > 0) {
            if (currPWM + stepVal >= PWM_MAX) {
                currPWM = PWM_MAX;
            } else {
                currPWM = currPWM + stepVal;
            }
        } else {
            if (currPWM + stepVal < PWM_MIN) {
                curPWM = PWM_MIN;
            } else {
                currPWM = currPWM + stepVal;
            }
        }
        stepTime = 0;
    }
}

// Manual test mode increases the current PWM by the user defined amount
void manualTest() {
    Serial.print("Please enter desired step value: ");
    if(Serial.available() > 0) {
        int stepVal = Serial.read();
    } else {
        stepVal = DEFAULT_STEP;
    }
    int targetPWM = currPWM + stepVal;
    escStepFunc(targetPWM, stepVal);
}
// Ramp test mode continually adds the step value until the max PWM is reached
void rampTest() {
    Serial.print("Please enter the desired ramp value: ");
    if(Serial.available() > 0) {
        int stepVal = Serial.read();
    } else {
        stepVal = DEFAULT_STEP;
    }
    escStepFunc(PWM_MAX, stepVal);
}
// Hold test mode holds the current state and outputs the sensor values
void holdTest() {
    Serial.print("Displaying current sensor values \n"
                "PWM: "+currPWM+", "
                "Left load: "+leftLoad+", "
                "Right load: "+rightLoad+", "
                "Torque: "+torque+"\n");
    escStepFunc(currPWM, 0);
}
// Setup
void setup() {
    Serial.begin(38400);
    buttonSetup();
    mauchPortSetup();
    loadCellSetup();
    torquePortSetup();
}

// Printing information to serial
void infoPrint() {
    val = analogRead(0);
    Serial.print("analog 0 is: ");
    Serial.println(val);
}

// Checking if button is pressed
void buttonState() {
    if (digitalRead(BUT_PORT) == HIGH &&debounce = 256) {
        butPress = 1;
    }
    delay(250);
}

// Capture the button state and perform loop
void buttonStateCapture() {
  if (butPress == 1){
    butPress = 0;
  }
}

// System Modes
#define SYS_MODE_NONE           (0x0000)
#define SYS_MODE_SET_TEST       (0x0001)
#define SYS_MODE_RUN_TEST       (0x0002)
int sys_mode = SYS_MODE_NONE;
int prev_sys_mode = SYS_MODE_NONE;
int next_sys_mode = SYS_MODE_NONE;

#define TEST_UNDEFINED          (0x0000)
#define TEST_MANUAL             (0x0001)
#define TEST_RAMP               (0x0002)
#define TEST_HOLD               (0x0003)
int test_type = TEST_UNDEFINED;

// Main operational loop
void loop() {
    switch (sys_mode) {
        case SYS_MODE_NONE:
            // Reset to defaults
            Serial.print("---Setting defaults---\n");
            next_sys_mode = SYS_MODE_SET_TEST;
            test_done = false;
            test_type = TEST_UNDEFINED;

            break;
        case SYS_MODE_SET_TEST:
            if (prev_sys_mode != sys_mode) {
                // Initial setup
                Serial.print("---Test Setup---\n");
                Serial.print("Manual, Ramp, Hold (M, R, H):\t");
            }

            if (test_type == TEST_UNDEFINED && Serial.available() > 0) {
                switch (Serial.read()) {
                    case 'm':
                    case 'M':
                        test_type = TEST_MANUAL;
                        Serial.print("Selecting manual test\n");
                        break;
                    case 'r':
                    case 'R':
                        test_type = TEST_RAMP;
                        Serial.print("Selecting ramping test\n");
                        break;
                    case 'h':
                    case 'H':
                        test_type = TEST_HOLD;
                        Serial.print("Selecting hold throttle test\n");
                        break;
                    default:
                        test_type = TEST_UNDEFINED;
                        Serial.print("Deselecting test mode\n");
                        break;
                }
                Serial.clear();
            }

            if (test_type != TEST_UNDEFINED) {
                next_sys_mode = SYS_MODE_RUN_TEST;
            }

            break;
        case SYS_MODE_RUN_TEST:
            if (prev_sys_mode != sys_mode) {
                Serial.print("---Running Test---\n");
            }

            switch (test_type) {
                case TEST_MANUAL:
                    test_done = runManualTest();
                    break;

                case TEST_RAMP:
                    test_done = runRampTest();
                    break;

                case TEST_HOLD:
                    test_done = runHoldTest();
                    break;

                default:
                    test_done = true;
                    break;
            }

            if (test_done) {
                next_sys_mode = SYS_MODE_DONE;
            }

            break;
        case SYS_MODE_DONE:
            if (prev_sys_mode != sys_mode) {
                Serial.print("---Test Done---\n");
                Serial.print("Restart? (y): ");
            }

            if (Serial.available() > 0) {
                switch (Serial.read()) {
                    case 'y':
                    case 'Y':
                        next_sys_mode = SYS_MODE_NONE;
                        break;
                    default:
                        Serial.print("Restart? (y): ");
                        break;
                }
                Serial.clear();
            }
            break;
        default:
            next_sys_mode = SYS_MODE_NONE;
            break;
    }

    // On state change
    if (sys_mode != next_sys_mode) {
        prev_sys_mode = sys_mode;
        sys_mode = next_sys_mode;
    }
}

#define MANUAL_INIT         (0x0000)
#define MANUAL_USR_IN       (0x0001)
#define MANUAL_RUN          (0x0002)
int manualMode = MANUAL_INIT;

bool runManualTest() {

}

#define RAMP_INIT           (0x0000)
#define RAMP_RUN            (0x0001)
int rampMode = RAMP_INIT;

bool runRampTest() {

}

#define HOLD_INIT           (0x0000)
#define HOLD_RUN            (0x0001)
int holdMode = HOLD_INIT;
bool askedUsr = false;
bool gotHoldVal = false;

elapsedMillis testTime;
bool runHoldTest() {
    switch (holdMode) {
        case HOLD_INIT:
            if (!askedUsr) {
                Serial.print("Test PWM val (1000-2000): ");
                askedUsr = true;
            }
            if (Serial.available() > 0 && holdVal == 0) {
                int holdVal = int(Serial.readString().toInt());
            }
            break;
        case HOLD_RUN:
            break;

    }
