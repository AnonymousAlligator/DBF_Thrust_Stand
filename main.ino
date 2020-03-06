#include <HX711.h>
//#include <Wire.h>

// Button setup variables
#define butPort 0                       // Button port

// Load cell setup variables
HX711 leftLoadCell;
#define leftLoadCellPort 17             // Left load cell port
#define leftLoadCellClk  16             // Left load cell clock
HX711 rightLoadCell;
#define rightLoadCellPort  18           // Right load cell port
#define rightLoadCellClk  19            // Right load cell clock
#define loadCellOffset  50682624        // Load cell offset
#define loadCellDivider  5895655        // Load cell divider

// Mauch setup variables
#define mauchVoltageLine  22            // Mauch voltage line port
#define mauchCurrentLine  23            // Mauch current line port
#define noBits  10                      // Bits of precision
#define voltageDiv  26.65421            // Voltage divider used by Mauch power module
#define ampPerVolt  30.06317            // Amps per volt of Mauch power module current line
#define scaledMax  3.3                  // Maximum current and voltage are scaled to

// Torque sensor variables
HX711 torqueSensor;
#define torquePort  25                  // Torque sensor port
#define torqueClk  24                   // Torque sensor clock
#define torqueOffset  50682624          // Load cell offset
#define torqueDivider  5895655          // Load cell divider

// ESC control variables
#define pwmMax  2000                    // Max pulse width in ms
#define pwmMin  1000                    // Min pulse width in ms
int currPWM = 0;                        // Initial PWM value
// Output values
int butPress = 0;                       // Button state variable
int loadLeft;                           // Left load sensor value
int loadRight;                          // Right load sensor value
int torque;                             // Torque sensor value

void buttonSetup() {
    pinMode(butPort, INPUT);
    attachInterrupt();
}

// Mauch port setup
void mauchPortSetup() {
    pinMode(mauchVoltageLine, INPUT);
    pinMode(mauchCurrentLine, INPUT);
}
// Mapping for analog read
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Obtaining voltage consumed by Mauch power module
float getMauchVoltages() {
    int mauchScaledVoltage = analogRead(mauchVoltageLine);
    return floatMap(mauchScaledVoltage, 0, 2^noBits-1, 0, scaledMax*voltageDiv);
}

// Obtaining current consumed by Mauch power module
float getMauchCurrent() {
    int mauchScaledCurrent = analogRead(mauchCurrentLine);
    return floatMap(mauchScaledCurrent, 0, 2^noBits-1, 0, scaledMax*ampPerVolt);
}
// Load cell setup
void loadCellSetup() {
    // Left load cell
    leftLoadCell.begin(loadLeftPort, loadLeftClk);
    leftLoadCell.set_scale(loadCellDivider);
    leftLoadCell.set_offset(loadCellOffset);

    // Right load cell
    rightLoadCell.begin(loadRightPort, loadRightClk);
    rightLoadCell.set_scale(loadCellDivider);
    rightLoadCell.set_offset(loadCellOffset);
}

// Left load cell data
float getLeftLoadCell() {
    return leftLoadCell.get_units(10);
}

// Right load cell data
float getRightLoadCell() {
    return rightLoadCell.get_units(10);
}

// Torque sensor setup
void torquePortSetup() {
    torqueSensor.begin(torquePort, torqueClk);
    torqueSensor.set_scale(torqueDivider);
    torqueSensor.set_offset(torqueOffset);
}

// Torque sensor data
float getTorque() {
    return torqueSensor.get_units(10);
}

// ESC setup, negative stepVal to step down
float escStepFunc(int targetPWM, int stepVal) {
    if (stepVal > 0) {
        while (currPWM < targetPWM) {
            if (currPWM + stepVal > pwmMax) {
                currPWM = pwmMax;
            } else {
                currPWM = currPWM + stepVal;
            }
        }
    } else {
        while (currPWM > targetPWM) {
            if (currPWM + stepVal < pwmMin) {
                curPWM = pwmMin;
            } else {
                currPWM = currPWM + stepVal;
            }
        }
    }
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
    if (digitalRead(butPort) == HIGH &&debounce = 256)
    {
        butPress = 1;
    }
    delay(250);
}

// Capture the button state and perform loop
void buttonStateCapture() {
    if (butPress == 1)
    {
        butPress = 0;
        loop;
    }
}
// Main operational loop
void loop() {
    if (digitalRead(butPort) == HIGH &&debounce = 256)
    {
        butPress = 1;
    }
    delay(250);
}