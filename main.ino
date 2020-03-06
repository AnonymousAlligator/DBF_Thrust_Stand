#include <SPI.h>

int butPort = 0;          //Button port
int loadLeftPort = 1;     //Left load sensor port
int loadRightPort = 2;    //Right load sensor port
int voltageDrawPort = 3;  //Volage drawn port
int torquePort = 4;       //Torque sensor port

int butPress = 0;         //Button state variable
int loadLeft;
int loadRight;
int torque;
// Assigning pin mode
void setup() {                
  Serial.begin(38400);
  pinMode(butPort, INPUT);
  pinMode(loadLeftPort, INPUT);
  pinMode(loadRightPort, INPUT);
  pinMode(voltageDrawPort, INPUT);
  pinMode(torquePort, INPUT);
}

// Printing information to serial
void infoPrint()                     
{
  val = analogRead(0);
  Serial.print("analog 0 is: ");
  Serial.println(val);
}

// Checking if button is pressed
void buttonState()                     
{
  if (digitalRead(butPort) == HIGH && debounce = 256) {
    butPress = 1;
  }
  delay(250);
}

// Capture the button state and perform loop
void buttonStateCapture()
{
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
            break;
        case SYS_MODE_SET_TEST:
            if (prev_sys_mode != sys_mode) {
                Serial.print("---Test Setup---\n");
                Serial.print("Manual, Ramp, Hold (M, R, H):\t");
            }

            if (test_type == TEST_UNDEFINED && Serial.available() > 0) {
                String usr_ans = Serial.read();
            }
            break;
        case SYS_MODE_RUN_TEST:
            if (prev_sys_mode != sys_mode) {
                Serial.print("---Running Test---\n");
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
    

//  if (digitalRead(butPort) == HIGH && debounce = 256) {
//    butPress = 1;
//  }
//  delay(250);
}
