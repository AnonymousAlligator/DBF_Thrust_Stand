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
    loop;
  }
}
// Main operational loop
void loop()                     
{
  if (digitalRead(butPort) == HIGH && debounce = 256) {
    butPress = 1;
  }
  delay(250);
}