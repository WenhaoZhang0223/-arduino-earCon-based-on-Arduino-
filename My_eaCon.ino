//LIBRARY
#include "FastLED.h"
#include <Servo.h>
#include "FastIMU.h"
#include <Wire.h>


//DEFINE
#define NUM_LEDS 14 
#define PIN_LED_RING 12 
#define UPDATE_TIME 250 
#define SWITCH_PIN 5  

#define SERVO_INPUT A0

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration

//LED
CRGB rgb_led_ring[NUM_LEDS]; 
unsigned index = 0; 
bool mode = false; 
//SERVOS A0
Servo servo1;  // create servo object to control a servo
Servo servo2;

//SERVOS IMU
MPU6500 IMU;            
calData calib = { 0 };  
AccelData accelData;   
GyroData gyroData;
MagData magData;
Servo servo3;  // create servo object to control a servo
Servo servo4;
int pos3;

void setup() {
  // LED SET UP
  pinMode(SWITCH_PIN, INPUT_PULLUP); 
  FastLED.addLeds<WS2813, PIN_LED_RING>(rgb_led_ring, NUM_LEDS); 

  //SERVOS A0 PART
  servo1.attach(6);
  servo2.attach(8);

  //SERVOS IMU
  servo3.attach(7);
  servo4.attach(9);
  //IMU set up
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(1000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(1000);
  }
  delay(1000);
  Serial.println("Keep IMU level.");
  delay(1000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);
#endif
}


void loop() {
   // LED PART
  mode = digitalRead(SWITCH_PIN) == LOW; // Read the switch state and update the mode

  // Debugging output for switch states
  Serial.print("Switch: ");
  Serial.println(mode ? "Yellow" : "Random Color");

  if (mode) {
    // Set all LEDs to yellow
    for (int i = 0; i < NUM_LEDS; i++) {
      rgb_led_ring[i] = CRGB::Yellow;
    }
    FastLED.show(); // Update the LED colors
    delay(10);
  } else {
    // Random color mode
    rgb_led_ring[index] = CHSV(random8(), 255, 255); // Set the color of the LED represented by index
    index++; // set the color of the next LED

    FastLED.show();
    if (index >= NUM_LEDS) { // If we walked over all LEDs, start from the beginning again
      index = 0;
    }
    delay(UPDATE_TIME);
  }

  // SERVOS A0 PART
  int servo1Angle = map(analogRead(SERVO_INPUT), 0, 1023, 0, 180);
  servo1.write(servo1Angle);
  int servo2Angle = map(analogRead(SERVO_INPUT), 0, 1023, 180, 0);
  servo2.write(servo2Angle);


  //SERVOS IMU PART
 //SERVOS  MOVE
  IMU.update();
  IMU.getGyro(&gyroData);
  Serial.print("The X moves: " );
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print("The Y moves: " );
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print("The Z moves: " );
  Serial.println(gyroData.gyroZ);
  // Adjust servo1 position based on gyro data
  if (gyroData.gyroX >= 30 || gyroData.gyroX <= -30) {
    pos3 += map(gyroData.gyroX, -1000, 1000, -10, 10); // Adjust position based on gyro X data
    pos3 = constrain(pos3, 0, 180); // limit angle
    servo3.write(pos3);
    delay(1);
  }
  
  if (gyroData.gyroX >= 30 || gyroData.gyroX <= -30) {
    int pos4 = 180 - pos3; // Calculate the position for servo2 as the opposite of servo1
    pos4 = constrain(pos4, 0, 180); // limit angle
    servo4.write(pos4);
    delay(1);
    }

}