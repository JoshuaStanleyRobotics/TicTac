/*
   Description: This program is used to control TicTac: a two wheel balancing robot

        Wiring: The required components are 2x Nema17 stepper motors, 2x TMC2208 stepper drivers, a NRF24L01 radio module, a MPU6050 inertial measurment unit, and a Seeed XIAO microcontroller
        The step and direction pin for the driver for the right motor are connected to pins D0 and D1 respectively.
        The step and direction pin for the driver for the left motor are connected to pins D2 and D3 respectively.
        The NRF24L01 is connected 3.3V to 3V3, GND to GND, CSN to D46, MOSI to D51, CE to D48, SCK to D52, MISO to D50
        The MPU5060 is connected VCC to 3V3, GND to GND, SCL to D5, and SDA to D4
*/

#include "RF24.h"
RF24 radio(6, 7);
const byte chan[6] = "00007";
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double tilt = 0, turn = 0;

#include "Wire.h"
double pitch = 0;
unsigned long millisPrev = 0;
int dt = 0;
int cnt = 0;
double setpoint = 9.3, gain = 15, output;

int stepL = 2, dirL = 3, delayL = 0;
unsigned long microsL = 0;

int stepR = 0, dirR = 1, delayR = 0;
unsigned long microsR = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  Wire.begin();                                                                                     //Initialize IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU Initialized");

  for (int i = 0; i < 4; i++) {                                                                     //Configure stepper motors
    pinMode(i, OUTPUT);
  }
  Serial.println("Stepper Motors Set");
  delay(250);
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    if (data[1] == 0) tilt = 0;                                                                     //Forward and reverse controlled by front to back motion of left joystick
    else {
      tilt = data[1] * 10;
      tilt = tilt / 255 - 5;
    }

    if (data[4] == 0) turn = 0;                                                                     //Turning controlled by left to right motion of right joystick
    else {
      turn = data[4] * 40;
      turn = turn / 255 - 20;
    }
    if(tilt < 0) turn = -turn;

    gain = data[9] * 10;                                                                            //Proportional gain controlled by right potentiometer
    gain = gain / 255 + 10;

    setpoint = data[8] * 6;                                                                         //Pitch offset controlled by left potentiometer
    setpoint = setpoint / 255 + 6;
  }

  if (cnt > 125) {                                                                                  //Recalcualte pitch angle and wheel speed
    cnt = 0;
    Wire.beginTransmission(0x68);                                                                   //Request and receive data from IMU
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    int16_t input[7];
    for (int i = 0; i < 7; i++) {
      input[i] = Wire.read() << 8 | Wire.read();                                                    //Assign accelerometer and gyroscope data to input array
    }

    dt = millis() - millisPrev;
    millisPrev = millis();

    pitch += (double(input[6]) / 131) * (dt) / 1000;                                                //Calculate new angles based on previous angle, angular speed, and elapsed time
    pitch = 0.95 * pitch + 0.05 * ((double(atan2(input[0], input[1])) * 180 / PI));                 //Complimentary filter combining accelerometer and gyroscope data

    output = gain * (pitch - setpoint + tilt);                                                      //Calculate output based on pitch, setpoint, requested tilt, and proportional gain
    if(abs(output) > 200) output = 0;                                                               //Don't spin motors if the robot is tilted too far (fallen over)
    delayL = 25000 / abs(output - turn);                                                            //Calculate wheel speeds based on output and turning rate
    delayR = 25000 / abs(output + turn);
  }
  cnt++;

  if ((micros() - microsL) >= delayL) {                                                             //Send a pulse to the stepper drivers when it is time for the next step
    microsL = micros();
    digitalWrite(dirL, ((output - turn) > 0));
    digitalWrite(stepL, HIGH);
    delayMicroseconds(1);
    digitalWrite(stepL, LOW);
  }
  if ((micros() - microsR) >= delayR) {
    microsR = micros();
    digitalWrite(dirR, ((output + turn) > 0));
    digitalWrite(stepR, HIGH);
    delayMicroseconds(1);
    digitalWrite(stepR, LOW);
  }
}
