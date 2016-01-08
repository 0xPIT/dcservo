/*
   This program uses an Arduino for a closed-loop control of a DC-motor. 
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   
   Pins used:
   Digital inputs 2 & 3 are connected to the two encoder signals (AB).
   Digital outputs 9 & 10 control the PWM outputs for the motor (using half a L298 here).

   Please note PID gains kP, kI, kD need to be tuned to each different setup.
*/

#include <EEPROM.h>
#include <PID_v1.h>
#include <TimerOne.h>
#include "crc8.h"

struct {
  const uint8_t A = 2;
  const uint8_t B = 3;
} Encoder;

struct {
  const uint8_t A =  9;
  const uint8_t B = 10;
} Motor;

int8_t pos[1000]; 
int16_t p = 0;

typedef struct {
  double kP;
  double kI;
  double kD;
  uint8_t checksum;
} PID_t;

PID_t pidParam = { 3.00, 0.40, 0.30, 0 };

double input    = 0, 
       output   = 0,
       setpoint = 0;

PID pidCtrl(&input, &output, &setpoint, pidParam.kP, pidParam.kI, pidParam.kD, DIRECT);

volatile int32_t encoderPos = 0;
int32_t targetPos = 0;  // destination location at any moment
int8_t skip = 0;

bool randomizeMovements = false,
     printCurrentPosition = false,
     printEncoder = false,
     counting = false;

volatile int16_t delta;
volatile int16_t last;
int16_t lastDelta = -1;

void setup() { 
  pinMode(Encoder.A, INPUT); 
  pinMode(Encoder.B, INPUT);  

  Timer1.initialize(10); // µS
  Timer1.attachInterrupt(timerIsrTable);

  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31KHz PWM
  
  Serial.begin(115200);
  while (!Serial); // leonardo needs this

  help();

  if (loadParameters()) {
    pidCtrl.SetTunings(pidParam.kP, pidParam.kI, pidParam.kD); 
    Serial.println(F("*** PID parameters loaded from EEPROM:"));
    printPIDParams();
  }
  else {
    Serial.println(F("*** No PID values in EEPROM"));
  }

  pidCtrl.SetMode(AUTOMATIC);
  pidCtrl.SetSampleTime(1);
  pidCtrl.SetOutputLimits(-255, 255);
}

// Depending on encoder, choose a table or the notable version

void timerIsrNoTable(void) {
  int8_t curr = 0;

  if (PIND & 0x02) curr = 3;
  if (PIND & 0x01) curr ^= 1;
  
  int8_t diff = last - curr;

  if (diff & 1) {            // bit 0 = step
    last = curr;
    delta += (diff & 2) - 1; // bit 1 = direction (+/-)
  }
}

// decoding table for hardware with flaky notch (half resolution)
const int8_t encoderTableFlaky[16] = { 
  0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0 
};    

// decoding table for normal hardware
const int8_t encoderTableNormal[16] = { 
  0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 
};    

// from dcservo project
const int8_t encoderTableQEM[16] = {
  0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0
};

void timerIsrTable(void) {
  last = (last << 2) & 0x0F;

  if (PIND & 0x02) last |= 1;
  if (PIND & 0x01) last |= 2;

  int8_t tbl = encoderTableQEM[last]; 
  if (tbl) {
    delta += tbl;
  }
}

void loop() {
  input = encoderPos; 
  setpoint = targetPos;

  pidCtrl.Compute();
  
  if (Serial.available()) {
    processLine(); // it may induce a glitch to move motion, so use it sparingly 
  }
  
  pwmOut(output); 
  
  if (randomizeMovements && millis() % 3000 == 0) { 
    targetPos = random(2000); // that was for self test with no input from main controller
  }

  if (printCurrentPosition && millis() % 1000 == 0) {
    printPos();
  }
  
  if (counting && (skip++ % 5) == 0) {
    pos[p] = encoderPos; 
    if (p < 999) p++; 
    else counting = false;
  }

  if (delta != lastDelta) {
    if (printEncoder & delta != 0) {
      Serial.println(delta);
    }
    lastDelta = delta;
    encoderPos += delta;
    delta = 0;
  }
}

void pwmOut(int out) {
  if (out < 0) {
    analogWrite(Motor.A, 0); 
    analogWrite(Motor.B, abs(out));
  }
  else {
    analogWrite(Motor.A, abs(out));
    analogWrite(Motor.B, 0);
  }
}

void processLine() {
  char cmd = Serial.read();  
  if (cmd > 'Z') cmd -= 32;

  switch (cmd) {
    case 'H': help(); break;

    case 'P': pidParam.kP = Serial.parseFloat(); pidCtrl.SetTunings(pidParam.kP, pidParam.kI, pidParam.kD); break;
    case 'I': pidParam.kI = Serial.parseFloat(); pidCtrl.SetTunings(pidParam.kP, pidParam.kI, pidParam.kD); break;
    case 'D': pidParam.kD = Serial.parseFloat(); pidCtrl.SetTunings(pidParam.kP, pidParam.kI, pidParam.kD); break;

    case 'X': 
      targetPos = Serial.parseInt(); 
      p = 0;
      counting = true;
      for(int i = 0; i < 300; i++) {
        pos[i] = 0;
      }
      break;

    case 'T': randomizeMovements = !randomizeMovements; break;
    case 'A': printCurrentPosition = !printCurrentPosition; break;
    case 'E': printEncoder = !printEncoder; break;

    case 'W': saveParameters(); Serial.println(F("*** PID parameters stored to EEPROM.")); break;
    case 'R': loadParameters(); break;

    case '?': printPos(); break;
    case 'Q': printPIDParams(); break;
    case 'S':
      for (int i = 0; i < p; i++) {
        Serial.println(pos[i]);
      }
      break;
  }

  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F("Position=")); Serial.print(encoderPos);
  Serial.print(F(" PID_output=")); Serial.print(output);
  Serial.print(F(" Target=")); Serial.println(setpoint);
}

void printPIDParams() {
  Serial.print("P="); Serial.print(pidParam.kP);
  Serial.print(" I="); Serial.print(pidParam.kI);
  Serial.print(" D="); Serial.println(pidParam.kD);
}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("by misan, modified by @0xPIT"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("? prints out current encoder, output and setpoint values"));
  Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
  Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
  Serial.println(F("Q will print out the current values of P, I and D parameters")); 
  Serial.println(F("W will store current values of P, I and D parameters into EEPROM")); 
  Serial.println(F("H will print this help message again")); 
  Serial.println(F("A will toggle on/off showing regulator status every second")); 
  Serial.println(F("E will toggle on/off showing raw encoder values\n")); 
}

bool saveParameters() {
  uint16_t offset = 0;

  pidParam.checksum = crc8((uint8_t *)&pidParam, sizeof(PID_t) - sizeof(uint8_t));
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&pidParam, (void *)offset, sizeof(PID_t));

  return true;
}

bool loadParameters() {
  uint16_t offset = 0;

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&pidParam, (void *)offset, sizeof(PID_t));

  return pidParam.checksum == crc8((uint8_t *)&pidParam, sizeof(PID_t) - sizeof(uint8_t));
}
