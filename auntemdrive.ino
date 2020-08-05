//Aunt Em Drive by Bob McGrath
//TOTAL PRODUCT RES TIME 198 SECONDS, 77RPM, 66 CYCLES, TWO 1.5 SECOND REST PERIODS

//Import Required libraries
#include "ClearCore.h"
#define motor ConnectorM0         // Define the motor's connector as ConnectorM0
#define TRIGGER_PULSE_TIME 25     // The TRIGGER_PULSE_TIME is set to 25ms to ensure it is within the Trigger Pulse Range defined in the MSP software (Default is 20-60ms)
#define INPUT_A_B_FILTER 20       // The INPUT_A_B_FILTER must match the Input A, B filter setting in MSP (Advanced >> Input A, B Filtering...)
#define RUN_SWITCH DI6            // Specify which input pin to read from. IO-0 through A-12 are all available as digital inputs.
#define JOG_SWITCH DI7
#define READY IO0                 // Configure pins IO-0 through IO-5 as digital outputs. These are the only pins that support digital output mode.
#define ERROR IO1
#define baudRate 9600             // Select the baud rate to match the target device.

const int SF=(163840/360);        //'SCALE FACTOR = REVS OF RESOLVER TO EQUALE ONE REV OF GEAR DRIVE OUTPUT SHAFT'
const int SPEED=(440*SF);         //'SCALE FACTORS PER SECOND = 73.3 RPM'
const int MAX_SPEED=(800*SF);     //'SCALE FACTORS PER SECOND = 73.3 RPM'
const int ACCEL=(480*SF);         //'SCALE FACTORS PER SECOND SQUARED = ? RPM’
const int DECEL=(480*SF);         //'SCALE FACTORS PER SECOND SQUARED = ? RPM'
const int FIRST_MOVE=(90*SF);
const int SECOND_MOVE=(-65*SF);
const int JOG_SPEED=(15*SF);
bool isError=false;

bool checkError()
{
  if (motor.IsInHwFault()
    || (motor.HlfbState() == MotorDriver::HLFB_UNKNOWN)
    )isError=true;
    else isError=false;

  if (isError)digitalWrite(ERROR,HIGH);
  if (!isError)digitalWrite(ERROR,LOW);
}

void setMode()
{
  motor.EnableRequest(false);
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);  // Sets the input clocking rate. This normal rate is ideal for ClearPath step and direction applications.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,   // Sets all motor connectors into step and direction mode.
                        Connector::CPM_MODE_STEP_AND_DIR);
  motor.VelMax(MAX_SPEED);  // Sets the maximum velocity in step pulses/sec.
  motor.AccelMax(ACCEL);   // Sets the maximum acceleration in step pulses/sec^2.

  checkError();
  motor.EnableRequest(true);// Enables the motor
  Serial.println("Motor Enabled");
}


void setSerial()
{
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }
}

void run()
{
  while(digitalRead(RUN_SWITCH)==HIGH){
    motor.Move(FIRST_MOVE);// Move the motor the specified number of pulses
    Serial.println("First Move Start");
    delay(10);// Delay so HLFB has time to deassert
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial.println("First Move Ends");
    delay(1500);

    motor.Move(SECOND_MOVE);// Move the motor the specified number of pulses
    Serial.println("Second Move Starts");
    delay(10);// wait until the command is finished and The motor's HLFB asserts
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial.println("Second Move Ends");
    delay(1500);
  }
}

void jog()
{
  motor.MoveVelocity(JOG_SPEED); // Move the motor forward at specified numver of pulses
  while(digitalRead(JOG_SWITCH)==HIGH){checkError();}
  motor.MoveStopAbrupt();
  delay(1000);
}

void setup()  // the setup routine runs once when you press reset:
{
  digitalWrite(ERROR,LOW);
  pinMode(READY, OUTPUT);
  pinMode(ERROR, OUTPUT);
  setMode();
  setSerial();
  checkError();
  digitalWrite(READY,HIGH);
}

void loop()// the main loop routine runs over and over again forever:
{
  checkError();
  if (digitalRead(RUN_SWITCH)==HIGH && digitalRead (JOG_SWITCH)==LOW)run();
  if (digitalRead(JOG_SWITCH)==HIGH && digitalRead (RUN_SWITCH)==LOW)jog();
}
