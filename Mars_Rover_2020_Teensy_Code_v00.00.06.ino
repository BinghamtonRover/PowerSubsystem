
#define FILTER_SIZE 10
#define SLEEP_DELAY 0
#define SHUTDOWN_DELAY 0

#define debugger 1   //Change this value to 1 see debugging information in the arduino terminal or serial monitor
// LED Strip output
#define LEDStripRed 7
#define LEDStripGreen 6
#define LEDStripBlue 5

// Relay Outputs
#define relayArmMotorsNotOdrive 27
#define relayArmMotorsOdrive 28
#define relayWheelMotorOdrive1 29
#define relayWheelMotorOdrive2 30
#define relaySceinceMisc 31
#define relaySoftware 32
#define standbyButton 4

//Input Buttons
#define offButton 2
#define onButton 34
#define faultButton 3

//Input Button States
int standbyButtonState = 0;
int offButtonState = 0;
int onButtonState = 0;
int n = 0;

//Sensor Inputs
#define V_5VoltOut A13
#define V_12VoltOut A15
#define V_BatOut A17 //Voltage sensor connected to positive lead
#define I_5VoltOut A8
#define I_5VoltFault 23 //fault triggered when 12amps is exceeded on the Power Supply Board(U8)
#define T_5Volt A0
#define I_12Volt A2
#define I_12VoltFault 17 //fault triggered when 12.5 amps is exceeded on the Power Supply Baord(U9)
#define T_12Volt A6 //temperature sensor
#define I_Bat A14
#define Lem2 A16
#define Lem3 A18

//int current_status_serial = 0;
//uint16_t[FILTER_SIZE] battery_current;

enum STATUS { SLEEP, STANDBY, TURN_ON, _ON, FAULT, SHUTDOWN };
//enum STATUS rover_status();
bool check_faults();
//void shutdown_rover();
void go_to_sleep();
bool verify_fault();
//void read_sensors();
//void rover_status(); //added by Zev
//uint16_t[FILTER_SIZE] filter( uint16_t[FILTER_SIZE] );

void setup() {
  // pin set ups
  //configurations/initializations
  pinMode(LEDStripRed, OUTPUT);
  pinMode(LEDStripBlue, OUTPUT);
  pinMode(LEDStripGreen, OUTPUT);
  pinMode(relayArmMotorsNotOdrive, OUTPUT);
  pinMode(relayArmMotorsOdrive, OUTPUT);
  pinMode(relayWheelMotorOdrive1, OUTPUT);
  pinMode(relayWheelMotorOdrive2, OUTPUT);
  pinMode(relaySceinceMisc, OUTPUT);
  pinMode(relaySoftware, OUTPUT);
  pinMode(standbyButton, INPUT);
  pinMode(offButton, INPUT);
  pinMode(onButton, INPUT);
  pinMode(faultButton, INPUT);
  if (debugger == 1) {
    Serial.begin(9600);
  } // Specifically for debugging
}

void loop() {
  // The following is test to make sure the device works on boot. We can customize this later to change the color of the status bar
  if (n <= 0) {

    delay(100);
    digitalWrite(relaySoftware, HIGH);
    delay(100);
    digitalWrite(relaySoftware, LOW);
    delay(100);
    digitalWrite(relaySoftware, HIGH);
    delay(100);
    digitalWrite(relaySoftware, LOW);

    n++;
  }

   
  if (debugger == 1) {
    debug();
  }

  read_sensors();
  //send_data();
  rover_status();

  //delay(2000);  // for debug
}

void rover_status() {
  int index = 0;
  offButtonState = digitalRead(offButton);
  onButtonState = digitalRead(onButton);
  standbyButtonState = digitalRead(standbyButton);
  static enum { SLEEP, STANDBY, TURN_ON, _ON, FAULT } rover_status = SLEEP;
  switch ( rover_status ) {
    default:
    case STANDBY:
      if ( check_faults() ) {
        rover_status = FAULT;
      }
      else if ( offButtonState == HIGH/*off_button_pressed*/ ) {
        rover_status = SLEEP;
        shutdown_rover();
      }
      else if ( onButtonState == HIGH/*on_button_pressed*/ ) {
        rover_status = _ON;
        turnon_rover();
      }
      Serial.print("STANDBY");
      digitalWrite(LEDStripRed, LOW);
      digitalWrite(LEDStripBlue, LOW);
      digitalWrite(LEDStripGreen, LOW);
      digitalWrite(LEDStripGreen, HIGH);
      digitalWrite(LEDStripBlue, HIGH);
      
      break;
    //  case TURN_ON:
    //      if ( check_faults() ) {
    //        rover_status = FAULT;
    //        //break;
    //      }
    ////      else if ( /*another condition other than just no faults*/ ) {
    ////        rover_status = _ON;
    ////      }
    //      else if (standbyButtonState == HIGH/*standy_button_pressed*/ ) {
    //        shutdown_rover();
    //          rover_status = STANDBY;
    //        }
    //      turnon_rover();
    //      rover_status = _ON;
    //      Serial.print("TURN_ON");
    //      break;
    case _ON:

      if ( check_faults() ) {
        rover_status = FAULT;
        //break;
      }
      else if ( offButtonState == HIGH /*off_button_pressed*/ ) {
        //shutdown_rover();
        rover_status = SLEEP;
        go_to_sleep();
      }
      else if (standbyButtonState == HIGH /*standy_button_pressed*/ ) {
        rover_status = STANDBY;
        go_to_standby();

      }
      Serial.print("ON");
      digitalWrite(LEDStripRed, LOW);
      digitalWrite(LEDStripBlue, LOW);
      digitalWrite(LEDStripGreen, LOW);
      digitalWrite(LEDStripGreen, HIGH);
      break;
    case FAULT:
      /*
         As soon as a fault is tripped, enter fault mode, then if the problem can be solved by
         turning off a relay, do that and if the fault is no longer present, enter standby mode,
         then base station can turn the rover back on after a human turns it back on via base station (possibly),
      */

      if ( 0/*criticalfault*/  ) {
        // shutdown_rover();
        rover_status = SLEEP;
        go_to_sleep();
      }
      else {
        rover_status = STANDBY;
        go_to_standby();
      }
      Serial.print("FAULT");
      digitalWrite(LEDStripRed, LOW);
      digitalWrite(LEDStripBlue, LOW);
      digitalWrite(LEDStripGreen, LOW);
      digitalWrite(LEDStripRed, HIGH);
      
      while(index<5){
        digitalWrite(LEDStripRed, HIGH);
        delay(250);
        digitalWrite(LEDStripRed, LOW);
        delay(250);
        index++;
        }
      break;
    case SLEEP:
      if ( check_faults() ) {
        rover_status = FAULT;
      }
      else if ( standbyButtonState == HIGH/* standby button pressed */ ) {
        go_to_standby();
        rover_status = STANDBY;
      }

      //Not sure if anything has to even go here
      if (debugger == 1) {
        Serial.print("SLEEP");
      }
      digitalWrite(LEDStripRed, LOW);
      digitalWrite(LEDStripBlue, LOW);
      digitalWrite(LEDStripGreen, LOW);
      digitalWrite(LEDStripBlue, HIGH);
      break;
  }
  return;
}

bool check_faults() {
  // Check recent sensor readings for critical faults
  //  battery ovc,ovt,underV, power supply ovc,oct, maybe any ovc
  int fault_status = digitalRead(faultButton);
  return fault_status; //for testing
}

//go_to_sleep turns off all relay except the software teams relay
void go_to_standby() {
  digitalWrite(relayArmMotorsNotOdrive, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayArmMotorsOdrive, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive1, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive2, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relaySceinceMisc, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relaySoftware, HIGH);
}

void turnon_rover() {
  digitalWrite(relayArmMotorsNotOdrive, HIGH);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayArmMotorsOdrive, HIGH);
  if ( check_faults() ) {
    //rover_status = FAULT;
    go_to_sleep();
    return;
  }
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive1, HIGH);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive2, HIGH);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relaySceinceMisc, HIGH);
}

void shutdown_rover() {
  // Turn of relays that connect to drive, arm, science
  // Might need to relay some shutdown message to arm, science drive so motors dont turn off in weird positions etc.
  digitalWrite(relayArmMotorsNotOdrive, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayArmMotorsOdrive, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive1, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relayWheelMotorOdrive2, LOW);
  delay(SHUTDOWN_DELAY);
  digitalWrite(relaySceinceMisc, LOW);
  //delay(SHUTDOWN_DELAY);
  //digitalWrite(relaySoftware, LOW)
}

void go_to_sleep() {
  // we need to see if the software team wants the raspberry pis to be connected to power during sleep mode
  // put teensy to sleep

  digitalWrite(relayArmMotorsNotOdrive, LOW);
  delay(SLEEP_DELAY);
  digitalWrite(relayArmMotorsOdrive, LOW);
  delay(SLEEP_DELAY);
  digitalWrite(relayWheelMotorOdrive1, LOW);
  delay(SLEEP_DELAY);
  digitalWrite(relayWheelMotorOdrive2, LOW);
  delay(SLEEP_DELAY);
  digitalWrite(relaySceinceMisc, LOW);
  //delay(SLEEP_DELAY);
  //digitalWrite(relaySoftware, LOW);//Uncomment is software does not want their boards to go into a sleep mode
}

void read_sensors() {
  //  for( int i = FILTER_SIZE; i > 0; i-- ) battery_current[i] = battery_current[i-1];
  //  battery_current[0] = filter/*read_batter_current_sensor*/;
  //
}

void debug() {
  Serial.print("\n");
  Serial.print("FAULT BUTTON: ");
  Serial.print(digitalRead(faultButton));
  Serial.print("\n");
  Serial.print("Standby Button State: ");
  Serial.print(digitalRead(standbyButton));
  Serial.print("\n");
  Serial.print("On Button State: ");
  Serial.print(digitalRead(onButton));
  Serial.print("\n");
  Serial.print("Off Button State: ");
  Serial.print(digitalRead(offButton));
  Serial.print("\n");
}

//uint16_t[FILTER_SIZE] filter( uint16_t[FILTER_SIZE] );
