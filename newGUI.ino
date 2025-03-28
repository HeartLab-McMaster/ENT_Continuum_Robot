#include <TMCStepper.h>

///////////////////////////// Coil control pins for coils

// PWM Pins
const int coilPinOne = 4;
const int coilPinTwo = 10;
const int coilPinThree = 22;
 
// Direction Pins
const int directionPinOne = 2;
const int directionPinTwo = 18;
const int directionPinThree = 21;
 
// PWM Channels
const int pwmChannelOne = 0;
const int pwmChannelTwo = 1;
const int pwmChannelThree = 2;
 
// PWM Settings
const int pwmFrequency = 10000; // 10kHz - 30kHz
const int pwmResolution = 5; // 0-255

///////////////////////////// Motor control pins for stepper motor

#define EN_PIN           13   // Enable pin (LOW = enabled, HIGH = disabled)
#define DIR_PIN          25  // Direction pin left and right
#define STEP_PIN         26  // Step pin, spins motor when high stops when low
#define SW_TX            33  // ESP32 UART TX to TMC2209 RX
#define SW_RX            27   // ESP32 UART RX to TMC2209 TX
#define DRIVER_ADDRESS   0b00  // Default driver address
#define R_SENSE          0.11f // Set according to your TMC2209 module

HardwareSerial mySerial(1);
TMC2209Stepper driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

unsigned long lastStepTime = 0; // Track time for step pulses
const int stepDelay = 500; // Adjust timing for speed
bool motorSpin = false;

void setup() {
  Serial.begin(115200);

    //Coil setup
    // Setup PWM channels
    ledcSetup(pwmChannelOne, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelTwo, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelThree, pwmFrequency, pwmResolution);
   
    // Attach PWM channels to pins
    //7.1
    ledcAttachPin(coilPinOne, pwmChannelOne);
    //7.2
    ledcAttachPin(coilPinTwo, pwmChannelTwo);
    //7.3
    ledcAttachPin(coilPinThree, pwmChannelThree);
   
    // Set direction pins as output
    pinMode(directionPinOne, OUTPUT);
    pinMode(directionPinTwo, OUTPUT);
    pinMode(directionPinThree, OUTPUT);
    
  // Stepper setup
    mySerial.begin(115200, SERIAL_8N1, SW_RX, SW_TX);
    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    digitalWrite(DIR_PIN, HIGH);
    driver.begin();
    driver.toff(5);
    driver.rms_current(600);
    driver.microsteps(16);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);

    Serial.println("Setup Done");
}

void loop() {
  
  // Generate step pulses at regular intervals
        if (motorSpin) {
            
            Serial.print("Moving motor");
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(500);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(500);
            }
        else {
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(500);
        }
        
  if (Serial.available()) {  
        String command = Serial.readStringUntil('\n');
        command.trim();  

        Serial.print("Received Command: ");
        Serial.println(command);


        // Coil control
        if (command == "X pos"){
          //away from bert coil 3
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
        }
        else if (command == "X neg"){
          //towards bert coil 3
          digitalWrite(directionPinThree, HIGH); // TOWARDS BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
        }
        else if (command == "Y pos"){
          //away from bert coil 1
          digitalWrite(directionPinOne, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelOne, 5); // Coil ON
        }
        else if (command == "Y neg"){
          //towards bert coil 1 **add hole in acrylic? 
          digitalWrite(directionPinOne, HIGH); // TOWARDS BERT
          ledcWrite(pwmChannelOne, 5); // Coil ON
        }
        else if (command == "Z pos"){
          // towards bert coil 2
          digitalWrite(directionPinTwo, HIGH); // TOWARDS BERT
          ledcWrite(pwmChannelTwo, 5); // Coil ON
        }
        else if (command == "Z neg"){
          //away from bert coil 2
          digitalWrite(directionPinTwo, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelTwo, 5); // Coil ON
        }
        else if (command == "XY pos"){
          //away from bert coil 3
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          digitalWrite(directionPinOne, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
          ledcWrite(pwmChannelOne, 5); // Coil ON
        }
        else if (command == "XY neg"){
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          digitalWrite(directionPinOne, HIGH); // TOWARDS BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
          ledcWrite(pwmChannelOne, 5); // Coil ON
        }
        else if (command == "XZ neg"){
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          digitalWrite(directionPinTwo, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
          ledcWrite(pwmChannelTwo, 5); // Coil ON          
        }
        else if (command == "XYZ Z neg"){
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          digitalWrite(directionPinOne, LOW); // TOWARDS BERT
          digitalWrite(directionPinTwo, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
          ledcWrite(pwmChannelOne, 5); // Coil ON
          ledcWrite(pwmChannelTwo, 5); // Coil ON
        }
        else if (command == "XYZ YZ neg"){
          digitalWrite(directionPinThree, LOW); // AWAY FROM BERT
          digitalWrite(directionPinOne, HIGH); // TOWARDS BERT
          digitalWrite(directionPinTwo, LOW); // AWAY FROM BERT
          ledcWrite(pwmChannelThree, 5); // Coil ON
          ledcWrite(pwmChannelOne, 5); // Coil ON
          ledcWrite(pwmChannelTwo, 5); // Coil ON
        }
        else if (command == "Coil OFF"){
          ledcWrite(pwmChannelThree, 100); // Coil OFF
          ledcWrite(pwmChannelOne, 100); // Coil OFF
          ledcWrite(pwmChannelTwo, 100); // Coil OFF
        }
        
        // Linear Actuator Control
        if (command == "EXT"){
          motorSpin = true;
          digitalWrite(DIR_PIN, LOW);
        }
        else if (command == "RTR"){
          //turn coils off
          ledcWrite(pwmChannelThree, 100); // Coil OFF
          ledcWrite(pwmChannelOne, 100); // Coil OFF
          ledcWrite(pwmChannelTwo, 100); // Coil OFF
          motorSpin = true;
          digitalWrite(DIR_PIN, HIGH);
        }
        else if (command == "OFF"){
          motorSpin = false;
        }

        // Emergency OFF
        if (command == "SHUT OFF"){
          //Coil and actuator is stopped
          ledcWrite(pwmChannelThree, 100); // Coil OFF
          ledcWrite(pwmChannelOne, 100); // Coil OFF
          ledcWrite(pwmChannelTwo, 100); // Coil OFF
          digitalWrite(directionPinThree, HIGH); // AWAY FROM BERT
          digitalWrite(directionPinOne, HIGH); // TOWARDS BERT
          digitalWrite(directionPinTwo, HIGH);
          motorSpin = false;
        }

        Serial.print("A");
            
  }
}
