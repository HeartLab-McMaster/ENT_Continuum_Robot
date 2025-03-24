#include <TMCStepper.h>

// RED
#define LEFT_LED 33
// WHITE
#define MIDDLE_LED 32
// GREEN
#define RIGHT_LED 25
// GREEN
#define UP_LED 26
// GREEN
#define DOWN_LED 14
// Motor control pins for stepper motor
#define EN_PIN           9   // Enable pin (LOW = enabled, HIGH = disabled)
#define DIR_PIN          22  // Direction pin left and right
#define STEP_PIN         19  // Step pin, spins motor when high stops when low
#define SW_TX            18  // ESP32 UART TX to TMC2209 RX
#define SW_RX            5   // ESP32 UART RX to TMC2209 TX
#define DRIVER_ADDRESS   0b00  // Default driver address
#define R_SENSE          0.11f // Set according to your TMC2209 module

HardwareSerial mySerial(1);
TMC2209Stepper driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

String lastCommand = "OFF"; // Store the last received command
unsigned long lastStepTime = 0; // Track time for step pulses
const int stepDelay = 500; // Adjust timing for speed

void setup() {
    Serial.begin(115200);
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

    pinMode(LEFT_LED, OUTPUT);
    pinMode(MIDDLE_LED, OUTPUT);
    pinMode(RIGHT_LED, OUTPUT);
    pinMode(UP_LED, OUTPUT);
    pinMode(DOWN_LED, OUTPUT);

    // Turn off all LEDs
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(MIDDLE_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
    digitalWrite(UP_LED, LOW);
    digitalWrite(DOWN_LED, LOW);
    digitalWrite(STEP_PIN, LOW);

    Serial.println("Setup Done");
}

void loop() {
    if (Serial.available()) {  
        String command = Serial.readStringUntil('\n');
        command.trim();  

        Serial.print("Received Command: ");
        Serial.println(command);

        lastCommand = command; // Store the last command received

        // Turn OFF all LEDs
        digitalWrite(LEFT_LED, LOW);
        digitalWrite(MIDDLE_LED, LOW);
        digitalWrite(RIGHT_LED, LOW);
        digitalWrite(UP_LED, LOW);
        digitalWrite(DOWN_LED, LOW);
        digitalWrite(STEP_PIN, LOW);

        if (command == "TOP LEFT") {
            digitalWrite(LEFT_LED, HIGH);
            digitalWrite(MIDDLE_LED, HIGH);
        } else if (command == "TOP RIGHT") {
            digitalWrite(RIGHT_LED, HIGH);
            digitalWrite(MIDDLE_LED, HIGH);
        } else if (command == "LEFT") {
            digitalWrite(LEFT_LED, HIGH);
        } else if (command == "MIDDLE") {
            digitalWrite(MIDDLE_LED, HIGH);
        } else if (command == "RIGHT") {
            digitalWrite(RIGHT_LED, HIGH);
        } else if (command == "OFF") {
            digitalWrite(STEP_PIN, LOW); // Stop stepper
        }
    }

    // CONTINUOUS MOVEMENT LOGIC
    if (lastCommand == "FORWARD") {
        digitalWrite(UP_LED, HIGH);
        digitalWrite(DIR_PIN, LOW);
    } else if (lastCommand == "BACKWARD") {
        digitalWrite(DOWN_LED, HIGH);
        digitalWrite(DIR_PIN, HIGH);
    } else {
        return; // No movement, exit loop
    }

    // Generate step pulses at regular intervals
    if (micros() - lastStepTime >= stepDelay) {
        lastStepTime = micros();
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10); // Short pulse
        digitalWrite(STEP_PIN, LOW);
    }
}
