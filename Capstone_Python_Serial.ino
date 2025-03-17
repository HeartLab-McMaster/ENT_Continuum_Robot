//RED
#define LEFT_LED 33
//WHITE
#define MIDDLE_LED 32
//RED
#define RIGHT_LED 25
//GREEN
#define UP_LED 26
//GREEN
#define DOWN_LED 14

void setup() {
    Serial.begin(115200);  
    pinMode(LEFT_LED, OUTPUT);
    pinMode(MIDDLE_LED, OUTPUT);
    pinMode(RIGHT_LED, OUTPUT);
    pinMode(UP_LED, OUTPUT);
    pinMode(DOWN_LED, OUTPUT);

    //Turn off all LEDs
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(MIDDLE_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
    digitalWrite(UP_LED, LOW);
    digitalWrite(DOWN_LED, LOW);
}

void loop() {
    if (Serial.available()) {  // Check if Serial Data is seen
        String command = Serial.readStringUntil('\n');  // Read Serial Data
        command.trim();  // 

        //Turn OFF all LEDs
        digitalWrite(LEFT_LED, LOW);
        digitalWrite(MIDDLE_LED, LOW);
        digitalWrite(RIGHT_LED, LOW);
        digitalWrite(UP_LED, LOW);
        digitalWrite(DOWN_LED, LOW);

        //Check received command and turn on corresponding LED
        if (command == "TOP LEFT") {
            digitalWrite(LEFT_LED, HIGH);
            digitalWrite(UP_LED, HIGH);
            digitalWrite(MIDDLE_LED, HIGH);
        } else if (command == "TOP RIGHT") {
            digitalWrite(RIGHT_LED, HIGH);
            digitalWrite(UP_LED, HIGH);
            digitalWrite(MIDDLE_LED, HIGH);
        } else if (command == "LEFT") {
            digitalWrite(LEFT_LED, HIGH);
            digitalWrite(UP_LED, HIGH);
        } else if (command == "MIDDLE") {
            digitalWrite(MIDDLE_LED, HIGH);
            digitalWrite(UP_LED, HIGH);
        } else if (command == "RIGHT") {
            digitalWrite(RIGHT_LED, HIGH);
            digitalWrite(UP_LED, HIGH);
        } else if (command == "DOWN") {
            digitalWrite(DOWN_LED, HIGH);
        }

        //Debugging Output (Check this in Serial Monitor)**
        Serial.print("Received Command: ");
        Serial.println(command);
    }
}
