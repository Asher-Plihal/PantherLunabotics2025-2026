#include <SoftwareSerial.h>
SoftwareSerial jetsonSerial(2, 3);  // RX=pin2, TX=pin3

"""
UNO Linear Actuator Control Code
This code is run on an arduino UNO to control a linear actuator based on commands 
received from a Jetson Nano. The actuator's position is read via an ADC and feedback 
is sent back to the Jetson for closed-loop control.
"""

#define FEEDBACK_PIN A0

#define IN1 8
#define IN2 9
#define ENA 10


int pwmSpeed = 200;
int tolerance = 8;

float targetInches = 0;
bool newCommand = false;

// ---- YOUR ORIGINAL CALIBRATION ----
int inchesToADC(float inches) {
  return map(inches * 100, 0, 400, 18, 1000);
}

float adcToInches(int adc) {
  return map(adc, 18, 1000, 0, 400) / 100.0;
}

void setup() {
  Serial.begin(9600);
  jetsonSerial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.println("READY");
}

void loop() {

  // -------------------------
  // Read serial command
  // -------------------------
  if (jetsonSerial.available()) {
    String cmd = jetsonSerial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("MOVE")) {
      targetInches = cmd.substring(5).toFloat();
      newCommand = true;
    }
  }

  // -------------------------
  // Control loop (NON-BLOCKING)
  // -------------------------
  if (newCommand) {

    int targetADC = inchesToADC(targetInches);
    int current = analogRead(FEEDBACK_PIN);
    int error = targetADC - current;

    // feedback to Jetson
    Serial.print("POS ");
    Serial.println(adcToInches(current), 2);

    if (abs(error) <= tolerance) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      Serial.println("DONE");
      newCommand = false;   // stop motion
    }
    else if (error > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
  }

  delay(10);
}