#define LaserPin A6
#define PressureSensorPin A1
#define PressureSensorPin2 A4
#define PressureSetPin 3

float farDist = 350;
float nearDist = 50;

void setup() {
  pinMode(LaserPin, INPUT);
  pinMode(PressureSensorPin, INPUT);
  pinMode(PressureSensorPin2, INPUT);
  pinMode(PressureSetPin, OUTPUT);
  Serial.begin(115200);
}

float maxP = 30;
float freq = .1;
float prevUpdate = 0;

void loop() {
  float time = (float(micros()) / 1000000.0);

  if (time - prevUpdate >= (1 / freq) * 10) {
    freq = freq + .1;
    prevUpdate = time;
  }

  int val = analogRead(LaserPin);
  float distance = (val / 1023.0) * (farDist - nearDist);


  float P = maxP / 2.0 + (maxP / 2.0) * sin(2.0 * 3.14159 * time * freq);
  int setPressure = int(constrain(P, 0, maxP));

  int SetVoltage = map(setPressure, 0, 145, 0, 255);
  analogWrite(PressureSetPin, SetVoltage);

  int PressureSensorVal = analogRead(PressureSensorPin);
  float actualPressure = float(PressureSensorVal) * (145. / 809.) - 38.3;

  PressureSensorVal = analogRead(PressureSensorPin2);
  float actualPressure2 = 0.615 * float(PressureSensorVal) - 25.83;

  // Start of packet marker
  uint8_t syncByte = 0xAA;
  Serial.write(syncByte);

  // // Send floats
  Serial.write((uint8_t*)&time, sizeof(float));
  Serial.write((uint8_t*)&distance, sizeof(float));
  Serial.write((uint8_t*)&freq, sizeof(float));
  Serial.write((uint8_t*)&P, sizeof(float));
  Serial.write((uint8_t*)&actualPressure, sizeof(float));
  Serial.write((uint8_t*)&actualPressure2, sizeof(float));

  // Serial.println(actualPressure2);
  delayMicroseconds(1);
}
