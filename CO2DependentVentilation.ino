/* Sketch to control a 14 cm Noctua fan to exhaust bedroom air, CO2 dependent.
   The 12V fan starts reliably at 5V and allows PWM control using its PWM input.
   This makes it quite silent, suitable for a bedroom.
   My wall is 10 cm thick; a 168mm core drill makes a nice round hole for the
   fan while allowing standard exhaust grills. The fan can be mounted without
   drills using 10mm thick foam in the hole.
 */

const int fanPwmPin = 9;  // pwm output controlled by OCR1A
const int sensorPwm = 10; // PWM input from CO2 sensor
const int fanRpm = 3;     // RPM input from fan, 2 pulses/rev
const int ledPin =
    13; // diagnostics:
        /// 1 second period blink = OK, duty cycle corresponds with CO2
        /// concentration
        /// 5 blinks = no signal from sensor
        /// 3 blinks = no signal from fan
        /// not on = program not running/crashed

// Circular buffer for CO2 readings
const int BUFFER_SIZE = 8;
long co2Readings[BUFFER_SIZE];
int readingIndex = 0;
bool bufferFilled = false;

// Error tracking
int consecutiveErrors = 0;
const int MAX_ERRORS_BEFORE_FALLBACK = 5;

// PI controller parameters
const float SETPOINT = 800.0; // Target CO2 level in ppm
const float KP = 0.05;        // Proportional gain (slow response)
const float KI = 0.002;       // Integral gain (slow response)
float integral = 0.0;         // Integral accumulator
const float MIN_FAN_SPEED = 10.0;
const float MAX_FAN_SPEED = 100.0;

void setup() {
  Serial.begin(9600);
  pinMode(fanPwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPwm, INPUT_PULLUP);
  pinMode(fanRpm, INPUT);
  setupNoctuaPwm();
  Serial.print("{ info: \"https://github.com/koen-lee/SilentMixfan\" }");
}

void setupNoctuaPwm() {
  // Set Timer1 to Fast PWM mode with ICR1 as TOP
  TCCR1A = _BV(COM1A1) | _BV(WGM11);            // Non-inverting mode, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Fast PWM, no prescaler
  // Noctua fans want a 25 kHz signal +-4kHz for an inaudible control
  ICR1 = 639; // TOP value for 25 kHz (16MHz / 640 = 25kHz)
}

void setFanSpeed(float percentDutyCycle) {
  int registerValue = map(percentDutyCycle, 0, 100, 0, 639);
  OCR1A = registerValue;
}

void writeData(float co2_ppm, float fanSpeed) {
  Serial.print("{ co2ppm: ");
  Serial.print(co2_ppm, 1);
  Serial.print(", fanPwm: ");
  Serial.print(fanSpeed);
  Serial.println(" }");
}

bool waitForState(int pin, bool targetState, unsigned long timeoutUs) {
  unsigned long start = micros();
  while (digitalRead(pin) != targetState) {
    if (micros() - start > timeoutUs)
      return false;
  }
  return true;
}

bool measurePulse(int pin, unsigned long *highTime, unsigned long *lowTime,
                  unsigned long timeoutUs) {
  // Wait for initial state (with timeout)
  if (!waitForState(pin, LOW, timeoutUs)) {
    return false;
  }

  digitalWrite(ledPin, LOW);
  if (!waitForState(pin, HIGH, timeoutUs)) {
    return false;
  }
  unsigned long startHigh = micros();
  digitalWrite(ledPin, HIGH);
  if (!waitForState(pin, LOW, timeoutUs)) {
    return false;
  }
  digitalWrite(ledPin, LOW);
  unsigned long startLow = micros();
  if (!waitForState(pin, HIGH, timeoutUs)) {
    return false;
  }
  unsigned long endLow = micros();

  *highTime = startLow - startHigh;
  *lowTime = endLow - startLow;

  return true;
}

long calculateCO2(unsigned long highTimeUs, unsigned long lowTimeUs) {
  // Convert microseconds to milliseconds
  long thMs = highTimeUs / 1000;
  long tlMs = lowTimeUs / 1000;

  // sps-siot-carbon-dioxide-crir-e1-sensor-user-guide-000841-ciid-179501.pdf
  // page 5 CO2 = 2000 * (thMs - 2) / (thMs + tlMs - 4)

  long co2_ppm = (2000L * (thMs - 2)) / (thMs + tlMs - 4);
  return co2_ppm;
}

void blink(int times) {
  digitalWrite(ledPin, LOW);
  for (int i = 0; i < times; i++) {
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
  }
  delay(100);
}

long getControlReading(long newReading) {
  co2Readings[readingIndex] = newReading;
  readingIndex++;
  if (readingIndex >= BUFFER_SIZE) {
    readingIndex = 0;
    bufferFilled = true;
  }
  return calculateMedian();
}

long calculateMedian() {
  long sortedReadings[BUFFER_SIZE];
  int validCount = bufferFilled ? BUFFER_SIZE : readingIndex;

  for (int i = 0; i < validCount; i++) {
    sortedReadings[i] = co2Readings[i];
  }

  // Simple bubble sort is OK for small arrays
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = 0; j < validCount - i - 1; j++) {
      if (sortedReadings[j] > sortedReadings[j + 1]) {
        long temp = sortedReadings[j];
        sortedReadings[j] = sortedReadings[j + 1];
        sortedReadings[j + 1] = temp;
      }
    }
  }

  // Return median
  if (validCount % 2 == 0) {
    return (sortedReadings[validCount / 2 - 1] +
            sortedReadings[validCount / 2]) /
           2;
  } else {
    return sortedReadings[validCount / 2];
  }
}

float calculatePIControl(float measuredValue) {
  // Calculate error (positive error means CO2 is above setpoint)
  float error = measuredValue - SETPOINT;

  // Update integral term
  integral += error;

  // Anti-windup: constrain integral to prevent excessive accumulation
  float maxIntegral = (MAX_FAN_SPEED - MIN_FAN_SPEED) / KI;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  // Calculate PI output
  float output = KP * error + KI * integral;

  // Add base speed and constrain to valid range
  float fanSpeed = MIN_FAN_SPEED + output;
  fanSpeed = constrain(fanSpeed, MIN_FAN_SPEED, MAX_FAN_SPEED);

  return fanSpeed;
}

void loop() {
  const unsigned long timeOutUs = 2000000UL;
  unsigned long highTime, lowTime;
  if (measurePulse(sensorPwm, &highTime, &lowTime, timeOutUs)) {
    consecutiveErrors = 0; // Reset error counter on successful read

    long co2_ppm = calculateCO2(highTime, lowTime);

    long medianCO2 = getControlReading(co2_ppm);

    // Use PI controller to calculate fan speed
    float fanSpeed = calculatePIControl(medianCO2);

    writeData(medianCO2, fanSpeed);
    setFanSpeed(fanSpeed);
  } else {
    consecutiveErrors++;
    writeDebug(highTime, lowTime);
    Serial.print("{ error: \"Sensor timeout\", consecutiveErrors: ");
    Serial.print(consecutiveErrors);
    Serial.print(" }\n");

    if (consecutiveErrors >= MAX_ERRORS_BEFORE_FALLBACK) {
      setFanSpeed(50.0);
      blink(5);
    }
  }
  if (measurePulse(fanRpm, &highTime, &lowTime, timeOutUs)) {
    unsigned long timePerRevUs = 2UL * (lowTime + highTime);
    unsigned long usPerMinute = (60UL * 1000UL * 1000UL);
    unsigned long rpm = usPerMinute / timePerRevUs;
    Serial.print("{ rpm: ");
    Serial.print(rpm);
    Serial.print(" }\n");
  } else {
    Serial.print("{ error: \"Fan blocked\" }\n");
    blink(3);
  }
}
