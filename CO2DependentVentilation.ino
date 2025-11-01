/* Sketch to control a 14 cm Noctua fan to exhaust bedroom air, CO2 dependent.
   The 12V fan starts reliably at 5V and allows PWM control using its PWM input.
   This makes it quite silent, suitable for a bedroom.
   My wall is 10 cm thick; a 168mm core drill makes a nice round hole for the fan while allowing standard exhaust grills.
   The fan can be mounted without drills using 10mm thick foam in the hole.
 */
 
const int fanPwmPin = 9;  // pwm output controlled by OCR1A 
const int sensorPwm = 10; // PWM input from CO2 sensor
const int fanRpm = 3;     // RPM input from fan, 2 pulses/rev
const int ledPin = 13;    // diagnostics:
 /// 1 second period blink = OK, duty cycle corresponds with CO2 concentration and fan speed
 /// 1 second on, 1 second off = no signal from sensor
 /// not on = program not running/crashed

void setup() {
    Serial.begin(9600);
    pinMode(fanPwmPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(sensorPwm, INPUT_PULLUP);  
    pinMode(fanRpm, INPUT);  
    setupNoctuaPwm();
}

void setupNoctuaPwm() {
    // Set Timer1 to Fast PWM mode with ICR1 as TOP
    TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Non-inverting mode, Fast PWM
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);  // Fast PWM, no prescaler
    // Noctua fans want a 25 kHz signal +-4kHz for an inaudible control
    ICR1 = 639;  // TOP value for 25 kHz (16MHz / 640 = 25kHz)
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

void writeDebug(long timeHigh, long timeLow) {
    Serial.print("{ high: ");
    Serial.print(timeHigh, 1);
    Serial.print(", low: ");
    Serial.print(timeLow);
    Serial.println(" }");
}

bool waitForState(int pin, bool targetState, unsigned long timeoutUs) {
  unsigned long start = micros();
  while(digitalRead(pin) != targetState) {
    if (micros() - start > timeoutUs) return false;
  }
  return true;
}

bool measurePulse(int pin, unsigned long* highTime, unsigned long* lowTime, unsigned long timeoutUs) {
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
  
  // CO2 = 2000 * (thMs - 2) / (thMs + tlMs - 4)
  long co2_ppm = (2000L * (thMs - 2)) / (thMs + tlMs - 4);
  
  return co2_ppm;
}

void blink(int times) {
  digitalWrite(ledPin, LOW);
  for( int i = 0; i < times; i++){
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
  }
  delay(100);
}

void loop() {
  const unsigned long timeOutUs = 2000000UL;
  unsigned long highTime, lowTime;
  if (measurePulse(sensorPwm, &highTime, &lowTime, timeOutUs)) {
      long co2_ppm = calculateCO2(highTime, lowTime);
       
      // Map CO2 to fan speed (400ppm or lower = 10%, 1000ppm = 55%, 2000ppm or higher = 100%)
      float fanSpeed = constrain(map(co2_ppm, 400, 2000, 10, 100), 10, 100);
    
      writeData(co2_ppm, fanSpeed);
      setFanSpeed(fanSpeed);
  } 
  else
  {
      writeDebug(highTime, lowTime);
      Serial.print("{ error: \"Sensor timeout\" }\n");
      setFanSpeed(50.0);
      blink(5);
  }
  if( measurePulse(fanRpm, &highTime, &lowTime, timeOutUs) )
  {
      unsigned long timePerRevUs = 2UL*(lowTime + highTime);
      unsigned long usPerMinute = (60UL * 1000UL * 1000UL ); 
      unsigned long rpm = usPerMinute / timePerRevUs;
      Serial.print("{ rpm: ");
      Serial.print(rpm);
      Serial.print(" }\n");
      
  } else {
      Serial.print("{ error: \"Fan blocked\" }\n");
      blink(3);
  }
  
}
