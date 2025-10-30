/* Sketch to control a 14 cm Noctua fan to exhaust bedroom air, CO2 dependent.
   The 12V fan starts reliably at 5V and allows PWM control using its PWM input.
   This makes it quite silent, suitable for a bedroom.
   My wall is 10 cm thick; a 168mm core drill makes a nice round hole for the fan while allowing standard exhaust grills.
   The fan can be mounted without drills using 10mm thick foam in the hole.
 */

const int fanPwmPin = 9; // pwm output controlled by OCR1A 
const int sensorPwm = 10; // PWM input from CO2 sensor

void setup() {
    Serial.begin(9600);
    pinMode(fanPwmPin, OUTPUT);
    pinMode(sensorPwm, INPUT);  
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

void loop() {
    unsigned long highTime = pulseIn(10, HIGH, 1100000); // Timeout ~1.1s
    unsigned long lowTime = pulseIn(10, LOW, 1100000);
    
    // Calculate CO2
    // sps-siot-carbon-dioxide-crir-e1-sensor-user-guide-000841-ciid-179501.pdf page 5
    // C(CO2) = 2000 × (TH-2) / (TH+TL-4)
    float co2_ppm = 2000.0 * (highTime/1000.0 - 2) / ((highTime + lowTime)/1000.0 - 4);
    
    // Map CO2 to fan speed (400ppm or lower = 10%, 1000ppm = 55%, 2000ppm or higher = 100%)
    float fanSpeed = constrain(map(co2_ppm, 400, 2000, 10, 100), 10, 100);
  
    writeData(co2_ppm, fanSpeed);
    delay(9000); // Including the high/low times results in 10 sec updates
}
