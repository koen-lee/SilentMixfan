 ## Silent Mix Fan

This repo contains a sketch to control a 14 cm Noctua fan to exhaust bedroom air, CO2 dependent.

A central HRV can now extract from wet rooms (bathroom, washing room, toilets) and supply to a central staircase/hallway where the mixfans exchange fresh air. 
This way, no supply ducting is required, greatly simplifying installation and maintenance (duct cleaning).

### My build
The fan I used is a `Noctua NF-A14x25r G2 PWM`, a 12V fan that starts reliably at 5V and allows PWM control using its PWM input.
This makes it quite silent, suitable for a bedroom.

My wall is 10 cm thick; a 168mm core drill makes a nice round hole for the fan while allowing standard exhaust grills.
The fan can be mounted without screws using 10-15mm thick foam in the hole.

The code runs on an `Arduino Nano` or clone, which fits in the same hole or in the grill around it if you find a suitable one.
CO2 concentration is measured with a `Honeywell CRIR-C1` but since we interface with its PWM output, most clones should be compatible. Check the datasheet for the formula to convert the PWM signal to concentration against the code for compatibility. 

I'd suggest to use a 7V power supply for the fan and Nano, power the CO2 sensor through the Arduino 5V pin. Because CO2 sensors actually have a glowing incancescent bulb, they require quite some current, so going to 12v will tax the voltage regulator of the Nano.

No passives are required, so if you don't feel like making a PCB (I had some protoboard lying around) you can even wire it completely with Dupont cables...

### What, no wifi / cloud / dashboard?

Indeed, that is a result of the design philosophy where home infrastructure should be simple, reliable and independent on network infrastructure.
The code does output useful info over the (usb)  serial line though, so it should be trivial to add it when desired.
