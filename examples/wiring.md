# Wiring for the Example
To get the example code to work, wire up two Teensy 3.x or 4.x boards as shown in the following table:

| Master         | Slave          |
|----------------|----------------|
| GND            | GND            |
| CS0 (Pin 10)   | CS0 (Pin 10)   |
| MOSI0 (Pin 11) | MISO0 (Pin 12) |
| MISO0 (Pin 12) | MOSI0 (Pin 11) |
| SCK0 (Pin 13)  | SCK0 (Pin 13)  |
