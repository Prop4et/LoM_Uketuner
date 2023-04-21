# Automatic Uke Tuner
The project consists in creating a (semi) automatic uke tuner by using an arduino uno board, a microphone and a stepper motor. Everything is also connected to a small circuit used to see which note we are currently tuning, if we are in tune and used also to rotate the stepper motor and put it in the correct position

### The idea
The idea was to create a somehow portable automatic uke tuner which uses a stepper motor to turn the uke keys in relation to the frequencies listened through a microphone. Everything needed to be controlled by an Arduino UNO R3, based on the atmega328P chip.

### The execution
During the development of the project different ideas actually overlapped and were explored, starting from a portable tuner the project evolved into a non portable tuner with the Arduino controlling 4 different stepper motors, one for each uke key. This way i can create a place where i could put the uke and tune it automatically. This idea was crushed when the first stepper that i tried wasn't strong enough to turn the keys and so i needed to find a stronger stepper that meant find a more expensive motor. Moreover the stepper needed the H-Bridge to be controlled, starting with a L298N bridge that could be used to power and controll a stepper and to alsow power the arduino and then shifting towards the drv8825 driver which allowed for a more precise control of the stepper but couldn't split the voltage spullied to the motor thus the need to a separate power source for the Arduino.

The final and probably tallest wall to face was the design of the key-crankshaft connector. I had an initial idea in mind for the connector with 3 cylinders put on the verices of a triangle to hold the key in position. This shape wasn't holding the key correcly so i changed it to 4 cylinders staying on the vertices of a rectangle. 
What needed to change was also the coupler between the crankshaft and the actual holder for the key. With the first motor i had a circular shaped crankshaft and i had to design a coupler that could be tightened with some screws. These motors didn't have enough power so by going to the strongest one i had another shape for the crankshaft that allowed me to change the shape of the coupler making it in one piece. The crankshaft has in fact a flat section that helps in holdin the coupler in place.

## Components used
+ Arduino UNO R3
+ Stepper NEMA 17HE19-2004S
+ Driver drv8825
+ Power source ~16V
+ Capacitor 100uF
+ Microphone Diamex Ky-037
+ LCD screen
+ I2C adapter for LCD
+ 2 LEDs (one green, one red)
+ 2 resistors (160 ohm)
+ 2 buttons  
+ 10K Ohm potentiometer
+ 1 switch
+ jumpers and wires

## Pinout
| Arduino | Diamex Ky-037 |
| ----------------- | -------------- |
| GND | VCC |
| GND | GND |
| A0 | A0 |
| D6 | D0 |

| Arduino | LCD |
| ----------------- | -------------- |
| 5V | VCC |
| GND | GND |
| A4 | SDA |
| A5 | SCL |

| Arduino | DRV 8825 |
| ----------------- | -------------- |
| 5V | M0 |
| 5V | RST |
| 5V | SLP |
| 12 | dir |
| 11 | stp |
| GND | GND Logic |

| DRV 8825 | NEMA17/Power Source |
| ----------------- | -------------- |
| VMOT | 16V |
| GNDMOT | GND |
| B2 | B+ |
| B1 | B- |
| A1 | A+ |
| A2 | A- |


| Arduino | Red LED |
| ----------------- | -------------- |
| 5 | VCC |
| GND | GND |

| Arduino | Green LED |
| ----------------- | -------------- |
| 4 | VCC |
| GND | GND |

| Arduino | Selector Button |
| ----------------- | -------------- |
| 5V | VCC |
| 3 | GND |

| Arduino | Motor Button |
| ----------------- | -------------- |
| 5V | VCC |
| 2 | GND |

| Arduino | Potentiometer |
| ----------------- | -------------- |
| 5V | VCC |
| A1 | SDA | 
| GND | GND |


![game_join](images/circuit.PNG)

For more information you can check the ![PDF](doc.pdf) 