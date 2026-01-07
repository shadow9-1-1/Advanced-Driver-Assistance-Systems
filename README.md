# Advanced-Driver-Assistance-Systems
Embedded Systems project that required to design and implement common ADAS (Advanced Driver-Assistance Systems) shipped in modern cars and EVs (Electric Vehicles).

---
1) Based on the input from the driver on the acceleration pedal, the microcontroller generates the corresponding output to the motor driving system to move the car. The vehicle speed is displayed on the LCD. When the brake pedal is pressed, the car decelerates until it stops regardless the status of the accelerator pedal. Vehicle system dynamics have to be preserved: no sudden stop. An ultrasonic sensor is used so that the car will decelerate if the TTC (time-to-collision) is less than the time needed to bring a car to halt.

2) Based on the surrounding light, the headlights of the vehicle are turned ON/OFF. An indicator in the LCD shows the status of the vehicle lights.

3) Before starting a trip, the vehicle does a check on seat belts and door locks. The car doesn’t move if any detected occupant doesn’t fasten his belt, or a door is still open.

4) All system parameters should be reported via a Bluetooth connection to a central data logging system.
