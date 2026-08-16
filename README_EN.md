# DIY Robot CAR

**This project started with my brother-in-law's idea to make a robot cart**
I have a background in electronics development (microcontrollers, power electronics and audio).
This was unknown territory for my brother-in-law, so as often happens, I offered my help.
Open source software was used for development. As OS I used Linux Fedora 44 desktop KDE plasma.
The challenge is to have the cart carry out these actions autonomously, especially the real-time nature of this is a challenge.
First we built everything on a breadboard. This was not very successful because the wiring often did not provide reliable connections.
Also the choice of the type of stepper motor (unipolar) and servos was inadequate (too light, too much mechanical play)
So version zero has been updated to version 1 with bipolar stepper motors and drivers, MG996 servos with metal gears.
To assemble this, various parts were 3D printed.
In version 1, most of the mechanics were made of sheet material and aluminum profiles.
In version 2 an electronic compass and acceleration sensor have been added. The gripper has also been fundamentally changed and has been 3D printed as much as possible.


The first questions that had to be answered were:
1. What is the scope of the project (what should the robot cart do)
2. What is readily available online
3. What materials are we going to use (motors and sensors)
4. What kind of software are we going to use to make the control possible.
5. presentation of status and measurement data
6. Develop tools and environment
 
1. **Scope: The robot cart must autonomously search for a light source and pick up an object there and bring it to the starting point**

2. **availability:**
1. We could find everything at various hobby stores, Amazon and Aliexpress.
 
3. **Materials**
1. *Motors:*
1. Two independently controllable stepper motors for locomotion.
2. Three independently controllable servo motors for the robot arm and gripper.
*Sensors:*
1. Ultrasonic sensor to measure the distance from the cart to the object.
2. Two light sensitive resistors to find the direction of the light source.
3. We chose the RP2040 of Raspberry Pi as the microcontroller, in version 2 the RP2350W.
4. Optional compass/acceleration sensor.
5. Optionally a WiFi connection with a web socket to read sensor data and issue commands.
    
4. **Programming language**
For the programming language we chose Micropython, which is easier to learn than C/C++.
The advantage is that no translation is required and many ready-made library components are built-in.
To get a good real-time working system there are a number of challenges.
Since Micropython is an interpreter, it runs a factor of 100 slower than C.
There are a number of options to optimize the real-time properties.
1. Using the PIO state machines integrated into the microcontroller.
2. Divide the Software modules (workload) over the two CPU cores
3. Possibly use cooperative multitasking (asyncio).
    
5. **Presentation**
As a presentation, we have a multi-color LED and an OLED display of 128x64 dots as a status message.
With version two we opted for the RP2350W. This has 2 M33 instead of the M0 cores.
With this processor update, calculating the compass/gear is much faster, and
remote control/reading is also possible with a web socket.
There are also two LEDs that signal 5V and 3.3V.
The REPL of the IDE tools is also present to display information with print commands.
    
6. **Development tools**
To design and produce the printed circuit board for electronics I use:
1. Schematic and layout: KiCAD (now version 10.4), FlatCAM Gerber to Gcode, CNC machine an upgraded 3018 with GRBLHAL controller.
2. Mechanical design: FreeCAD for 3D parametric models and a Prusa MK4 3D printer.
3. Software development environment: IDE VSCODE with micropico extension and AI claude 4.6 chat.
4. Thonny IDE simpler development environment than VSCODE.
    

