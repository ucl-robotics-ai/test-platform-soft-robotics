# GUI with its source code. 
The GUI can be installed directly when the Matlab is available. The source codes are also made accessible. As such, it can be editted if necessary.


# Instruction
This section presents the GUI details and its instruction.

## Overview
Port: See NI Max- Devices and Interfaces
Baud Rate:  9600; 14400; 19200; 38400; 57600; 115200; 921600; 230400
Input Pressure: 0.003 bar < Pressure < 3bar

 
Steps:
-Select the mode. 

-Press "Initialization" to initialize the Aurora tracking system or force sensor. For the Aurora System, users need to set the port and the baud rate before initialization could be done. 

-Set the time, control period, and input pressure. For the input pressure, there are four default wave types: constant wave, sine wave, triangular wave, and step wave. By pressing "setting", the wave pattern could be edited. After all the pressure input settings are done, press "OK" to see the expected input. When using "user defined" pressure, the control period is generated automatically.

-Press "send" to send the input pressure, wait for seconds before output generated. Users could go to the output pages to check and save the data. 

-Press "Reset" in the setting page to clear all the data before generating the next test.

Mode:

Position tracking: to track the position of the manipulator under differnt pressure. Aurora System needs to be initialized.

Chamber actuation: to actuate the soft robot and verify the pressure input by comparing to the command pressure. 

Force identification: measuring the force and torque of the manipulator by the force sensor. Force Sensor needs to be initialized.

Stiffness identification: comparing the position of the manipulator with and without load applied. Aurora Systme needs to be initialized. In this mode, only constant pressure input is accepted.

![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/GUI_page0.png)
