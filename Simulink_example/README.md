# Example for implementing control using Simulink
The simulink file contains the structure to implement inverse kinematics control, using or modifined by using a higher version above MATLAB/Simulink R2022a.


## pressure processing
This block collects the desired & real pressure from the six channels which can be saved and visualised.
![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/pressure_processing.png)

This block generates the desired shapes and does the inverse kinematic control. The trajectory is collected from the Aurora tracking system. 
![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/inverse_control.png)
