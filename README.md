# Repository-for-a-soft-robotics-evaluation-platform

This repository aims to provide a solution for designing a characterisation and control platform for pneumatic-driven soft robotic system. The platform design follows
a top-down approach, starting from the analysis of the required functionalities of a platform to providing a specific design. Please note the design variety exists, only one solution is provided here. We aim to provide some insights and inspirations of designing a platform, which might be useful to the soft robots community, especially for new starters. We are welcoming to further comments/questions, and your contributions/solutions to buid a more general design framework. 

This repository incudes the CAD files for the platform, the designed GUI and the Simulink example for the inverse kinematics implementation using MATLAB/Simulink.


## Platform Overview:
The overeiew of the platform is shown: 

![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/platform.png)


## System Architecture 
The system architecture is: 
![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/arch_rig.png)

Main components: 

[Pressure regulators (Camozzi K8P)](http://catalogue.camozzi.com/CATALOGUES/CCC-GENCAT/00153/PDF/ENG.6.2.10.pdf)

[Tracking system (NDI, Aurora)](https://www.ndigital.com/electromagnetic-tracking-technology/aurora/)

[USB-6341 (NI)](https://www.ni.com/en-gb/support/model.usb-6341.html)

[Arduino Due](https://store.arduino.cc/products/arduino-due)

[AC-DC converter (Mean Well, TP150)](https://uk.rs-online.com/web/p/switching-power-supplies/0414068?redirect-relevancy-data=7365617263685F636173636164655F6F726465723D31267365617263685F696E746572666163655F6E616D653D4931384E525353746F636B4E756D626572267365617263685F6D617463685F6D6F64653D6D61746368616C6C267365617263685F7061747465726E5F6D6174636865643D5E2828282872737C5253295B205D3F293F285C647B337D5B5C2D5C735D3F5C647B332C347D5B705061415D3F29297C283235285C647B387D7C5C647B317D5C2D5C647B377D29292924267365617263685F747970653D52535F53544F434B5F4E554D424552267365617263685F77696C645F63617264696E675F6D6F64653D4E4F4E45267365617263685F6B6579776F72643D3431342D303638267365617263685F6B6579776F72645F6170703D3034313430363826)


[Penumatic connector fitting (SMC, KDM10-04)](https://uk.rs-online.com/web/p/pneumatic-fittings/2453080?redirect-relevancy-data=7365617263685F636173636164655F6F726465723D31267365617263685F696E746572666163655F6E616D653D4931384E525353746F636B4E756D626572267365617263685F6D617463685F6D6F64653D6D61746368616C6C267365617263685F7061747465726E5F6D6174636865643D5E2828282872737C5253295B205D3F293F285C647B337D5B5C2D5C735D3F5C647B332C347D5B705061415D3F29297C283235285C647B387D7C5C647B317D5C2D5C647B377D29292924267365617263685F747970653D52535F53544F434B5F4E554D424552267365617263685F77696C645F63617264696E675F6D6F64653D4E4F4E45267365617263685F6B6579776F72643D3234352D33303830267365617263685F6B6579776F72645F6170703D3234353330383026)

[Slotted panel trunking (Betaduct)](https://uk.rs-online.com/web/p/cable-trunking/8781148?redirect-relevancy-data=7365617263685F636173636164655F6F726465723D31267365617263685F696E746572666163655F6E616D653D4931384E525353746F636B4E756D626572267365617263685F6D617463685F6D6F64653D6D61746368616C6C267365617263685F7061747465726E5F6D6174636865643D5E2828282872737C5253295B205D3F293F285C647B337D5B5C2D5C735D3F5C647B332C347D5B705061415D3F29297C283235285C647B387D7C5C647B317D5C2D5C647B377D29292924267365617263685F747970653D52535F53544F434B5F4E554D424552267365617263685F77696C645F63617264696E675F6D6F64653D4E4F4E45267365617263685F6B6579776F72643D3837382D31313438267365617263685F6B6579776F72645F6170703D3837383131343826)

[LM358AN (TI)](https://www.ti.com/lit/ds/symlink/lm158-n.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1676935629392&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Flm158-n)

[Extruded aluminum, 20 mm x 20 mm](https://ooznest.co.uk/product/v-slot-linear-rail-20x20mm-cut-to-size/)

[Shutdown button (Schneider Electric Harmony XB5)](https://uk.rs-online.com/web/p/emergency-stop-push-buttons/6096186?redirect-relevancy-data=7365617263685F636173636164655F6F726465723D31267365617263685F696E746572666163655F6E616D653D4931384E525353746F636B4E756D626572267365617263685F6D617463685F6D6F64653D6D61746368616C6C267365617263685F7061747465726E5F6D6174636865643D5E2828282872737C5253295B205D3F293F285C647B337D5B5C2D5C735D3F5C647B332C347D5B705061415D3F29297C283235285C647B387D7C5C647B317D5C2D5C647B377D29292924267365617263685F747970653D52535F53544F434B5F4E554D424552267365617263685F77696C645F63617264696E675F6D6F64653D4E4F4E45267365617263685F6B6579776F72643D3630392D36313836267365617263685F6B6579776F72645F6170703D3630393631383626)

[Circuit breaker (Allen Bradley 1492 1492SPM MCB)](https://uk.rs-online.com/web/p/mcbs/1251510)

[Plug socket (CLA3356)](https://uk.rs-online.com/web/p/plug-sockets/1752882)

Apart from using RC circuits to achieve DAC conversion, other solutions could be: 

1. [Adafruit MCP4728](https://www.adafruit.com/product/4470)
2. [MP4725](https://www.sparkfun.com/datasheets/BreakoutBoards/MCP4725.pdf)


Other useful links are: 

[MATLAB DAQ Toolbox](https://www.mathworks.com/products/data-acquisition.html), to connect the USB-6341 to the MATLAB.

[Arduino Support from MATLAB](https://www.mathworks.com/hardware-support/arduino-matlab.html)

[Arduino Support from Simulink](https://www.mathworks.com/hardware-support/arduino-simulink.html)

## GUI Design:
The design GUI is presented here. 
![image](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/My_figures/GUI_interface.png)

## Citation: 

The paper is atttached([see the manuscript](https://github.com/ucl-robotics-ai/test-platform-soft-robotics/blob/main/Paper_manucript/manuscript_accepted_version.pdf)).  The copyright has been tranferred to the IEEE, this work can be cited as follows: 

J. Shi, W. Gaozhang, H. Jin, G. Shi, and H. Wurdemann. "Characterisation and control platform for pneumatically driven soft robots: Design
and applications". International Conference on Soft Robotics (RoboSoft), 2023.
