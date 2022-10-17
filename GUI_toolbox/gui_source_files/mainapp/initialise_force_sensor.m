function initialise_force_sensor
%% Function:
% 1. generate 3 output PWM channels and 3 input analog channels.
%    3 PWM channels for generating 0-100% duty pwm
%    3 input channel for measuring real-time pressure
% 2. The aurora system is integrated as well, which can be used 
%    for measuring real-time position and orientation as well as
%    estimating velocity.
% 3. User-defined function can be added for close-loop control, for
%    real-time control the tip position.
% Author: Jialei Shi
% Data: 14/08/2020
% Modified Data: 08/16/2020

% load pressure_array.mat

%% aurora system initialization
global PositionArray ControlPeriod PositionLastStep n tn pressure 
global  FT_Sensor
%Time_period = 20;
n = 1;
tn = 0;
% saving 5-step position to estimate velocity
PositionArray = zeros(5,3);
%ControlPeriod = 0.05;  % defalut as 25ms
PositionLastStep = zeros(3,1);  % original position
% aurora_device = AuroraDriver('COM3');
% serial_present = instrfind;
%% FT SENSOR SETUP %%%
%% SETUP BEFORE STARTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Change the ethernet connection settings of your laptop to:
%%%% IP address: 192.168.1.2
%%%% Subnet Mask: 255.255.0.0

%%%% Make sure your firewall settings allow UDP connections to the sensor:
%%%% IP address: 192.168.1.1
%%%% Port number 23
%%%% Subnet Mask: 255.255.0.0

%Adds the folder with the UDP code to the current search path
%addpath('IIT_FT_17_Sensor','-end');
disp('initialise F/T sensor...')
%Force sensor UDP buffer size

inputBuffer=100;% This is the UDP buffer size

%  This is the structure of the sensor data
FT_SensorData = struct(...
        'ChRaw_Offs',int16(zeros(6,1)),...
        'FT',int32(zeros(6,1)),...
        'ChRaw',uint16(zeros(6,1)),...
        'tStamp',uint32(zeros(2,1)),...
        'filt_FT',int32(zeros(6,1)),...
        'UDP_PACKET_ID',uint32(zeros(1,1)),...
        'ft',double(zeros(6,1)),...
        'filt_ft',double(zeros(6,1)),...
        'ctime', double(0));
        
    FT_Sensor_Poll=struct(...
       'Data', FT_SensorData,...
        'Policy0',uint8(0),...
        'Policy1',uint8(0),...
        'UDPPolicy',uint16(0),...
        'BoardNumber',uint8(0),...
        'IP',uint8(16),...
        'Port',uint8(0),...
        'UDPHandle',double(0),...
        'UDPRecvBuff', uint8(zeros(inputBuffer,1)));

     
%     Create 2 instances of FT_Sensor_Poll structures FT_Sensor(1) and FT_Sensor(2)
%     This demonstrates how to get readings from an arbitrary number of sensors 
    for i=1:2   
    FT_Sensor=FT_Sensor_Poll;
    end
    
% Set the Policy0 and Policy1 members of FT_Sensor(1) structure
    FT_Sensor(1).Policy0=215;
    FT_Sensor(1).Policy1=0;
    
% Set the BoardNumber of the Sensor
FT_Sensor(1).BoardNumber=1;
    
  
%************************ Setup Plot **************************
Fx=double(0);
Fy=double(0);
Fz=double(0);
Tx=double(0);
Ty=double(0);
Tz=double(0);

%figure(1);
%axis equal
%hold on
%a Handle to ploted force vector
%h1=plot3(Fx,Fy,Fz);
%hold on
%A Handle to ploted component forces
%h2=plot3(Fx,Fy,Fz);
%hold on
%A handle to ploted torques
%h3=plot3(Fx,Fy,Fz);
%hold on
%Set Viewpoint
%azimuth=45;
%elevation=15;
%view(azimuth, elevation);
%grid on

% ****************************Setup UDP********************
echoudp('off')% first disable the Echo of the UDP
fclose('all')%close opened files and connections

echoudp('on',4012);

FT_Sensor(1).UDPHandle=udp('192.168.1.1',23);% IP Address and port of the sensor 
set(FT_Sensor(1).UDPHandle,'DatagramTerminateMode', 'off')
FT_Sensor(1).UDPHandle.Timeout =0.1;
FT_Sensor(1).UDPHandle.InputBufferSize=inputBuffer;
fopen(FT_Sensor(1).UDPHandle)
% *********************************************************

SendUDPcommand('SET_SINGLE_UDP_PACKET_POLICY',FT_Sensor(1));

SendUDPcommand('GET_SINGLE_UDP_PACKET',FT_Sensor(1));
 
SendUDPcommand('UDP_CALIBRATE_OFFSETS',FT_Sensor(1));
disp('initialization completed');
end
