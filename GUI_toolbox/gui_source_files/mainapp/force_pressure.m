clc
clear all
close all
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
global aurora_device PositionArray ControlPeriod PositionLastStep n tn pressure 
global  FT_Sensor
Time_period = 20;
n = 1;
tn = 0;
% saving 5-step position to estimate velocity
PositionArray = zeros(5,3);
ControlPeriod = 0.05;  % defalut as 25ms
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
addpath('IIT_FT_17_Sensor','-end');
disp('initialise F/T sensor...')
%Force sensor UDP buffer size
inputBuffer = 100;
%Set up Force sensor COMMS data structure
FT_SensorData = struct(...
        'ChRaw_Offs',int16(zeros(6,1)),...
        'FT',int32(zeros(6,1)),...
        'ChRaw',uint16(zeros(6,1)),...
        'temp_Vdc',int32(0),...
        'tStamp',int32(0),...
        'fault',int16(0),...
        'filt_FT',int32(zeros(6,1)),...
        'ft',double(zeros(6,1)),...
        'filt_ft',double(zeros(6,1)));
    
FT_Sensor = struct(...
       'Data', FT_SensorData,...
        'Policy0',uint8(0),...
        'Policy1',uint8(0),...
        'UDPPolicy',uint16(0),...
        'BoardNumber',uint8(0),...
        'IP',uint8(16),...
        'Port',uint8(0),...
        'UDPHandle',double(0),...
        'UDPRecvBuff', uint8(zeros(inputBuffer,1)));    

% %Provision for multiple sensors, must be kept in, otherwise the struct
% %sizes are mismatched and errors are thrown around.
% for i = 1:2   
%     
%     FT_Sensor(i) = FT_Sensor_Poll;
%     
% end  
% FT_Sensor = FT_Sensor_Poll;  

%Set the force sensor mandatory broadcast policies, Policy1 = 0 is required
%Policy0 determines the amount of data transmitted, at Policy0 = 127,
%forces and torques for axes are broadcast.
FT_Sensor(1).Policy0 = 127;
FT_Sensor(1).Policy1 = 0;

%Board number definition, by default = 1, can be anything.
FT_Sensor(1).BoardNumber = 1;

%% UDP SENSOR CONNECTION SETUP %%

% Clear the connection and restart in case anything else was previously
% left running
echoudp('off'); 
fclose('all');
echoudp('on',4012);

% Set values to the default address and port of IIT-FT sensors
FT_Sensor(1).UDPHandle = udp('192.168.1.1',23);
set(FT_Sensor(1).UDPHandle,'DatagramTerminateMode', 'off');
FT_Sensor(1).UDPHandle.Timeout = 0.1;
FT_Sensor(1).UDPHandle.InputBufferSize = inputBuffer;

%Change the default "off" of enabling port sharing so the data can be read
FT_Sensor(1).EnablePortSharing = 'on';

%Open a UDP connection at the above IP and port.
fopen(FT_Sensor(1).UDPHandle);

%Call the other functions, give them command strings and they
%translate and create and populate appropriate datagrams. These datagrams
%are then sent to buffer and broadcast to the sensor.

% sets the mandatory policies
SendUDPcommand('SET_SINGLE_UDP_PACKET_POLICY',FT_Sensor(1));
% receives an echo
SendUDPcommand('GET_SINGLE_UDP_PACKET',FT_Sensor(1));
% zeroes the sensor values
SendUDPcommand('UDP_CALIBRATE_OFFSETS',FT_Sensor(1));
disp('initialisation of F/T sensor is complete')



%% Generate DAQ session
disp('initialise NI-USB....')
s = daq.createSession('ni');

%% Session Properties
% Set properties that are not using default values.
% Continuous recording and generating
s.IsContinuous = true;
% Scanning rate, Set Racording rate number of points/second.
% If want to capture the output PWM, set this value equals to
% 10*PWM_frequency
% set too high will introduce delay
s.Rate = 500;
% auto ratio is 0.1,means every 100ms, 'DataAvailable' will be checked
% So call function period is 0.1s
% disable auto setting.
s.IsNotifyWhenDataAvailableExceedsAuto = 0;
% Set checking period at 10ms
% Waring: On this platform, notifications more frequent than 20 times
%per second may not be achievable
s.NotifyWhenDataAvailableExceeds = s.Rate*ControlPeriod;
% Add Channels to Session
% Add channels and set channel properties, if any.
% Add Analog Input Channels
% check PWM
% ach1 = addAnalogInputChannel(s,'Dev1','ai4','Voltage');
% ach2 = addAnalogInputChannel(s,'Dev1','ai5','Voltage');
% ach3 = addAnalogInputChannel(s,'Dev1','ai13','Voltage');

% Feedback channels
ach1 = addAnalogInputChannel(s,'Dev1','ai15','Voltage');%ai15,ai14 for test
ach2 = addAnalogInputChannel(s,'Dev1','ai12','Voltage');
ach3 = addAnalogInputChannel(s,'Dev1','ai6','Voltage');

% Set voltage type as SingleEnd
ach1.TerminalConfig = 'SingleEnded';
ach2.TerminalConfig = 'SingleEnded';
ach3.TerminalConfig = 'SingleEnded';

% Add PWM Generation Channels
% |PulseGeneration| measurement type. An analog input channel is added to
% monitor the pulse generated by the counter output channel. 
% 0 = 'ctr0' = PFI12 = P2.4; 1 = 'ctr1' = PFI13 = P2.5
% 2 = 'ctr2' = PFI14 = P2.6; 3 = 'ctr3' = PFI15 = P2.7
addCounterOutputChannel(s,'Dev1', 0, 'PulseGeneration');
addCounterOutputChannel(s,'Dev1', 1, 'PulseGeneration');
addCounterOutputChannel(s,'Dev1', 2, 'PulseGeneration');

% Initial Value for ctr0
s.Channels(4).Frequency = 1000;
s.Channels(4).DutyCycle = 0.001;
s.Channels(4).InitialDelay = 0;
% Initial Value for ctr1
s.Channels(5).Frequency = 1000;
s.Channels(5).DutyCycle = 0.001;
s.Channels(5).InitialDelay = 0;
% Initial Value for ctr2
s.Channels(6).Frequency = 1000;
s.Channels(6).DutyCycle = 0.001;
s.Channels(6).InitialDelay = 0;
% Initialize Session UserData Property
% Initialize the custom fields for managing the acquired data across callbacks.
% Add wanted recording data to here.
s.UserData.Data = [];
s.UserData.TimeStamps = [];
s.UserData.StartTime = [];
s.UserData.PWMduty = [];
s.UserData.Position = [];
s.UserData.RotationAngle = [];
s.UserData.RotationMatirx = [];
s.UserData.Force = [];
s.UserData.Torque= [];
s.UserData.Timeinterval = [];
s.UserData.quat = [];
s.UserData.Velocity1 = [];
s.UserData.Velocity2 = [];
disp('initialisation of NI-USB is complete')
% Add Listeners
% Add listeners to session for available data and error events.
lh1 = addlistener(s, 'DataAvailable', @recordData);
lh2 = addlistener(s, 'ErrorOccurred', @(~,eventData) disp(getReport(eventData.Error)));

% Acquire Data
% Start the session in the background.
startBackground(s)
pause(Time_period) % Increase or decrease the duration time to fit your needs.

% stop NI-USB device
stop(s)

% stop Aurora system
% aurora_device.stopTracking();
% delete(aurora_device);

% Log Data
figure(1)
% Convert the acquired data and timestamps to a timetable in a workspace variable.
ai0 = s.UserData.Data(:,1);
ai1 = s.UserData.Data(:,2);
ai2 = s.UserData.Data(:,3);
DAQ_pressure = timetable(seconds(s.UserData.TimeStamps),s.UserData.Data/3-0.08);
DAQ_cmd_pressure = timetable(seconds((s.UserData.Timeinterval)'),3*(s.UserData.PWMduty));
% force information
force_raw = s.UserData.Force;
torque_raw = s.UserData.Torque;
force = timetable(seconds((s.UserData.Timeinterval)'),force_raw);
torque  = timetable(seconds((s.UserData.Timeinterval)'),torque_raw);
% Aurora_position = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.Position);
% Aurora_quat = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.quat);
% Aurora_z_axis = s.UserData.RotationMatirx(:,3:3:size(s.UserData.RotationMatirx,2));
% theta = acos(sum(Aurora_z_axis.*[0;0;1]))/pi*180;
% Aurora_theta = timetable(seconds((s.UserData.Timeinterval)'),theta');
File_name = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'Type1_force1.mat');
% DAQ_pressure = timetable(seconds(s.UserData.TimeStamps),ai0);
% Plot Data
% Plot the acquired data on labeled axes.
plot(DAQ_pressure.Time, DAQ_pressure.Variables)
hold on
plot(DAQ_cmd_pressure.Time, DAQ_cmd_pressure.Variables)
title('pressure')
xlabel('Time')
ylabel('bar')
legend('real_pressure','cmd_pressure')

figure (2)
subplot(2,1,1);
plot(force.Time,force.Variables)
title('force')
legend('x-force', 'y-force','z-force')
subplot(2,1,2);
plot(torque.Time,torque.Variables)
title('torque')
legend('x-torque', 'y-torque','z-torque')

figure(3)
a=size(s.UserData.TimeStamps);
b=size(s.UserData.Timeinterval);
step=a(1)/b(2);
feedback_pressure = s.UserData.Data/3-0.08;
plot( feedback_pressure(1:step:end,2),s.UserData.Force(:,3));
% figure(3)
% plot()
% figure (2)
% raw_analog_signal = s.UserData.Data(:,1);
% feeback_signal = timetable(seconds(s.UserData.TimeStamps),raw_analog_signal);
% DAQ_cmd_signal = timetable(seconds((s.UserData.Timeinterval)'),10*(s.UserData.PWMduty));
save(File_name,'DAQ_cmd_pressure','force','DAQ_pressure');
% % legend(DAQ_2.Properties.VariableNames)
% plot(feeback_signal.Time, feeback_signal.Variables)
% hold on
% plot(DAQ_cmd_signal.Time, DAQ_cmd_signal.Variables)
% plot position and rotation
% figure(2)
% subplot(3,1,1)
% plot(s.UserData.Timeinterval,s.UserData.Position(:,1),...
%      s.UserData.Timeinterval,s.UserData.Position(:,2),...
%      s.UserData.Timeinterval,s.UserData.Position(:,3));
% subplot(3,1,2)
% plot(s.UserData.Timeinterval,s.UserData.RotationAngle(:,1)/pi*180,...
%      s.UserData.Timeinterval,s.UserData.RotationAngle(:,2)/pi*180,...
%      s.UserData.Timeinterval,s.UserData.RotationAngle(:,3)/pi*180);
%  % plot theta
% subplot(3,1,3)
% plot(s.UserData.Timeinterval,theta);
% figure(3)
% subplot(3,1,1)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,1),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,1));
%  
% subplot(3,1,2)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,2),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,2));
%  
% subplot(3,1,3)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,3),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,3));
%  
% % Clean Up
% Remove event listeners and clear the session and channels, if any.
delete(lh1)
delete(lh2)
% clear aurora_device
% fclose(port);
% clear s lh1 lh2

% Callback Function
% Define the callback function for the 'DataAvailable' event.
% generating from matlab auto-generation
function recordData(src, eventData)
global n FT_Sensor
% RECORDDATA(SRC, EVENTDATA) records the acquired data, timestamps and
% trigger time. You can also use this function for plotting the
% acquired data live.

% SRC       - Source object      i.e. Session object
% EVENTDATA - Event data object  i.e. 'DataAvailable' event data object
if n == 1
    disp('start collecting...')
end

n = n + 1;
% Record the data and timestamps to the UserData property of the session.
src.UserData.Data = [src.UserData.Data; eventData.Data];
src.UserData.TimeStamps = [src.UserData.TimeStamps; eventData.TimeStamps];
src.UserData.Timeinterval  = [src.UserData.Timeinterval,eventData.TimeStamps(1)];

%% force sensor data
% save the force
FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));
Fx = FT_Sensor(1).Data.ft(1);
Fy = FT_Sensor(1).Data.ft(2);
Fz = -FT_Sensor(1).Data.ft(3); 
src.UserData.Force = [src.UserData.Force;Fx, Fy, Fz];
% save the torque
Tx = FT_Sensor(1).Data.ft(4);
Ty = FT_Sensor(1).Data.ft(5);
Tz = FT_Sensor(1).Data.ft(6);
src.UserData.Torque = [src.UserData.Torque;Tx, Ty, Tz];
% Derive the Aurora position And Orientation information     
% aurora_device.updateSensorDataPositionAndRotation();
% get the position in three directions, unit is m
% temP =  aurora_device.port_handles.trans/1000;
% get the rotation angle around x,y,z
% temrot = aurora_device.port_handles.rot;
% src.UserData.quat = [src.UserData.quat;temrot];
% src.UserData.RotationMatirx = [src.UserData.RotationMatirx,quat2rotm(temrot)];
% [temRx,temRy,temRz] = quat2angle(temrot);
%[R,~] =  aurora_device.measureTipOrientation();
% src.UserData.Position = [src.UserData.Position;temP];
% src.UserData.RotationAngle = [src.UserData.RotationAngle;[temRx,temRy,temRz]];

% velocity estimation function
% or add you own velocity estiomation function here
% <temP> is current step position and the time period is <ControlPeriod> 
% [temVx1,temVy1,temVz1] = getVelocitySimplified(temP);
% src.UserData.Velocity1 = [src.UserData.Velocity1; [temVx1,temVy1,temVz1]];
% 
% [temVx2,temVy2,temVz2] = getVelocity5Points(temP);
% src.UserData.Velocity2 = [src.UserData.Velocity2; [temVx2,temVy2,temVz2]];

% Add any self-defined PWM generation function here
%-------------------------------------------------------%
%change PWM duty in a user-defined way
[pwmtemp1,pwmtemp2,pwmtemp3] = Ramp3_PWM(eventData.TimeStamps(1));
% time = eventData.TimeStamps
src.Channels(4).DutyCycle = pwmtemp1;
src.Channels(5).DutyCycle = pwmtemp2;
src.Channels(6).DutyCycle = pwmtemp3;
%------------------------------------------------------%

% Record PWM duty to the UserData property of the session.
src.UserData.PWMduty = [src.UserData.PWMduty;...
                       [pwmtemp1,pwmtemp2,pwmtemp3]];
% Record the starttime from the first execution of this callback function.

if isempty(src.UserData.StartTime)
    src.UserData.StartTime = eventData.TriggerTime;
end
%  toc
% Uncomment the following lines to enable live plotting.
% for example:
% figure(8)
% plot(eventData.TimeStamps, eventData.Data)
% xlabel('Time (s)')
% ylabel('Amplitude (V)')
% legend('ai13')
end

%% Subfunctions
% derive the velocity from position
%% method 1
function [vx,vy,vz] = getVelocitySimplified(p)
global PositionLastStep ControlPeriod

CurrentPosition = p;
%  calculate the velocity
vx = (CurrentPosition(1)- PositionLastStep(1))/ControlPeriod; 
vy = (CurrentPosition(2)- PositionLastStep(2))/ControlPeriod;  
vz = (CurrentPosition(3)- PositionLastStep(3))/ControlPeriod;  

% update last step position informaiton
PositionLastStep = CurrentPosition;
end

%% method 2
% f(x(n))'=4/3(f(x(n+1))-f(x(n-1)))/(2t)-1/3(f(x(n+2)-f(x(n-2)))/(4t)
% This will introduce 2*control periods delay
% for instance, if the position updates 25ms, the speed estimation 
% will delay for about 50ms.
function [vx,vy,vz] = getVelocity5Points(p)
global PositionArray ControlPeriod
%  update the position array
    for i = 5:-1:2
        PositionArray(i,:) = PositionArray(i-1,:);
    end
    PositionArray(1,:) = p;
    
%  calculate the velocity
vx = (4/3)*(PositionArray(2,1)-PositionArray(4,1))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,1)-PositionArray(5,1))/(4*ControlPeriod);
vy = (4/3)*(PositionArray(2,2)-PositionArray(4,2))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,2)-PositionArray(5,2))/(4*ControlPeriod);
vz = (4/3)*(PositionArray(2,3)-PositionArray(4,3))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,3)-PositionArray(5,3))/(4*ControlPeriod); 
end
%% This function is to generate user-defined variable_PWM duty in three chamber 
%with respects to time 

function [pwm1,pwm2,pwm3] = Variable_PWM(time)
    pwm1 = 0.001;
    pwm2 = 0.001;
    pwm3 = 0.001;
    if time < 3
         pwm1 = 0.05;
          pwm2 = 0.05;
          pwm3 = 0.05;        
    elseif time >= 3 && time < 6
        pwm1 = 0.1;
        pwm2 = 0.1;
         pwm3 = 0.1;
    elseif time >= 6 && time < 9
        pwm1 = 0.15;
       pwm2 = 0.15;
        pwm3 = 0.15;
    elseif time >= 9 && time < 12
       pwm1 = 0.2;
       pwm2 = 0.2;
       pwm3 = 0.2;
    elseif time >= 12 && time < 15
        pwm1 = 0.25;
        pwm2 = 0.25;
        pwm3 = 0.25;
    elseif time >= 15 && time < 18
        pwm1 = 0.3;
        pwm2 = 0.3;
        pwm3 = 0.3;
    elseif time >= 18 && time < 21
        pwm1 = 0.35;
        pwm2 = 0.35;
        pwm3 = 0.35;
    elseif time >= 21 && time < 24
        pwm1 = 0.4;
        pwm2 = 0.4;
        pwm3 = 0.4;
    elseif time >= 24 && time < 27
        pwm1 = 0.45;
        pwm2 = 0.45;
        pwm3 = 0.45;
    elseif time >= 27 && time < 30
        pwm1 = 0.5;
        pwm2 = 0.5;
        pwm3 = 0.5;
    elseif time >= 30 && time < 33
         pwm1 = 0.55;
%         pwm2 = 0.55;
%          pwm3 = 0.55;
    else
        pwm1 = 0.001;
        pwm2 = 0.001;
        pwm3 = 0.001;
    end
end
function [pwm1,pwm2,pwm3] = continous_step_PWM(time)
   if time <=2
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time<=6
       pwm1 = 0.05;
       pwm2 = 0.05;
       pwm3 = 0.05;
   elseif time <=9
       pwm1 = 0.1;
       pwm2 = 0.1;
       pwm3 = 0.1;
   elseif time <=13
       pwm1 = 0.15;
       pwm2 = 0.15;
       pwm3 = 0.15;
   elseif time<=16
       pwm1 = 0.2;
       pwm2 = 0.2;
       pwm3 = 0.2;
   elseif time <=21
       pwm1 = 0.25;
       pwm2 = 0.25;
       pwm3 = 0.25;
   elseif time <=24
       pwm1 = 0.3;
       pwm2 = 0.3;
       pwm3 = 0.3;       
   elseif time <=28
       pwm1 = 0.35;
       pwm2 = 0.35;
       pwm3 = 0.35;
   elseif time <=31
       pwm1 = 0.35;
       pwm2 = 0.35;
       pwm3 = 0.35;
   elseif time <=35
       pwm1 = 0.35;
       pwm2 = 0.35;
       pwm3 = 0.35;
   elseif time<=38
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time<=41
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.33;
   elseif time<= 45
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <=48
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <= 52
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <= 55
       pwm1 = 0.001;
       pwm2 = 0.3;
       pwm3 = 0.001;
   elseif time<=59
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <=62
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.3;
   else 
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   end
end

function [pwm1,pwm2,pwm3] = Step_PWM(time)
    if time < 1
        pwm1 = 0.001;
        pwm2 = 0.001;
        pwm3 = 0.001;
    elseif time >= 1 && time < 3
         pwm1 = 0.15;
%         pwm2 = 0.1;
%         pwm3 = 0.1;
    elseif time >= 3 && time < 5
         pwm1 = 0.2;
%         pwm2 = 0.15;
%         pwm3 = 0.15;
    elseif time >= 5 && time < 7
         pwm1 = 0.2;
%         pwm2 = 0.2;
%         pwm3 = 0.2;
    elseif time >= 7 && time < 9
         pwm1 = 0.25;
%         pwm2 = 0.25;
%         pwm3 = 0.25;
    elseif time >= 9 && time < 11
         pwm1 = 0.3;
%         pwm2 = 0.3;
%         pwm3 = 0.3;
    elseif time >= 11 && time < 13
         pwm1 = 0.35;
%          pwm2 = 0.35;
%         pwm3 = 0.35;
    elseif time >= 13 && time < 15
         pwm1 = 0.4;
%         pwm2 = 0.4;
%         pwm3 = 0.4;
    elseif time >= 15 && time < 17
        pwm1 = 0.45;
%         pwm2 = 0.001;
%         pwm3 = 0.001;
    elseif time >= 17 && time < 19
        pwm1 = 0.5;
%         pwm2 = 0.001;
%         pwm3 = 0.001;
    elseif time >= 19 && time < 21
        pwm1 = 0.55;
%         pwm2 = 0.001;
%         pwm3 = 0.001;
    else
        pwm1 = 0.001;
        pwm2 = 0.001;
        pwm3 = 0.001;
    end
end
function [pwm1,pwm2,pwm3] = Sine_PWM(time)
       if time <= 0.5
           pwm1 = 0.001;
           pwm2 = 0.001;
           pwm3 = 0.001;
       elseif time < 8.5
           pwm1 = 0.001;
           pwm2 = 0.21+0.2*sin(pi*(time-0.5)/2-pi/2);
           pwm3 = 0.001;
       elseif time < 16.5 
           pwm1 = 0.201+0.2*sin(pi*(time-8.5)/2-pi/2);
           pwm2 = 0.201+0.2*sin(pi*(time-8.5)/2-pi/2);
           pwm3 = 0.201+0.2*sin(pi*(time-8.5)/2-pi/2);
       elseif time < 24.5 
           pwm1 = 0.201+0.2*sin(pi*(time-16.5)/2-pi/2);
           pwm2 = 0.001;
           pwm3 = 0.201+0.2*sin(pi*(time-16.5)/2-pi/2);        
       else
           pwm1 = 0.001;
           pwm2 = 0.001;
           pwm3 = 0.001;           
       end 

end
function [pwm1,pwm2,pwm3] = Constant_PWM(time)
    if time < 1
        pwm1 = 0.001;
        pwm2 = 0.001;
        pwm3 = 0.001;
    elseif time < 6
        pwm1 = 0.4;  %0.067=0.2bar
        pwm2 = 0.4;
        pwm3 = 0.4;
    elseif time < 12
        pwm1 = 0.4;  %0.067=0.2bar
        pwm2 = 0.4;
        pwm3 = 0.4;
     elseif time < 18
         pwm1 = 0.55;  %0.067=0.2bar
         pwm2 = 0.001;
         pwm3 = 0.001;
%     elseif time < 15
%         pwm1 = 0.001;  %0.067=0.2bar
%         pwm2 = 0.001;
%         pwm3 = 0.1;
%     elseif time < 20
%         pwm1 = 0.001;  %0.067=0.2bar
%         pwm2 = 0.001;
%         pwm3 = 0.1;
    else
        pwm1 = 0.001;
        pwm2 = 0.001;
        pwm3 = 0.001;        
    end
end
function [pwm1,pwm2,pwm3] = Workspace_PWM(time)
global pressure 
   if time < 1
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time < 65
       tn = floor(time);
       pwm1 = pressure(tn,1);
       pwm2 = pressure(tn,2);
       pwm3 = pressure(tn,3);
   else       
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   end  
end
function [pwm1,pwm2,pwm3] = Ramp3_PWM(time)
  if time <=2
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time<=7
%        pwm1 = 0.1*(time-2)+0.001;
%        pwm1 = 0.1*(time-2)/2.5 + 0.001;%
%        pwm2 = 0.1*(time-2)/2.5 + 0.001;
       pwm1 = 0.1*(time-2)/1.25+0.001;
       pwm2 = 0.1*(time-2)/1.25+0.001;
       pwm3 = 0.1*(time-2)/1.25+0.001;
   elseif time <=12
%        pwm1 = -0.1*(time-7)/2.5+0.5/2.5+0.001;
%        pwm2 = -0.1*(time-7)/2.5+0.5/2.5+0.001;
       pwm1 = -0.1*(time-7)/1.25+0.5/1.25+0.001;
       pwm2 = -0.1*(time-7)/1.25+0.5/1.25+0.001;
       pwm3 = -0.1*(time-7)/1.25+0.5/1.25+0.001;
   elseif time <=15
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time<=21
       pwm1 = 0.001;
%        pwm2 = 0.1*(time-15)/1.67+0.001;
%        pwm3 = 0.1*(time-15)/1.67+0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <=25
       pwm1 = 0.001;
%        pwm2 = -0.1*(time-20)/1.67+0.5/1.67+0.001;
       pwm2 = -0.1*(time-20)/1.25+0.5/1.25+0.001;
%        pwm3 = -0.1*(time-20)/1.67+0.5/1.67+0.001;
       pwm3 = 0.001;
   elseif time <=28
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   elseif time <=33
%        pwm1 = 0.1*(time-28)/1.25+0.001;
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.1*(time-28)/1.25+0.001;
   elseif time <=38
%        pwm1 = -0.1*(time-33)/1.25+0.5/1.25+0.001;
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = -0.1*(time-33)/1.25+0.5/1.25+0.001;
   else
       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.001;
   end
end
%% This function is used for tip position control 
function [pwm1,pwm2,pwm3] = PositionCtrl(p,R,v)
% Add your pwm generation function here
%--------------------------------------%


%--------------------------------------%
end
