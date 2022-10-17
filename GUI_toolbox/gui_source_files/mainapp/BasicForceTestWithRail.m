
%% BASIC INFORMATION %%%

% Zaber X-LSM100A
% IIT FT-17 
clc
clear all
close all
data_n = 5;


%% CLEAR OUT PREVIOUS CODE %%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end


%% LINEAR RAIL SETUP %%%

%You might need to use INSTRFIND or something else to double check which
%port you have connected it to

%Adds the folder with the UDP code to the current search path
addpath('IIT_FT_17_Sensor','-end');

%Declare the COMMS port for linear rail 
port = serial('COM5');

%Set up the default ASCII protocol for linear rail
set(port, ...
    'BaudRate', 115200, ...
    'DataBits', 8, ...
    'FlowControl', 'none', ...
    'Parity', 'none', ...
    'StopBits', 1);

%Stop Zaber delay
set(port, 'Timeout', 0.5)
warning off MATLAB:serial:fread:unsuccessfulRead

time_span = [];
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
    
   
%Initialise force and torque variables.
Fx = double(0);
Fy = double(0);
Fz = double(0);
Tx = double(0);
Ty = double(0);
Tz = double(0);

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

%% SETUP CONNECTION TO LINEAR RAIL AND START %%%

%Open up a Serial connection at the USB port to talk to linear rail
fopen(port);
%Declare the COMMS protocol for the linear rail.
protocol = Zaber.AsciiProtocol(port);

try

%For linear rail, 1 microstep = 0.000047625mm.
steptomm = 0.000047625;

% to move the rail a fixed amount then must specify in rail microsteps:
% device.moverelative(amount_mm/steptomm);
% device.waitforidle();

% the length that the manipulator is deformed from 0mm at touching
testdistance = 15; % [mm]
linear_rail_speed = 0.25; % [mm/s]
% the threshold force at which the sensor cannot exceed in case it breaks
fthreshold = 0.1; % 
% supposedly maximum velocity in steps
absmaxvel = 206408;
    
device = Zaber.AsciiDevice.initialize(protocol, 1);

% resets maximum speed
device.set('maxspeed',absmaxvel);
device.waitforidle();

%Save maximum range of the linear rail.
maxrange = device.getrange();

%Convert units for speeds and distance
linear_rail_speed_steps = device.Units.velocitytonative(linear_rail_speed/1000);
testdistance_steps = device.Units.positiontonative(testdistance/1000);
max_range_manual = device.Units.positiontonative(0.12);

% repeats = 5;

%Mandatory initialisation homing sequence.
device.home();
device.waitforidle();

    
%% MOVE FORWARD UNTIL TOUCHING SOMETHING TO DETERMINE START POSITION %%

%Set speed and force threshold to stop on touch.

device.moveatvelocity(absmaxvel); 

while true
    
    FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));
    
    Fx = FT_Sensor(1).Data.ft(1);
    Fy = FT_Sensor(1).Data.ft(2);
    Fz = -FT_Sensor(1).Data.ft(3);
    F = [Fx;Fy;Fz];
        
    Tx = FT_Sensor(1).Data.ft(4);
    Ty = FT_Sensor(1).Data.ft(5);
    Tz = FT_Sensor(1).Data.ft(6);
    T = [Tx;Ty;Tz];
    
    % if the force exceeds the threshold, end the movement
    if abs(Fz) > fthreshold
        
        device.stop();
        stoppos = device.getposition();            
        testfail1 = 0;
        break
%         searching_m = 0;
    
    elseif device.getposition() == max_range_manual
              
        device.waitforidle();
        device.set('maxspeed',absmaxvel);
        device.home();
        device.waitforidle();
        testfail1 = 1;
        break
        
    end
    
end
        
%If encountered an object, proceed with measuring sequence in the forward direction. If not, abort.

%Move back so not touching and deforming to start
preload = 1.5; % [mm] Can change from 0.3mm offset
device.moverelative(-(preload/steptomm));
device.waitforidle();

pause(3.0)
datetime('now')

starting_position = device.getposition();

%% START FORWARDS MOTION TESTING %%%

i = 0;
Overload = 0;

% device.moveabsolute(testdistance_steps);
device.moveatvelocity(linear_rail_speed_steps);

moving_forwards = 1;
while 1
    
    FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));
    PositionSteps = device.getposition();
    Pos = (PositionSteps-stoppos)*0.000047625+preload %Convert position in steps to mm
    
    Fx = FT_Sensor(1).Data.ft(1);
    Fy = FT_Sensor(1).Data.ft(2);
    Fz = -FT_Sensor(1).Data.ft(3); 
    F = [Fx;Fy;Fz];
        
    Tx = FT_Sensor(1).Data.ft(4);
    Ty = FT_Sensor(1).Data.ft(5);
    Tz = FT_Sensor(1).Data.ft(6);
    T = [Tx;Ty;Tz];
    
    if abs(Fx) > 50 || abs(Fy) > 50 || abs(Fz) > 25 || abs(Tx) > 0.5 || abs(Ty) > 0.5 || abs(Tz) > 0.5
        
        device.stop();                    
        Overload = 1;
        break   
    end
    
    i = i+1;
    ForceArrayf(i,:) = F;
    TorqueArrayf(i,:) = T;
    PositionArrayf(i,1) = Pos; 
    
    if Pos > testdistance
        device.stop();
        datetime('now')
        break
%         moving_forwards = 0;
    end
    
end


pause(3);

%% BACKWARDS MOTION TESTING %%%

i = 0;
Overload = 0;

device.moveatvelocity(-linear_rail_speed_steps);

moving_backwards = 1;
while 1
    
    FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));
    PositionSteps = device.getposition();
    Pos = (PositionSteps-stoppos)*0.000047625+preload %Convert position in steps to mm
    
    Fx = FT_Sensor(1).Data.ft(1);
    Fy = FT_Sensor(1).Data.ft(2);
    Fz = -FT_Sensor(1).Data.ft(3); 
    F = [Fx;Fy;Fz];
        
    Tx = FT_Sensor(1).Data.ft(4);
    Ty = FT_Sensor(1).Data.ft(5);
    Tz = FT_Sensor(1).Data.ft(6);
    T = [Tx;Ty;Tz];
    
    if abs(Fx) > 50 || abs(Fy) > 50 || abs(Fz) > 25 || abs(Tx) > 0.5 || abs(Ty) > 0.5 || abs(Tz) > 0.5
        
        device.stop();                    
        Overload = 1;
        break   
    end
    
    i = i+1;
    ForceArrayb(i,:) = F;
    TorqueArrayb(i,:) = T;
    PositionArrayb(i,1) = Pos; 
    
    if Pos <= 0
        device.stop();
%         moving_backwards = 0;
        break
    end
    
end

%% CLOSE AND RESET %%%

catch exception
    fclose(port);
    rethrow(exception);
    
end

%Test finished successfully, return to home position.
device.home();
device.waitforidle();
    
%Disconnect, delete and clear ports; otherwise the port remains hanging and
%prevents connecting subsequently.

fclose(port);
delete(port);
clear port;

%% SAVE DATA %%%

name = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'_ForceTest_long_vac.mat');
% namefig = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'_ForceTest_long_vac.fig');

pf = plot(PositionArrayf(1:end),abs(ForceArrayf(:,end)));
hold on
pb = plot(PositionArrayb(1:end),abs(ForceArrayb(:,end)));
legend('forward','backward')
xlabel('identation (mm)')
ylabel('force (N)')
% saveas(pf,namefig);
save (name,'PositionArrayf','ForceArrayf','PositionArrayb','ForceArrayb');



