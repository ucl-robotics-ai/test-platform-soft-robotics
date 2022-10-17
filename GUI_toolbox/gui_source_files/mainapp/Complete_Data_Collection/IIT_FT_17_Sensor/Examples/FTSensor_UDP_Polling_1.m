%This is a polling example of FT sensor data acquisition with a 3D plot. 
%This example does not use any external software apart form the following m
%files that should reside in the same folder:
% ComputeUDPResponsePacketSize.m
% GetFTsensorData.m
% ParseUDPPacket.m
% ReceivedUDPPacketCHECKSUM.m
% SendUDPcommand.m
% SetCommandCheckSum.m
%The FTSensor_UDP_Polling_1.m example implements the UDP connection and all
% data exchange and parsing using Matlab's built-in communications functionality.

%The purpose of this example is only to demonstrate the necessary steps in order to
%configure a connection and to exchange commands and data between Matlab
%and a sensor.  
% This example is by no means intended as an example of efficient UDP
%data exchange between a Matlab and a sensor. Ideally this should be handled by
%external C code e.g. a Dll library that should be called within Matlab to set-up
%communications and perform data exchange and parsing.

%##########################################################################
%##########################################################################
%##########################################################################
%UDP Commands

%SET_SINGLE_UDP_PACKET_POLICY	FF	3	3	n	Array0	Array1	CHKSUM
%SET_SINGLE_UDP_PACKET_POLICY=[255;3;3;BoardNumber;policy0;policy1;CHKSUM];
%GET_SINGLE_UDP_PACKET	FF	1	4	n	CHKSUM	
%GET_SINGLE_UDP_PACKET=[255;1;4;BoardNumber;CHKSUM];
%UDP_CALIBRATE_OFFSETS	FF	1	5	n	CHKSUM	
%UDP_CALIBRATE_OFFSETS=[255;1;5;BoardNumber;CHKSUM];
%--------------------------------
%UDP Replies
%BCAST_DATA_PACKET_MT*	FD	n+2	BC	Board ID	data0	...	datan	CHKSUM


%##########################################################################
%##########################################################################
%##########################################################################

% Transmit data to the server (or a request for data from the server). 
%fprintf(t, 'GET /');
close all
clear all

inputBuffer=100;% This is the UDP buffer size

%  This is the structure of the sensor data
FT_SensorData=struct(...
        'ChRaw_Offs',int16(zeros(6,1)),...
        'FT',int32(zeros(6,1)),...
        'ChRaw',uint16(zeros(6,1)),...
        'temp_Vdc',int32(0),...
        'tStamp',int32(0),...
        'fault',int16(0),...
        'filt_FT',int32(zeros(6,1)),...
        'ft',double(zeros(6,1)),...
        'filt_ft',double(zeros(6,1)));
        
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
    FT_Sensor(1).Policy0=127;
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

figure(1);
axis equal
hold on
%a Handle to ploted resultant force
h1=plot3(Fx,Fy,Fz);
hold on
%A Handle to ploted component forces
h2=plot3(Fx,Fy,Fz);
hold on
%A handle to ploted torques
h3=plot3(Fx,Fy,Fz);
hold on
%Set Viewpoint
azimuth=45;
elevation=15;
view(azimuth, elevation);
grid on

% ****************************Setup UDP********************
echoudp('off')% first disable the Echo of the UDP
fclose('all')%close opened files and connections

echoudp('on',4012);

FT_Sensor(1).UDPHandle=udp('192.168.1.1',23)% IP Address and port of the sensor 
set(FT_Sensor(1).UDPHandle,'DatagramTerminateMode', 'off')
FT_Sensor(1).UDPHandle.Timeout =0.1
FT_Sensor(1).UDPHandle.InputBufferSize=inputBuffer
fopen(FT_Sensor(1).UDPHandle)
% *********************************************************

SendUDPcommand('SET_SINGLE_UDP_PACKET_POLICY',FT_Sensor(1));

SendUDPcommand('GET_SINGLE_UDP_PACKET',FT_Sensor(1));
 
SendUDPcommand('UDP_CALIBRATE_OFFSETS',FT_Sensor(1));


delay=0;

while 1
tic;
tin=tic;

%Get the data from the sensor and fill the structure.
FT_Sensor(1)= GetFTsensorData(FT_Sensor(1));

        Fx=FT_Sensor(1).Data.ft(1);
        Fy=FT_Sensor(1).Data.ft(2);
        Fz=-FT_Sensor(1).Data.ft(3); 
        F=[Fx;Fy;Fz];
%         display(F'); %comment this for not printing incommand window

        Tx=FT_Sensor(1).Data.ft(4);
        Ty=FT_Sensor(1).Data.ft(5);
        Tz=FT_Sensor(1).Data.ft(6);
        T=[Tx;Ty;Tz];
%         display(T');%comment this for not printing incommand window
        
%------------------------Update 3D Plot ---------------------------------
        delay=delay+1;

if  delay>2%toc>0.05
            
        delay=0;
        %Create Force and Torque vectors for visualisation
        FX=Fx.*[0 1 0 0 0 0 0];
        FY=Fy.*[0 0 0 1 0 0 0];
        FZ=Fz.*[0 0 0 0 0 1 0];
        axisX=[0 -5 0 5 0 ];

        TX=10*Tx.*[0 1 0 0 0 0];
        TY=10*Ty.*[0 0 0 1 0 0];
        TZ=10*Tz.*[0 0 0 0 0 1];

        %Plot the measured resultant force vector
        set(h1,'Xdata',[0 Fx],'Ydata',[0 Fy],'Zdata',[0 Fz],'Marker','>','color','b','LineStyle','-'); 
        %Plot the measured component force vectors
        set(h2,'Xdata',[FX],'Ydata',[FY],'Zdata',[FZ],'Marker','o','color','r','LineStyle','-');
        %Plot the measured torque vectors
        set(h3,'Xdata',[TX],'Ydata',[TY],'Zdata',[TZ],'Marker','o','color','g','LineStyle','-');
        drawnow;

        %compute auto scaling of plot axis
        absFmax=max(max(F'), -min(F'));
        Tmax=max(T');
        range=absFmax+1;
        axis([-range range -range range -range range]);    
        legend on;
        
        Fz
        
        end
%------------------------End Update 3D Plot ---------------------------------

%     disp(toc); %uncomment for printing loop time

end %While end


% Disconnect and clean up the server connection. 
echoudp('off');
fclose(s);
delete(s); 
clear s 

fclose(t); 
delete(t); 
clear t 