function n=SendUDPCommand(CommandString, SensorObject)
% This Function receives as argument a string of the UDP command to be sent
% to a sensor and a reference to the sensor structure.
%It calls the checksum calculation function SetCommandCheckSum.m  to
%complete the packet with the correct CHCKSUM field and then sends the
%packet through UDP to the associated UDP connection
%"SensorObject.UDPHandle"


%SET_SINGLE_UDP_PACKET_POLICY	FF	3	3	n	Array0	Array1	CHKSUM
% SET_SINGLE_UDP_PACKET_POLICY=[255;3;3;BoardNumber;Policy0;Policy1;CHKSUM];
% GET_SINGLE_UDP_PACKET	FF	1	4	n	CHKSUM	
% GET_SINGLE_UDP_PACKET=[255;1;4;BoardNumber;CHKSUM];
%UDP_CALIBRATE_OFFSETS	FF	1	5	n	CHKSUM	
% UDP_CALIBRATE_OFFSETS=[255;1;5;BoardNumber;CHKSUM];


switch CommandString
  case 'SET_SINGLE_UDP_PACKET_POLICY'
      command=SetCommandCheckSum([255;3;3;SensorObject.BoardNumber;SensorObject.Policy0;SensorObject.Policy1;0]);
%       display(command);
  case 'GET_SINGLE_UDP_PACKET'
      command=SetCommandCheckSum([255;1;4;SensorObject.BoardNumber;0]);
%       display(command);
  case 'UDP_CALIBRATE_OFFSETS'
      command=SetCommandCheckSum([255;1;5;SensorObject.BoardNumber;0]);
%       display(command);
  otherwise
    display('SendUDPCommand Error: SendUDPCommand received a Command  not recognised');
end
        fwrite(SensorObject.UDPHandle,command,'uint8');  %UDP command necessary to make the the stream visible;

%  SET_SINGLE_UDP_PACKET_POLICY=SetCommandCheckSum(SET_SINGLE_UDP_PACKET_POLICY)
% fwrite(s,check_Protocol);  %UDP command necessary to make the the stream visible
% fwrite(s,UDPPacketToSend,'uint8');  %UDP command necessary to make the the stream visible;

        