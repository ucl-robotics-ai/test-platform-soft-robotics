function SensorObject=GetFTsensorData(SensorObject)
%This function requests, receives and returns to the caller data from a
%sensor. It takes as argument a structure of the sensor from which to
%receive data.  
%1. it sends a request for a single data packet to the sensor
%2. it tries to receive a response to the request of the expected amount of data
%according to the selected Policy fields
%3. it calculates the checksum of the received data and if the checksum is
%correct it calls the parsing function which fills the sensor's structure members

SendUDPcommand('GET_SINGLE_UDP_PACKET',SensorObject);

PacketSize=ComputeUDPResponsePacketSize('GET_SINGLE_UDP_PACKET',SensorObject);

SensorObject.UDPRecvBuff = fread(SensorObject.UDPHandle,PacketSize,'uint8');%Read response

chcksumOk=ReceivedUDPPacketCHECKSUM(SensorObject.UDPRecvBuff);

if chcksumOk==1
	SensorObject=ParseUDPPacket(SensorObject);	
    flushinput(SensorObject.UDPHandle);
end
