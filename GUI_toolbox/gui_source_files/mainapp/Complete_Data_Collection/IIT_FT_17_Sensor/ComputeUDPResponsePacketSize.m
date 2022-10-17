function PacketSize=ComputeUDPResponsePacketSize(Command,SensorObject)

% This function counts the bytes to expect in the udp received packet

if Command=='GET_SINGLE_UDP_PACKET'
        SensorObject.UDPPolicy=bitshift(SensorObject.Policy1,8,'uint16')+SensorObject.Policy0;

        PcCnt=4;%initialise packet counter

	if bitand(SensorObject.UDPPolicy,1,'uint16')>0 %check if bit 0 of the BCAST_POLICY lower byte is asserted    
        for C = 1:6      
		PcCnt=PcCnt+2;
        end
    end
	if bitand(SensorObject.UDPPolicy,2,'uint16')>0%//check if bit 1 of the BCAST_POLICY lower byte is asserted
        for C = 1:6  
		PcCnt=PcCnt+4;
        end		
    end
	if bitand(SensorObject.UDPPolicy,4,'uint16')>0%//check if bit 2 of the BCAST_POLICY lower byte is asserted		
		 for C = 1:6    		 
%             //This is a 2 byte unsigned number of the AD converter (value range 0-65535)		
		PcCnt=PcCnt+2;
         end
    end		
	if bitand(SensorObject.UDPPolicy,8,'uint16')>0%//check if bit 3 of the BCAST_POLICY lower byte is asserted
		PcCnt=PcCnt+4;
    end
	if bitand(SensorObject.UDPPolicy,16,'uint16')>0%//check if bit 4 of the BCAST_POLICY lower byte is asserted
		PcCnt=PcCnt+4;
    end
	if bitand(SensorObject.UDPPolicy,32,'uint16')>0%//check if bit 5 of the BCAST_POLICY lower byte is asserted
		PcCnt=PcCnt+2;
    end
	if bitand(SensorObject.UDPPolicy,64,'uint16')>0%//check if bit 6 of the BCAST_POLICY lower byte is asserted		
		 for C = 1:6    
		PcCnt=PcCnt+4;				
         end	
    end
    if bitand(SensorObject.UDPPolicy,128,'uint16')>0%//check if bit 6 of the BCAST_POLICY lower byte is asserted		
         for C = 1:6    
                PcCnt=PcCnt+4;					
         end
    end
    PcCnt=PcCnt+1;% addone byte to the counter for the chechsum
end

    PacketSize=PcCnt;