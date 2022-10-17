function [SensorObject]=ParseUDPPacket(SensorObject)	
% debug=1;
debug=0;

if SensorObject.UDPRecvBuff(3) == 188   %Check if this is a FT sensor boradcast packet

    SensorObject.BoardNumber = SensorObject.UDPRecvBuff(4);        
    SensorObject.UDPPolicy=bitshift(SensorObject.Policy1,8,'uint16')+SensorObject.Policy0;
    PcCnt=4;%initialise packet counter

	if bitand(SensorObject.UDPPolicy,1,'uint16')>0 %check if bit 0 of the BCAST_POLICY lower byte is asserted
         for k = 1:6           		
            if bitand(SensorObject.UDPRecvBuff(PcCnt+2),128)>0   %Conversion to signed numbers 
                SensorObject.Data.ChRaw_Offs(k)=(int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'uint16'))+int32(SensorObject.UDPRecvBuff(PcCnt+1)))-65535;
            else
                SensorObject.Data.ChRaw_Offs(k)=(int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'uint16'))+int32(SensorObject.UDPRecvBuff(PcCnt+1)));
            end
            PcCnt=PcCnt+2;
         end
        if debug==1;  display(SensorObject.Data.ChRaw_Offs'); end
    end
    
	if bitand(SensorObject.UDPPolicy,2,'uint16')>0%//check if bit 1 of the BCAST_POLICY lower byte is asserted
        for k = 1:6  
            SensorObject.Data.FT(k) = int32(SensorObject.UDPRecvBuff(PcCnt+1)+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'int32'))...
                + int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+3),16,'int32'))+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+4),24,'int32')));
            PcCnt=PcCnt+4;
            SensorObject.Data.ft(k)=double(SensorObject.Data.FT(k))/1000000;
        end
        if debug==1;  display(SensorObject.Data.ft'); end      
    end
    
    if bitand(SensorObject.UDPPolicy,4,'uint16')>0%//check if bit 2 of the BCAST_POLICY lower byte is asserted    
		 for k = 1:6    		 
%             //This is a 2 byte unsigned number of the AD converter (value range 0-65535)		
                SensorObject.Data.ChRaw(k)=(int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'uint16'))+int32(SensorObject.UDPRecvBuff(PcCnt+1)));                   
            PcCnt=PcCnt+2;
         end
         if debug==1;  display(SensorObject.Data.ChRaw'); end   
    end		
    
	if bitand(SensorObject.UDPPolicy,8,'uint16')>0%//check if bit 3 of the BCAST_POLICY lower byte is asserted	
        SensorObject.Data.temp_Vdc = int32(SensorObject.UDPRecvBuff(PcCnt+1)+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'int32'))...
            + int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+3),16,'int32'))+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+4),24,'int32')));          
		PcCnt=PcCnt+4;
       if debug==1;  display(SensorObject.Data.temp_Vdc'); end  
    end
    
	if bitand(SensorObject.UDPPolicy,16,'uint16')>0%//check if bit 4 of the BCAST_POLICY lower byte is asserted        
        SensorObject.Data.tStamp = int32(SensorObject.UDPRecvBuff(PcCnt+1)+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'int32'))...
        + int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+3),16,'int32'))+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+4),24,'int32')));		
        PcCnt=PcCnt+4;
        if debug==1;  display(SensorObject.Data.tStamp); end 
    end
    
	if bitand(SensorObject.UDPPolicy,32,'uint16')>0%//check if bit 5 of the BCAST_POLICY lower byte is asserted		
         SensorObject.Data.fault=(int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'uint16'))+int32(SensorObject.UDPRecvBuff(PcCnt+1)));
		PcCnt=PcCnt+2;
        if debug==1;  display(SensorObject.Data.fault); end 
    end
    
	if bitand(SensorObject.UDPPolicy,64,'uint16')>0%//check if bit 6 of the BCAST_POLICY lower byte is asserted		
		 for k = 1:6    
            SensorObject.Data.filt_FT(k) = int32(SensorObject.UDPRecvBuff(PcCnt+1)+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+2),8,'int32'))...
                + int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+3),16,'int32'))+ int32(bitshift(SensorObject.UDPRecvBuff(PcCnt+4),24,'int32')));
		PcCnt=PcCnt+4;
        SensorObject.Data.filt_ft(k)=double(SensorObject.Data.filt_FT(k))/1000000;					
         end	
         if debug==1;  display(SensorObject.Data.filt_ft'); end 
    end
    if bitand(SensorObject.UDPPolicy,128,'uint16')>0%//check if bit 7 of the BCAST_POLICY lower byte is asserted		
            display("Policy Definition Error: Policy Bit 7 has been asserted by USER. Bit 7 is RESERVED Do Not Assert");
    end
end
     