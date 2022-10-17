function chcksumOk=ReceivedUDPPacketCHECKSUM(M)
% This function receives as input the UDP received packet and performs a
% Checksum validation. 
% If the calculated checksum of the received payload matches the checksum
% transmitted in the payload it returns 1 otherwise it returns -1

 buf=length(M);

K=uint8(zeros(100,1));

chcksumOk=0;        
 for i=1:buf  %ToDo: Put condition to exit this for when One streaming command has been detected and read ( e.g.brake after checksum)
chksum=256;



if i<=buf-2   
        if  M (i)==253 && M(i+2)==188 %Detect start of UDP BCAST_DATA_PACKET_MT
            commlength=M (i+1)+4;
           if commlength<=buf
           for r=1:commlength
                   if (i+r-1)<=buf  %ToDo: Whatch out for the end of buffer character!!!
                          K(r)=M(i+r-1);
                         if K(r)<0 K(r)=256+K(r);
                   end
                   %end
                   else
                          %if (i+r)>inputBuffer
                          K(r)=M(i+r-buf);
                          if K(r)<0 K(r)=256+K(r);
                          end
                   end
               
               %perform checksum
                  if r<commlength
                    chksum=chksum-int32(K(r));
                    if chksum<0 chksum=256+chksum;
                    end
                  else
                          if r==commlength && chksum==K(r) 
                             % disp('checksum is Valid')
                              chcksumOk=1;
                              break
                          else
                              chcksumOk=-1;
                              break
                          end
                  end
           end 
           end
        end
else
       if  M(i)==253 && M(buf-i+2)==188%Detect start of UDP BCAST_DATA_PACKET_MT
                commlength=M (i+1)+4;
         if commlength<=buf
           for r=1:commlength
                   if (i+r-1)<=inputBuffer  %ToDo: Whatch out for the end of buffer character!!!
                          K(r)=M(i+r-1);
                         if K(r)<0 K(r)=256+K(r);
                   end
                   %end
                   else
                          %if (i+r)>inputBuffer
                          K(r)=M(i+r-inputBuffer);
                          if K(r)<0 K(r)=256+K(r);
                          end
                   end
               
               %perform checksum
                  if r<commlength
                    chksum=chksum-int32(K(r));
                    if chksum<0 chksum=256+chksum;
                    end
                  else
                          if r==commlength && chksum==K(r) 
                            %  disp('checksum is Valid')
                              chcksumOk=1;
                              break
                          else
                              chcksumOk=-1;
                              break
                          end
                  end
           end       
           end
       end

end


if chcksumOk<0 % get out of the packet scaning for once a data stream command has been read.
    break
end

 end