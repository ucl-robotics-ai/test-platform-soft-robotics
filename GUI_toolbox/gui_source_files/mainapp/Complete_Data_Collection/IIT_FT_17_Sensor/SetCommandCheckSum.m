function CommandOut=SetCommandCheckSum(CommandIn)
% This Function receives the Command packet as an array of numbers,
% it calculates the Checksum and inserts it into the last byte of the
% Command array.

chksum=int16(256); %Initialise the checksum variable

%calculate the checksum of all the command bytes except the last
for i=1:length(CommandIn)-1
  chksum=chksum-CommandIn(i); 
end

 if chksum<0 
     chksum=256+chksum;
 end
 
%  Insert the checksum value into the last byte of the command array
CommandIn(length(CommandIn))=chksum;

% Return the Command
CommandOut=CommandIn;