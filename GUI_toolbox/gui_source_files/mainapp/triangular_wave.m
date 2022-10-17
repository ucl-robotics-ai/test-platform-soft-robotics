function [y,n]=triangular_wave(t,a,f,d,st)
global ControlPeriod 
%y for pressure input
%a for amplitude
%f for frequency
%d for displacement
%t for time

%n for x-axis output
n=0:ControlPeriod:t-ControlPeriod;
y=0;
m=1;
%fourier transform for triangular wave
for m=1:1:101
    %y for y-aixs output
    
    y=y+(-1)^(m-1)*sin((2*m-1)*2*pi/f.*n)/((2*m-1)^2);
    m=m+1;
end
y=8*a/(pi^2)*y+d;
s=zeros([1,st/ControlPeriod]);
y=[s,y(1:(t-st)/ControlPeriod)];
y(1:st/ControlPeriod)=0.003;
n=n';
y=y';
end
 