function [y,n]=sin_wave(t,a,f,d,st)
global ControlPeriod
%y for pressure input
%a for amplitude
%f for frequency
%d for displacement
%t for time

%n for x-axis output
n=0:ControlPeriod:t-ControlPeriod;
y=a*sin(2*pi/f.*n)+d;
s=zeros([1,st/ControlPeriod]);
y=[s,y(1:(t-st)/ControlPeriod)];
y(1:st/ControlPeriod)=0.003;
n=n';
y=y';
%y for y-aixs output

end