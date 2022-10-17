function [y,n]=constant_wave(t,~,~,d,st)
global ControlPeriod 
%y for pressure input
%d for displacement (or magnitude here)
%t for time

%n for x-axis output
n=0:ControlPeriod:t-ControlPeriod;
%y for y-aixs output
y=zeros([1,t/ControlPeriod]);
y(:)=d;
s=zeros([1,st/ControlPeriod]);
y=[s,y(1:(t-st)/ControlPeriod)];
y(1:st/ControlPeriod)=0.003;
n=n';
y=y';
end
