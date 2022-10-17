function [y,n]=step_wave(t,a,f,d,st)
global ControlPeriod 
y=zeros([1,(t-st)/ControlPeriod]);
m=0;
n=0:ControlPeriod:t-ControlPeriod;
for m=0:f
   % y(1+m*(t-st)/(2*(f+1))/ControlPeriod:1+(m+1)*(t-st)/(2*(f+1))/ControlPeriod)=roundn(m*a/f+d,-2);
    %y((2*(f+1)-1-m)*(t-st)/(2*(f+1))/ControlPeriod:(2*(f+1)-m)*(t-st)/(2*(f+1))/ControlPeriod)=roundn(m*a/f+d,-2);
    y(round(1+m*(t-st)/(2*(f+1))/ControlPeriod):round(1+(m+1)*(t-st)/(2*(f+1))/ControlPeriod))=m*a/f+d;
    y(round((2*(f+1)-1-m)*(t-st)/(2*(f+1))/ControlPeriod):round((2*(f+1)-m)*(t-st)/(2*(f+1))/ControlPeriod))=m*a/f+d;
    m=m+1;
end
s=zeros([1,st/ControlPeriod]);
y=[s,y];
y(1:st/ControlPeriod)=0.003;
n=n';
y=y';
end