%% method 2
% f(x(n))'=4/3(f(x(n+1))-f(x(n-1)))/(2t)-1/3(f(x(n+2)-f(x(n-2)))/(4t)
% This will introduce 2*control periods delay
% for instance, if the position updates 25ms, the speed estimation 
% will delay for about 50ms.
function [vx,vy,vz] = getVelocity5Points(p)
global PositionArray ControlPeriod
%  update the position array
    for i = 5:-1:2
        PositionArray(i,:) = PositionArray(i-1,:);
    end
    PositionArray(1,:) = p;
    
%  calculate the velocity
vx = (4/3)*(PositionArray(2,1)-PositionArray(4,1))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,1)-PositionArray(5,1))/(4*ControlPeriod);
vy = (4/3)*(PositionArray(2,2)-PositionArray(4,2))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,2)-PositionArray(5,2))/(4*ControlPeriod);
vz = (4/3)*(PositionArray(2,3)-PositionArray(4,3))/(2*ControlPeriod)-...
     (1/3)*(PositionArray(1,3)-PositionArray(5,3))/(4*ControlPeriod); 
end