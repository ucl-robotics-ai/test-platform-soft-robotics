function [vx,vy,vz] = getVelocitySimplified(p)
global PositionLastStep ControlPeriod

CurrentPosition = p;
%  calculate the velocity
vx = (CurrentPosition(1)- PositionLastStep(1))/ControlPeriod; 
vy = (CurrentPosition(2)- PositionLastStep(2))/ControlPeriod;  
vz = (CurrentPosition(3)- PositionLastStep(3))/ControlPeriod;  

% update last step position informaiton
PositionLastStep = CurrentPosition;
end
