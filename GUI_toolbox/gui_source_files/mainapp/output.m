function output
s = daq.createSession('ni');
%   part 3: output
% Log Data
figure(1)
% Convert the acquired data and timestamps to a timetable in a workspace variable.
ai0 = s.UserData.Data(:,1);
ai1 = s.UserData.Data(:,2);
ai2 = s.UserData.Data(:,3);
DAQ_pressure = timetable(seconds(s.UserData.TimeStamps),s.UserData.Data/3-0.08);
DAQ_cmd_pressure = timetable(seconds((s.UserData.Timeinterval)'),3*(s.UserData.PWMduty));
Aurora_position = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.Position);
Aurora_quat = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.quat);
Aurora_z_axis = s.UserData.RotationMatirx(:,3:3:size(s.UserData.RotationMatirx,2));
theta = acos(sum(Aurora_z_axis.*[0;0;1]))/pi*180;
Aurora_theta = timetable(seconds((s.UserData.Timeinterval)'),theta');
File_name = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'workspace_type1.mat');
% DAQ_pressure = timetable(seconds(s.UserData.TimeStamps),ai0);
% Plot Data
% Plot the acquired data on labeled axes.
plot(DAQ_pressure.Time, DAQ_pressure.Variables)
hold on 
plot(DAQ_cmd_pressure.Time, DAQ_cmd_pressure.Variables)
xlabel('Time')
ylabel('Bar')
save(File_name,'DAQ_cmd_pressure','DAQ_pressure','Aurora_position','Aurora_quat','Aurora_theta');
% legend(DAQ_2.Properties.VariableNames)

% plot position and rotation
figure(2)
subplot(2,1,1)
plot(s.UserData.Timeinterval,s.UserData.Position(:,1),...
     s.UserData.Timeinterval,s.UserData.Position(:,2),...
     s.UserData.Timeinterval,s.UserData.Position(:,3));
 legend('x','y','z');
% subplot(3,1,2)
% plot(s.UserData.Timeinterval,s.UserData.RotationAngle(:,1)/pi*180,...
%      s.UserData.Timeinterval,s.UserData.RotationAngle(:,2)/pi*180,...
%      s.UserData.Timeinterval,s.UserData.RotationAngle(:,3)/pi*180);
%  % plot theta
subplot(2,1,2)

% plot(s.UserData.Timeinterval,theta);

% view(0,90);
figure(3)
x = s.UserData.Position(:,1) - s.UserData.Position(1,1);
y = s.UserData.Position(:,2) - s.UserData.Position(1,2);
z = -0.0324 - s.UserData.Position(:,3);
plot3(x, y, z, '*')
xlabel('x')
ylabel('y')
zlabel('z')
view(90,0);
% subplot(3,1,1)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,1),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,1));
%  
% subplot(3,1,2)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,2),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,2));
%  
% subplot(3,1,3)
% plot(s.UserData.Timeinterval,s.UserData.Velocity1(:,3),...
%      s.UserData.Timeinterval,s.UserData.Velocity2(:,3));
 
% Clean Up
% Remove event listeners and clear the session and channels, if any.
delete(lh1)
delete(lh2)
clear aurora_device
% fclose(port);
% clear s lh1 lh2

% Callback Function
% Define the callback function for the 'DataAvailable' event.
% generating from matlab auto-generation
function recordData(src, eventData)
% tic
global aurora_device pressure
% RECORDDATA(SRC, EVENTDATA) records the acquired data, timestamps and
% trigger time. You can also use this function for plotting the
% acquired data live.

% SRC       - Source object      i.e. Session object
% EVENTDATA - Event data object  i.e. 'DataAvailable' event data object

% Record the data and timestamps to the UserData property of the session.
src.UserData.Data = [src.UserData.Data; eventData.Data];
src.UserData.TimeStamps = [src.UserData.TimeStamps; eventData.TimeStamps];
src.UserData.Timeinterval  = [src.UserData.Timeinterval,eventData.TimeStamps(1)];

% Derive the Aurora position And Orientation information     
aurora_device.updateSensorDataPositionAndRotation();
% get the position in three directions, unit is m
temP =  aurora_device.port_handles.trans/1000;
% get the rotation angle around x,y,z
temrot = aurora_device.port_handles.rot;
src.UserData.quat = [src.UserData.quat;temrot];
src.UserData.RotationMatirx = [src.UserData.RotationMatirx,quat2rotm(temrot)];
temR = quat2eul(temrot);
temRx = temR(1);
temRy = temR(2);
temRz = temR(3);
%[R,~] =  aurora_device.measureTipOrientation();
src.UserData.Position = [src.UserData.Position;temP];
src.UserData.RotationAngle = [src.UserData.RotationAngle;[temRx,temRy,temRz]];

% velocity estimation function
% or add you own velocity estiomation function here
% <temP> is current step position and the time period is <ControlPeriod> 
[temVx1,temVy1,temVz1] = getVelocitySimplified(temP);
src.UserData.Velocity1 = [src.UserData.Velocity1; [temVx1,temVy1,temVz1]];

[temVx2,temVy2,temVz2] = getVelocity5Points(temP);
src.UserData.Velocity2 = [src.UserData.Velocity2; [temVx2,temVy2,temVz2]];

% Add any self-defined PWM generation function here
%-------------------------------------------------------%
%change PWM duty in a user-defined way
[pwmtemp1,pwmtemp2,pwmtemp3] = Workspace_PWM(eventData.TimeStamps(1));
src.Channels(4).DutyCycle = pwmtemp1;
src.Channels(5).DutyCycle = pwmtemp2;
src.Channels(6).DutyCycle = pwmtemp3;
%------------------------------------------------------%

% Record PWM duty to the UserData property of the session.
src.UserData.PWMduty = [src.UserData.PWMduty;...
                       [pwmtemp1,pwmtemp2,pwmtemp3]];
% Record the starttime from the first execution of this callback function.

if isempty(src.UserData.StartTime)
    src.UserData.StartTime = eventData.TriggerTime;
end
%  toc
% Uncomment the following lines to enable live plotting.
% for example:
% figure(8)
% plot(eventData.TimeStamps, eventData.Data)
% xlabel('Time (s)')
% ylabel('Amplitude (V)')
% legend('ai13')
end
end