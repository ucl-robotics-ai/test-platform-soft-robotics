function send_input_pressure
%% aurora system initialization
global PositionArray ControlPeriod PositionLastStep pressure time pwmtemp1 pwmtemp2 pwmtemp3 pwm_counter
load pressure_array.mat
% saving 5-step position to estimate velocity
%pwm_counter = 1;
PositionArray = zeros(5,3);
%ControlPeriod = 0.1;  % defalut as 25ms
PositionLastStep = zeros(3,1);  % original position
pwm_counter=1;
% Generate DAQ session
s = daq.createSession('ni');

%% Session Properties
% Set properties that are not using default values.
% Continuous recording and generating
s.IsContinuous = true;
% Scanning rate, Set Racording rate number of points/second.
% If want to capture the output PWM, set this value equals to
% 10*PWM_frequency
% set too high will introduce delay
s.Rate = 500;
% auto ratio is 0.1,means every 100ms, 'DataAvailable' will be checked
% So call function period is 0.1s
% disable auto setting.
s.IsNotifyWhenDataAvailableExceedsAuto = 0;
% Set checking period at 10ms
% Waring: On this platform, notifications more frequent than 20 times
%per second may not be achievable
s.NotifyWhenDataAvailableExceeds = s.Rate*ControlPeriod;
% Add Channels to Session
% Add channels and set channel properties, if any.
% Add Analog Input Channels
% check PWM
% ach1 = addAnalogInputChannel(s,'Dev1','ai4','Voltage');
% ach2 = addAnalogInputChannel(s,'Dev1','ai5','Voltage');
% ach3 = addAnalogInputChannel(s,'Dev1','ai13','Voltage');

% Feedback channels
ach1 = addAnalogInputChannel(s,'Dev1','ai0','Voltage');
ach2 = addAnalogInputChannel(s,'Dev1','ai1','Voltage');
ach3 = addAnalogInputChannel(s,'Dev1','ai2','Voltage');

% Set voltage type as SingleEnd
ach1.TerminalConfig = 'SingleEnded';
ach2.TerminalConfig = 'SingleEnded';
ach3.TerminalConfig = 'SingleEnded';

% Add PWM Generation Channels
% |PulseGeneration| measurement type. An analog input channel is added to
% monitor the pulse generated by the counter output channel. 
% 0 = 'ctr0' = PFI12 = P2.4; 1 = 'ctr1' = PFI13 = P2.5
% 2 = 'ctr2' = PFI14 = P2.6; 3 = 'ctr3' = PFI15 = P2.7
addCounterOutputChannel(s,'Dev1', 0, 'PulseGeneration');
addCounterOutputChannel(s,'Dev1', 1, 'PulseGeneration');
addCounterOutputChannel(s,'Dev1', 2, 'PulseGeneration');

% Initial Value for ctr0
s.Channels(4).Frequency = 1000;
s.Channels(4).DutyCycle = 0.001;
s.Channels(4).InitialDelay = 0;
% Initial Value for ctr1
s.Channels(5).Frequency = 1000;
s.Channels(5).DutyCycle = 0.001;
s.Channels(5).InitialDelay = 0;
% Initial Value for ctr2
s.Channels(6).Frequency = 1000;
s.Channels(6).DutyCycle = 0.001;
s.Channels(6).InitialDelay = 0;

%completion of the initilisation

% Initialize Session UserData Property
% Initialize the custom fields for managing the acquired data across callbacks.
% Add wanted recording data to here.
s.UserData.Data = [];
s.UserData.TimeStamps = [];
s.UserData.StartTime = [];
s.UserData.PWMduty = [];
s.UserData.Position = [];
s.UserData.RotationAngle = [];
s.UserData.RotationMatirx = [];
s.UserData.Timeinterval = [];
s.UserData.quat = [];
s.UserData.Velocity1 = [];
s.UserData.Velocity2 = [];

% part 2�� input
% Add Listeners
% Add listeners to session for available data and error events.
lh1 = addlistener(s, 'DataAvailable', @recordData);
lh2 = addlistener(s, 'ErrorOccurred', @(~,eventData) disp(getReport(eventData.Error)));

% Acquire Data
% Start the session in the background.
startBackground(s)
pause(time) % Increase or decrease the duration time to fit your needs.

% stop NI-USB device
stop(s)

% stop Aurora system
%aurora_device.stopTracking();
%delete(aurora_device);
% Clean Up
% Remove event listeners and clear the session and channels, if any.
delete(lh1)
delete(lh2)
%fclose(port);
% fclose(port);
% clear s lh1 lh2
%   part 3: output
% Log Data
global DAQ_pressure DAQ_cmd_pressure %Aurora_position Aurora_quat Aurora_theta
% Convert the acquired data and timestamps to a timetable in a workspace variable.
DAQ_pressure = timetable(seconds(s.UserData.TimeStamps),s.UserData.Data/3-0.12);
DAQ_cmd_pressure = timetable(seconds((s.UserData.Timeinterval)'),3*(s.UserData.PWMduty));
%Aurora_position = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.Position);
%Aurora_quat = timetable(seconds((s.UserData.Timeinterval)'),s.UserData.quat);
%Aurora_z_axis = s.UserData.RotationMatirx(:,3:3:size(s.UserData.RotationMatirx,2));
%theta = acos(sum(Aurora_z_axis.*[0;0;1]))/pi*180;
%Aurora_theta = timetable(seconds((s.UserData.Timeinterval)'),theta');

% Plot Data

 
% Clean Up
% Remove event listeners and clear the session and channels, if any.
delete(lh1)
delete(lh2)
%clear aurora_device
% fclose(port);
% clear s lh1 lh2


function recordData(src,eventData)

% tic
%global aurora_device pressure
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
%aurora_device.updateSensorDataPositionAndRotation();
% get the position in three directions, unit is m
%temP =  aurora_device.port_handles.trans/1000;
% get the rotation angle around x,y,z
%temrot = aurora_device.port_handles.rot;
%src.UserData.quat = [src.UserData.quat;temrot];
%src.UserData.RotationMatirx = [src.UserData.RotationMatirx,quat2rotm(temrot)];
%temR = quat2eul(temrot);
%temRx = temR(1);
%temRy = temR(2);
%temRz = temR(3);
%[R,~] =  aurora_device.measureTipOrientation();
%src.UserData.Position = [src.UserData.Position;temP];
%src.UserData.RotationAngle = [src.UserData.RotationAngle;[temRx,temRy,temRz]];

% velocity estimation function
% or add you own velocity estiomation function here
% <temP> is current step position and the time period is <ControlPeriod> 
%[temVx1,temVy1,temVz1] = getVelocitySimplified(temP);
%src.UserData.Velocity1 = [src.UserData.Velocity1; [temVx1,temVy1,temVz1]];

%[temVx2,temVy2,temVz2] = getVelocity5Points(temP);
%src.UserData.Velocity2 = [src.UserData.Velocity2; [temVx2,temVy2,temVz2]];

% Add any self-defined PWM generation function here
%-------------------------------------------------------%
%change PWM duty in a user-defined way
%[pwmtemp1,pwmtemp2,pwmtemp3] = Sine_PWM(eventData.TimeStamps(1));
src.Channels(4).DutyCycle = pwmtemp1(pwm_counter);
src.Channels(5).DutyCycle = pwmtemp2(pwm_counter);
src.Channels(6).DutyCycle = pwmtemp3(pwm_counter);
%------------------------------------------------------%
%if pwmtemp1(pwm_counter)<0.001
%    pwmtemp1(pwm_counter)=0.001;
%elseif pwmtemp1(pwm_counter)>1
%    pwmtemp1(pwm_counter)=1;
%else
%    pwmtemp1(pwm_counter)=pwmtemp1(pwm_counter);
%end

% Record PWM duty to the UserData property of the session.
src.UserData.PWMduty = [src.UserData.PWMduty;...
                       [pwmtemp1(pwm_counter),pwmtemp2(pwm_counter),pwmtemp3(pwm_counter)]];
% Record the starttime from the first execution of this callback function.

if isempty(src.UserData.StartTime)
    src.UserData.StartTime = eventData.TriggerTime;
end
pwm_counter=pwm_counter+1;
end
end
