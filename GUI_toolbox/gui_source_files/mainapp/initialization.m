function initialization
clc
disp('initializing...');
%% aurora system initialization
global aurora_device PositionArray  PositionLastStep pressure baudrate 
%load pressure_array.mat
% saving 5-step position to estimate velocity
PositionArray = zeros(5,3);
%ControlPeriod = 0.1;  % defalut as 25ms
PositionLastStep = zeros(3,1);  % original position

% part 1 : initilisation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

serial_present = instrfind;

if(~isempty(serial_present)) 
    aurora_device.openSerialPort();
    aurora_device.init();
    aurora_device.detectAndAssignPortHandles();
    aurora_device.initPortHandleAll();
    aurora_device.enablePortHandleDynamicAll();
    %set BaudRate
    aurora_device.setBaudRate(baudrate);
    % enable tracking status before start tracking
    % sensor status needs to be setted as SENSOR_STATUS_VALID
    aurora_device.port_handles.updateSensorStatus(aurora_device.SENSOR_STATUS_VALID);
    % start tracking
    aurora_device.startTracking();
    % beeping for tracking start noticing 
    aurora_device.BEEP('1');
    disp('initialization completed');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end