%% Serial
%SerialPort = 'COM5'; % Yujiao
SerialPort = 'COM4'; % Jason
finished_mapping = sprintf("2\n");

while (1)
    
    % open SCI1 object
    s = serialport(SerialPort, 9600, 'Timeout', 20);
    
    % read from serial port to check if there is obstacle detected in front
    readLidar(s);

    % read from serial the elevation, azimuth, distance and ground distance
    dataMatrix = readSerial(s);

    %% Mapping and Guiding module
    % find the best angle to navigate through the obstacles
    % in degrees, measured CCW from current position
    angleToTurn = -mapAndGuide(dataMatrix);
    
    % send through a flag to indicate mapping is finished
    write(s, finished_mapping, "char");
    
    % guide the user to turn to the right extent
    readMagnet(s, angleToTurn);
    
    % close SCI1
    disp('restarting...');
    delete(s);
    clear s;
end