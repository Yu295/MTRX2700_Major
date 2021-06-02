%% Serial
%SerialPort = 'COM5'; % Yujiao
SerialPort = 'COM4'; % Jason
finished_mapping = sprintf("2\n");

while (1)
    s = serialport(SerialPort, 9600);
    
    % read from serial port to check if there is obstacle detected in front
    readLidar(s);

    % read from serial the elevation, azimuth, distance and ground distance
    dataMatrix = readSerial(s);
    %disp(dataMatrix);

    %% Mapping and Guiding module
    % degrees, measured CCW from current position
    angleToTurn = -mapAndGuide(dataMatrix);
    write(s, finished_mapping, "char");
    
    % send through a flag to start sending magnetometer data
    angleMatch = readMagnet(s, angleToTurn);

    disp('restarting...');
    delete(s);
    clear s;
end