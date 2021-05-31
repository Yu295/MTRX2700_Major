%% Voice Instructions
NET.addAssembly('System.Speech');
obj = System.Speech.Synthesis.SpeechSynthesizer;
obj.Volume = 100;

%% Serial
%SerialPort = 'COM5'; % Yujiao
SerialPort = 'COM4'; % Jason
start_panning = sprintf("1\n");
finished_turning = sprintf("3\n");

% read from serial port to check if there is obstacle detected in front
dist_to_obstacle = readLidar(SerialPort);

% send through a flag indicating task finished, can now start panning.
sendSerial(SerialPort, start_panning);

% read from serial the elevation, azimuth, distance and ground distance
dataMatrix = readSerial(SerialPort);
%disp(dataMatrix);

%% Mapping and Guiding module

% degrees, measured CCW from current position
angleToTurn = mapAndGuide(dataMatrix);

% send through a flag to start sending magnetometer data
angleMatch = readMagnet(SerialPort, angleToTurn);

%% Serial Contd.
% send through a flag indicating user is facing the right way.
sendSerial(SerialPort, finished_turning);