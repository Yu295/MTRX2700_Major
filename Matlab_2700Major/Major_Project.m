%% Voice Instructions
NET.addAssembly('System.Speech');
obj = System.Speech.Synthesis.SpeechSynthesizer;
obj.Volume = 100;

turn_right = 'Please turn right slowly.';
turn_left = 'Please turn left slowly.';
move_forward = 'Clear to go forward. Please start walking.';
turn_around = 'No walkway in front. Please turn around.';

%Speak(obj,obstacle_detected);
%Speak(obj,turn_right);
%Speak(obj,turn_left);
%Speak(obj,move_forward);
%Speak(obj,turn_around);

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
disp(dataMatrix);

%% Mapping/Guiding module here
angleToTurn = 45; % degrees, measured CCW from current position

% tell user which way to turn
if (angleToTurn > 0)
    Speak(obj, turn_right);
else
    Speak(obj, turn_left);
end

% send through a flag to start sending magnetometer data
angleMatch = readMagnet(SerialPort, angleToTurn);

%% Serial Contd.
% send through a flag indicating user is facing the right way.
sendSerial(SerialPort, finished_turning);