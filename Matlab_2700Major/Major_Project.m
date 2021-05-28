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

% read from serial port to check if there is obstacle detected in front
dist_to_obstacle = readLidar(SerialPort);

% send through a flag indicating task finished, can now start panning.
s = serialport(SerialPort, 9600);
%s1 = fopen(s);
start_panning = "1\0";
write(s, start_panning, "char");
%fclose(s1);
delete(s);
clear s;

% read from serial the elevation, azimuth, distance and ground distance
dataMatrix = readSerial(SerialPort);

% Mapping/Guiding module here
angleToTurn = 45; % degrees

% tell user which way to turn
if (angleToTurn > 0)
    Speak(obj, turn_right);
else
    Speak(obj, turn_left);
end

angleMatch = readMagnet(SerialPort, angleToTurn);

% send through a flag indicating user is facing the right way.
s = serialport(SerialPort, 9600);
%s1 = fopen(s);
finished_turning = "2\0";
write(s, finished_turning, "char");
%fclose(s1);
delete(s);
clear s;