%% Serial
SerialPort = 'COM5';

% read from serial port to check if there is obstacle detected in front
dist_to_obstacle = readLidar(SerialPort);

% send through a flag indicating task finished, can now start panning.
s = serialport('COM5',9600);
%s1 = fopen(s);
start_panning = "1\0";
write(s, start_panning, "char");
%fclose(s1);
delete(s);
clear s;


%read from serial the elevation, aimuth, distance and ground distance
dataMatrix = readSerial(SerialPort);

  
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