%% Serial
SerialPort = 'COM5';
dataMatrix = readSerial(SerialPort);

  
%% Voice Instructions

NET.addAssembly('System.Speech');
obj = System.Speech.Synthesis.SpeechSynthesizer;
obj.Volume = 100;

obstacle_detected = 'Obstacle detected in your way. Please stop.';
stand_still = 'Start scanning. Please stand still.';
turn_right = 'Please turn right slowly.';
turn_left = 'Please turn left slowly.';
move_forward = 'Clear to go forward. Please start walking.';
turn_around = 'No walkway in front. Please turn around.';

%Speak(obj,obstacle_detected);
%Speak(obj,turn_right);
%Speak(obj,turn_left);
%Speak(obj,move_forward);
%Speak(obj,turn_around);


