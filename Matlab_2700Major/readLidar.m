function data = readLidar(SerialPort)
%% Voice Instructions
    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;
    obstacle_detected = 'Obstacle detected in your way. Please stop.';
    stand_still = 'Start scanning. Please stand still.';

%% Serial
    s = serial(SerialPort, 'BaudRate', 9600, 'Timeout', 30);
    fopen(s);
    
    count = 1;
    
    % reading for the lidar data
    [line, count] = fscanf(s,"%s");
    disp(line);
    data = line;
    Speak(obj,obstacle_detected);
    
    % delay for 10 seconds to allow the user to stop
    i = 5;
    pause(i);
    
    Speak(obj,stand_still);
    
    fclose(s);
    delete(s);
    clear s;