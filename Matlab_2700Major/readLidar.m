function data = readLidar(serialPort)
%% Voice Instructions
    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;
    obstacle_detected = 'Obstacle detected in your way. Please stop.';
    stand_still = 'Start scanning. Please stand still.';

%% Serial
    s = serial(serialPort);
    set(s,'BaudRate',9600);
    fopen(s);
    fprintf(s, '*IDN?');
    
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