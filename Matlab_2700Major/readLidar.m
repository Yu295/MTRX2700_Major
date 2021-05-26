function obstacle = readLidar(serialPort)
%% Voice Instructions

    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;

%% Serial
    s = serial(serialPort);
    set(s,'BaudRate',9600);
    fopen(s);
    fprintf(s, '*IDN?');
    
    count = 1;
    
    % reading for the lidar data
    [line, count] = fscanf(s,"%s");
    disp(line);
    
    
    obstacle_detected = 'Obstacle detected in your way. Please stop.';
    Speak(obj,obstacle_detected);
    
    