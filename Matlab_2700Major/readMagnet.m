function angleMatch = readMagnet (serialPort,idealAngle)
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
    %idealAngle = 0;
    %readAngle = 1;
    
    % reading 10 lines for the lidar data
    
    [line, count] = fscanf(s,"%s");
    %disp(line);
    %A = sscanf(line,"%d");
    while line(1) ~= idealAngle
    [line, count] = fscanf(s,"%s");
    readAngle = sscanf(line,"%d"); 
    
    end
    
    move_forward = 'Clear to go forward. Please start walking.';
    Speak(obj,move_forward);
    angleMatch = 1;
    
    fclose(s);
    delete(s);
    clear s;
    
    
end