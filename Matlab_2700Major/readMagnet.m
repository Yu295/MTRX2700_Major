function angleMatch = readMagnet(serialPort, angleToTurn)
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
    %readAngle = 1;
    
    % read in current orientation
    [line, count] = fscanf(s,"%s");
    bearing = sscanf(line, "%d");
    
    %% Angle calculations
    tolerance = 5; % degrees
    
    % keep bearings in [0, 360) degrees
    MIN_BEARING = 0;
    MAX_BEARING = 360;
    idealAngle = bearing + angleToTurn;
    
    if idealAngle >= MAX_BEARING
        idealAngle = idealAngle - MAX_BEARING;
    elseif idealAngle < MIN_BEARING
        idealAngle = idealAngle + MIN_BEARING;
    end
    
    %disp(line);
    %A = sscanf(line,"%d");
    while abs(bearing - idealAngle) > tolerance
        [line, count] = fscanf(s,"%s");
        bearing = sscanf(line, "%d");
    end
    
    move_forward = 'Clear to go forward. Please start walking.';
    Speak(obj,move_forward);
    angleMatch = 1;
    
    fclose(s);
    delete(s);
    clear s;
    
    
end