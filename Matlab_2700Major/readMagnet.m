function angleMatch = readMagnet(serialPort, angleToTurn)
%% Voice Instructions

    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;
    
%% Serial
    s = serial(serialPort, 'BaudRate', 9600);
    fopen(s);
    fprintf(s, '*IDN?');
    
    count = 1;
    %readAngle = 1;
    
    % read in current orientation
    [line, count] = fscanf(s,"%s");
    bearing = sscanf(line, "%d");
    
    fclose(s);
    delete(s);
    clear s;
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
    
    while abs(bearing - idealAngle) > tolerance
        
        % send flag to tell C program to send a bearing
        s = serialport(SerialPort, 9600);
        write(s, "2\0", "char");
        delete(s);
        clear s;
        
        % read in the bearing
        s = serial(serialPort, 'BaudRate', 9600);
        fopen(s);
        fprintf(s, '*IDN?');
        [line, count] = fscanf(s,"%s");
        bearing = sscanf(line, "%d");
        disp(bearing);
        
        fclose(s);
        delete(s);
        clear s;
    end
  
    move_forward = 'Clear to go forward. Please start walking.';
    Speak(obj,move_forward);
    angleMatch = 1; 
end