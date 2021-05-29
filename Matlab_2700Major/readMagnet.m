function angleMatch = readMagnet(SerialPort, angleToTurn)
%% Voice Instructions

    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;
    
%% Serial   
    send_mag = sprintf("2\n");
    % send signal to C program to send data
    disp('Sending flag');
    sendSerial(SerialPort, send_mag);
    
    % read in data
    s = serial(SerialPort, 'BaudRate', 9600, 'Timeout', 30);
    fopen(s);
    [line, ~] = fscanf(s, "%s");
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
        idealAngle = idealAngle + MAX_BEARING;
    end
    
    while abs(bearing - idealAngle) > tolerance
        
        % send flag to tell C program to send a bearing
        s = serialport(SerialPort, 9600);
        write(s, send_mag, "char");
        delete(s);
        clear s;
        
        % read in the bearing
        s = serial(SerialPort, 'BaudRate', 9600, 'Timeout', 30);
        fopen(s);
        [line, ~] = fscanf(s,"%s");
        bearing = sscanf(line, "%d");
        disp([bearing, idealAngle]);
        
        fclose(s);
        delete(s);
        clear s;
    end
  
    move_forward = 'Clear to go forward. Please start walking.';
    Speak(obj,move_forward);
    angleMatch = 1; 
end