function angleMatch = readMagnet(SerialPort, angleToTurn)

    % tell the user which way to turn
    if (angleToTurn < 0)
        playPrompt('Please turn left slowly');
        overshoot_left = 1;
    else
        playPrompt('Please turn right slowly');
        overshoot_left = 0;
    end
    
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
    tolerance = 2; % degrees
    
    % keep bearings in [0, 360) degrees
    MIN_BEARING = 0;
    MAX_BEARING = 360;
    idealAngle = bearing + angleToTurn;
    
    if idealAngle >= MAX_BEARING
        idealAngle = idealAngle - MAX_BEARING;
    elseif idealAngle < MIN_BEARING
        idealAngle = idealAngle + MAX_BEARING;
    end
    
    prevAngleDiff = angleToTurn;
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
        
        angleDiff = bearing - idealAngle;
        if angleDiff > 180
            angleDiff = angleDiff - MAX_BEARING;
        elseif angleDiff < -180
            angleDiff = angleDiff + MAX_BEARING;
        end
        
        disp([angleToTurn, bearing, idealAngle, angleDiff]);
    
        if angleDiff * prevAngleDiff < -4 && overshoot_left
           overshoot_left = 0; 
           playPrompt('You turned too much. Please turn right.');
           
        elseif angleDiff * prevAngleDiff < -4 && ~overshoot_left
           overshoot_left = 1;
           playPrompt('You turned too much. Please turn left.');      
        end
        
        prevAngleDiff = angleDiff;     
        fclose(s);
        delete(s);
        clear s;
    end
    
    playPrompt('Clear to go forward. Please start walking.');
    angleMatch = 1; 
end