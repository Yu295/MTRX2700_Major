function angleMatch = readMagnet(SerialPort, angleToTurn)

    % tell the user which way to turn
    % overshoot_left indicates which way the user could overshoot
    % i.e. if they are turning CCW (i.e. left) they can turn too far left
    if (angleToTurn < 0)
        playPrompt('Please turn right slowly');
        overshoot_left = 0;
    else
        playPrompt('Please turn left slowly');
        overshoot_left = 1;
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
    
    % turning instructions are generally [-90, 90] degrees
    % with the exception of turning around which is 180 degrees
    MAX_TURN_INSTRUCTION = 180;
    
    % initialise values
    prevAngleDiff = angleToTurn;
    angleDiff = angleToTurn;
    
    while abs(angleDiff) > tolerance
        
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
        
        % calculate current deviation from ideal bearing
        % with positive values denoting CCW rotation is required
        % values should be in the range [-180, 180]
        angleDiff = idealAngle - bearing;
        if angleDiff > MAX_TURN_INSTRUCTION
            angleDiff = angleDiff - MAX_BEARING;
        elseif angleDiff < -MAX_TURN_INSTRUCTION
            angleDiff = angleDiff + MAX_BEARING;
        end
        
        disp([angleToTurn, bearing, idealAngle, angleDiff]);
        
        % detect if the user has turned too much
        % this will be denoted when angleDiff flips sign
        % i.e. goes from tolerance to -tolerance or vice versa
        % second condition is to prevent apparent overshoots
        % when angleDiff is near +/- 180
        if (angleDiff * prevAngleDiff < (-tolerance^2)) &&...
           (angleDiff * prevAngleDiff > (-0.5*MAX_TURN_INSTRUCTION^2))
            if overshoot_left
                overshoot_left = 0; 
                playPrompt('You turned too much. Please turn right.');
            else
                overshoot_left = 1;
                playPrompt('You turned too much. Please turn left.');  
            end
        end
        
        prevAngleDiff = angleDiff;     
        fclose(s);
        delete(s);
        clear s;
    end
    
    playPrompt('Clear to go forward. Please start walking.');
    angleMatch = 1; 
end