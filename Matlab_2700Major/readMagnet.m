% Guides user to turn to the right extent via audio prompts based on their
% bearing as measured by the magnetometer
% - SerialPort: serialport object corresponding to SCI1
% - angleToTurn: desired angle measured CCW in degrees 
function readMagnet(SerialPort, angleToTurn)  
    
    % flag indicating user has finished turning
    finished_turning = sprintf("3\n");
    
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
    write(SerialPort, send_mag, "char");
    
    % read in data
    line = readline(SerialPort);
    bearing = sscanf(line, "%d");

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
        
        % read in the bearing
        write(SerialPort, send_mag, "char");
        line = readline(SerialPort);
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
    end
    
    % send flag to stop further magnetometer data
    write(SerialPort, finished_turning, "char");
    
    % play audio instruction to indicate it's safe to proceed
    playPrompt('Clear to go forward. Please start walking.');
end