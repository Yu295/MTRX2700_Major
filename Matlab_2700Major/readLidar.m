% Reads LiDAR distances until an obstacle is detected in front of the user
% expected format: measured dist,ground dist
% - SerialPort: serialport object corresponding to SCI1
function readLidar(SerialPort)
    
    % flags to synchronise with C program
    send_lidar = sprintf("2\n");
    stop_lidar = sprintf("0\n");
    
    % initial distance values (mm)
    groundDist = 0; % how far the ground is expected to be
    actualDist = 1; % actual LiDAR reading
    
    % obstacle is detected if the reading is less than where we expect the
    % ground to be
    while groundDist < actualDist
        
        % send flag to tell C program to send a LiDAR reading
        write(SerialPort, send_lidar, "char");
        
        % read in the distances 
        line = readline(SerialPort);
        data = sscanf(line, "%d,%d");
        actualDist = data(1);
        groundDist = data(2);
        disp(data');
    end
    
    % send flag through serial
    write(SerialPort, stop_lidar, "char");
    
    % instruct user to stop after obstacle is detected in their path
    playPrompt('Obstacle detected in your way. Please stop.');
    
    % delay for 3 seconds to allow the user to stop
    pause(3);
    
    % notify user panning will begin
    playPrompt('Start scanning. Please stand still.');
end