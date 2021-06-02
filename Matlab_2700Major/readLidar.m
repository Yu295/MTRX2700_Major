function readLidar(SerialPort)
%% Serial
    send_lidar = sprintf("2\n");
    stop_lidar = sprintf("0\n");
    
    groundDist = 0;
    actualDist = 1;
    
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
    
    write(SerialPort, stop_lidar, "char");
    
    playPrompt('Obstacle detected in your way. Please stop.');
    
    % delay for 10 seconds to allow the user to stop
    pause(3);
    
    playPrompt('Start scanning. Please stand still.');
end