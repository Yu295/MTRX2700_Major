function data = readLidar(SerialPort)
%% Serial
    % assume an obstacle will be detected within 2 minutes
    s = serial(SerialPort, 'BaudRate', 9600, 'Timeout', 120);
    fopen(s);
    
    % reading the lidar data
    [line, ~] = fscanf(s,"%s");
    disp(line);
    data = line;
    playPrompt('Obstacle detected in your way. Please stop.');
    
    % delay for 10 seconds to allow the user to stop
    pause(3);
    
    playPrompt('Start scanning. Please stand still.');
    
    fclose(s);
    delete(s);
    clear s;