% Parses comma-separated values transmitted by the C program through SCI1
% expected format: measured elev,set elev,set az,measured dist,ground dist
% - SerialPort: serialport object corresponding to SCI1
% - data: output matrix
function data = readSerial(SerialPort) 
    
    % send flag to signal C program to send LiDAR data
    start_panning = sprintf("1\n");
    write(SerialPort, start_panning, "char");
    
    data = []; % resulting data matrix
    A = []; % for storing the current row read in    
    
    while (1)
       
        line = readline(SerialPort);
        
         % single character flag is sent after the last set of data
        if (strlength(line) < 3)
            break;
        end
        
        A = sscanf(line, "%d,%d,%d,%d,%d");
        disp(A');
        data = [data; A'];
    end
end