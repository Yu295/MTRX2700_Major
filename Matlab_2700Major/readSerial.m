function data = readSerial(SerialPort) 
%% Serial
    start_panning = sprintf("1\n");
    write(SerialPort, start_panning, "char");
    data = [];
    A = [];    
    
    while (1)
        line = readline(SerialPort);
        if (strlength(line) < 3)
            break;
        end
        A = sscanf(line, "%d,%d,%d,%d,%d");
        disp(A');
        data = [data; A'];
    end
end