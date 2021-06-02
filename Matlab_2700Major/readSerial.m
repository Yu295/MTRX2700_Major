function data = readSerial(SerialPort) 
%% Serial
    s = serial(SerialPort, 'BaudRate', 9600, 'Timeout', 30);
    fopen(s);

    data = [];
    A = [];    
 
    %this for loop is for testing:
    %for mapCount = 1:5
    %line = fscanf(s,"%d");
    while (1)
        [line,~] = fscanf(s,"%s");
        if (strlength(line) < 3)
            break;
        end
        A = sscanf(line,"%d,%d,%d,%d,%d");
        disp(A');
        data = [data; A'];
    end
    
    fclose(s);
    delete(s);
    clear s;
end