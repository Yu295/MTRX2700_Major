function data = readSerial(serialPort) 
%% Serial
    s = serial(serialPort);
    set(s,'BaudRate',9600);
    fopen(s);
    fprintf(s, '*IDN?');

    data = [];
    A = [];    
 
    %this for loop is for testing:
    %for mapCount = 1:5
    [line, count] = fscanf(s,"%s");
    %line = fscanf(s,"%d");
    while count < 6
        [line,count] = fscanf(s,"%s");
        A = sscanf(line,"%d,%d,%d,%d,%d");
        data = [data; A'];
        disp(data);
    end

    fclose(s);
    delete(s);
    clear s;
end