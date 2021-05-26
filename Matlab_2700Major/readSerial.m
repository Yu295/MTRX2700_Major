function data = readSerial(serialPort) 
%% Voice Instructions

    NET.addAssembly('System.Speech');
    obj = System.Speech.Synthesis.SpeechSynthesizer;
    obj.Volume = 100;

%% Serial
    s = serial(serialPort);
    set(s,'BaudRate',9600);
    fopen(s);
    fprintf(s, '*IDN?');
    stand_still = 'Start scanning. Please stand still.';

    data = [];
    A = [];    
 
    %this for loop is for testing:
    %for mapCount = 1:5
    [line, count] = fscanf(s,"%s");
    Speak(obj,stand_still);
    %line = fscanf(s,"%d");
    while count < 5
        [line,count] = fscanf(s,"%s");
        A = sscanf(line,"%d,%d,%d,%d");
        data = [data; A'];
        disp(data);
    end

    fclose(s);
    delete(s);
    clear s;
end