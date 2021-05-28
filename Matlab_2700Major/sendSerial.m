function sendSerial(SerialPort, str)
    s = serialport(SerialPort, 9600);
    %s1 = fopen(s);
    write(s, str, "char");
    %fclose(s1);
    delete(s);
    clear s;
end