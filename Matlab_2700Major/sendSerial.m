function sendSerial(SerialPort, str)
    s = serialport(SerialPort, 9600);
    write(s, str, "char");
    delete(s);
    clear s;
end