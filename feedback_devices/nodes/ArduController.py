import serial
import time

class ArduWheel():

    def __init__(self, port):
        self.arduino = serial.Serial(port, 9600, 8, 'N', 1, timeout=0.1)
        time.sleep(1)
        self.arduino.write('x')
        print self.arduino.readline()
        self.Stop()
    
    def Control(self, motor_power):
        if abs(motor_power) > 1.0:
            print "Motors commanded out of speed bounds"
            raise BaseException
        command_bytes = self.ConvertToRawBytes(motor_power)
        self.arduino.write(command_bytes)
        print self.arduino.readline()
        
    def ConvertToRawBytes(self, motor_power):
        command_bytes = bytearray(4)
        command_bytes[0] = '$'
        if motor_power >= 0.0:
            command_bytes[1] = 'O'
        else:
            command_bytes[1] = 'C'
        command_bytes[2] = int(255.0 * abs(motor_power))
        command_bytes[3] = '\n'
        return command_bytes
        
    def Stop(self):
        self.Control(0.0)

#Startup code run only if this file is executed directly        
if __name__ == '__main__':
    mybot = ArduWheel("/dev/ttyACM0")
    for i in range(10):
        print "Cycling force up and down"
        speed = 0.0
        while speed >= -1.0:
            mybot.Control(speed)
            speed = speed - .1
            time.sleep(1)
        speed = -1.0
        while speed <= 0.0:
            mybot.Control(speed)
            speed = speed + .1
            time.sleep(1)
    mybot.Stop()
