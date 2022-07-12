import serial.tools.list_ports
from time import sleep, time
import numpy as np

'''
Instructions how to use this:
#1. reboot arduino which then waits for any input over serial to calibrate 
2. run this python script and reboot arduino directly after that
3. hit ENTER as soon as weight to calibrate is mounted to load cell
4. readings that are sent from arduino are read and ready for processing
'''

# find active ports
ports = serial.tools.list_ports.comports()
arduino = serial.Serial()

myPort = '/dev/ttyACM0'
myBaudRate = 38400
portList = []

print("AVAILABLE PORTS \n")
for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))


# setup configuration for serial interface
arduino.baudrate = myBaudRate
arduino.port = myPort
arduino.open()

sleep(1)
arduino.flushInput()

# wait until first serial data from arduino is available
while not arduino.in_waiting:
    pass
    
force_log = []
t0 = time()

while True:
    if arduino.in_waiting: # get the number of bytes in the input buffer
        packet = arduino.readline() # type: bytes  
        str_receive = packet.decode('utf-8').rstrip('\n')
        force = float(str_receive)/1000.0
        print(force)
        force_log.append(force)
        if time()-t0 > 180.0:
            break

    
force_save = np.array(force_log)
np.save('./data/forceSignal-500', force_save)