import serial.tools.list_ports
from time import sleep, time

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
    
#print("Mount the calibration weight to the load cell and then hit ENTER")
#print("message from arduino: " + arduino.readline().decode('utf-8').rstrip('\n'))
#print("message from arduino: " + arduino.readline().decode('utf-8').rstrip('\n'))
#input()
#arduino.write(bytes('0', 'utf-8'))

#sleep(1)
time_prev = time()
timestamp = time()

while True:
    if arduino.in_waiting: # get the number of bytes in the input buffer
        packet = arduino.readline() # type: bytes  
        time_diff = time() - time_prev  
        str_receive = packet.decode('utf-8').rstrip('\n')
        print(str_receive)
        force = float(str_receive)
        #print(f"force as float is {force}")
        #print("serial freq: " + str(1/time_diff))
        time_prev = time()
        #arduino.flushInput()
        print(time())
        sleep(1/80)
    
    if (time() - timestamp) >= (1.0/80.0):
        print(time()-timestamp)
        timestamp = time()
