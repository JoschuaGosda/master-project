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
serialInst = serial.Serial()

myPort = '/dev/ttyACM0'
myBaudRate = 38400
portList = []

print("AVAILABLE PORTS \n")
for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))


# setup configuration for serial interface
serialInst.baudrate = myBaudRate
serialInst.port = myPort
serialInst.open()

sleep(1)
serialInst.flushInput()

# wait until first serial data from arduino is available
while not serialInst.in_waiting:
    pass
    
#print("Mount the calibration weight to the load cell and then hit ENTER")
print("message from arduino: " + serialInst.readline().decode('utf-8').rstrip('\n'))
print("message from arduino: " + serialInst.readline().decode('utf-8').rstrip('\n'))
input()
serialInst.write(bytes('0', 'utf-8'))

#sleep(1)
time_prev = time()

while True:
    if serialInst.in_waiting: # get the number of bytes in the input buffer
        packet = serialInst.readline() # type: bytes  
        time_diff = time() - time_prev  
        str_receive = packet.decode('utf-8').rstrip('\n')
        print(str_receive)
        #print("serial freq: " + str(1/time_diff))
        time_prev = time()
        
