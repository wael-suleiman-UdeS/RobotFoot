import serial
import time

#-----------------------------------------------------
# Write on UART
# L : List of data to write
# debug : Activate debug
def writeUART(L, debug=False):
        s = ''.join(chr(i) for i in L)
        ser.write( s )
        
        if debug :
                print("Sending on UART : " + s)                
        
#-----------------------------------------------------
# Send command to servomotor
# id : ID of servomotor
# cmd : Command to send
# data : Data to send
def sendCommand( id, cmd, data=[], debug=False ):
        L = [255, 255] 		#Header
        L += [(7+len(data))]	#Packet Size
        L += [id]		#ID
        L += [cmd]		#Command
        L += [0,0]		#CheckSum 1 and 2
        L += data		#Data
        
        CheckSum1 = 0
        for i in L:
                CheckSum1 ^= i
                
        L[5] = CheckSum1 & 0xFE
        L[6] = ~CheckSum1 & 0xFE
        
        writeUART(L,debug)
#-----------------------------------------------------
# Set servomotor position
# id : ID of servomotor
# pos : Position of servomotor (min = 21 max = 1002)
# time : Time of mouvement
def goPos(id, pos, time=60):
        sendCommand(id,6, [time,(pos & 0xFF),((pos >> 8) & 0xFF),0x0,id])
	
#-----------------------------------------------------
# Read data of UART port
# strLength : Message length
# debug : Activate debug
def readData(strLength=13,debug=False):

        s = ser.read(strLength)
        if debug:
                print("Receiving on UART : " + s)
        return s
#-----------------------------------------------------
# Ask servomotor position
# id : ID of servomotor
def askPosition(id):
        sendCommand(id,4, [60, 2])

#-----------------------------------------------------
# Read servomotor position
# id : ID of servomotor
def readPosition(id):        

        template = ['header', 'header', 'Size', 'ID', 'cmd', 'check1', 'check2', 'data_0', 'data_1', 'data_2', 'data_3', 'data_4', 'data_5']  

        askPosition(id)

        s = readData(13)
        for i in range(1,13):
        	print(hex(ord(s[i])) + ' ' + template[i])

        pos = ord(s[9]) | (ord(s[10]) << 8);

        print('pos = ' + str(pos))

#-----------------------------------------------------
# Flash led of servormotor test
# id : ID of servomotor
def flashLed(id):
        while 1:
        	sendCommand(id,3, [0x35,0x01,0x01])
        	time.sleep(0.5)
        	sendCommand(id,3, [0x35,0x01,0x02])
        	time.sleep(0.5)
        	sendCommand(id,3, [0x35,0x01,0x04])
        	time.sleep(0.5)

#-----------------------------------------------------
# Clear error message
# id : ID of servomotor
def clearErrorMsg(id):
        sendCommand(id,3, [0x30,0x02,0x00,0x00])
        sendCommand(id,3, [0x35,0x01,0x00])
        	
#-----------------------------------------------------
# Main        	
if 'ser' not in globals() or ser.isOpen() == False:
        # Configuration of UART : http://pyserial.sourceforge.net/pyserial_api.html
        # Timeout time is 100 second.
        ser = serial.Serial(2,115200,serial.EIGHTBITS,
                            serial.PARITY_NONE, serial.STOPBITS_ONE, 100);

pos = 21#min = 21 max = 1002
id = 0xFD

#Servomotor led test
#flashLed(id)
        
#Servomotor position test
sendCommand(id,3,[0x34,0x01,0x60]) #??
#sendCommand(id,3,[22,2,0xAA, 0]) #??
goPos(id, pos)
time.sleep(1)
readPosition(id)

#Servomotor clear error message
#clearErrorMsg(id)

#Microcontroller test
#writeUART([3,3,3,3,3,3,3,3,3,3,3,3,3], True)
#readData(13,True)


ser.close()


