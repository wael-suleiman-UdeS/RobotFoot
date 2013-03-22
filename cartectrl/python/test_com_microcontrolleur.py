import serial
import time

# Set true for debug information
DEBUG = True

#-----------------------------------------------------
# Write on UART
# L : List of data to write
# debug : Activate debug
def writeUART(L):
        s = ''.join(chr(i) for i in L)
        ser.write( s )
        
        if DEBUG :
                for i in range(0,len(s)):
                        print(hex(ord(s[i])))
                print("Sending on UART : " + s)                
        
#-----------------------------------------------------
# Send command to servomotor
# id : ID of servomotor
# cmd : Command to send
# data : Data to send
def sendCommand( id, cmd, data=[] ):
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
        
        writeUART(L)
	
#-----------------------------------------------------
# Read data of UART port
# strLength : Message length
# debug : Activate debug
def readData(strLength=12):

        s = ser.read(strLength)
        if DEBUG:
                for i in range(0,len(s)):
                        print(hex(ord(s[i])))
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
        for i in range(0,13):
        	print(hex(ord(s[i])) + ' ' + template[i])

        pos = ord(s[9]) | (ord(s[10]) << 8);

        print('pos = ' + str(pos))
#-----------------------------------------------------
# Main        	
if 'ser' not in globals() or ser.isOpen() == False:
        # Configuration of UART : http://pyserial.sourceforge.net/pyserial_api.html
        # Timeout time is 100 second.
        ser = serial.Serial(2,115200,serial.EIGHTBITS,
                            serial.PARITY_NONE, serial.STOPBITS_ONE, 15);

id = 0xFD 

#askPosition(id)

#Microcontroller test
writeUART([0xff,0xff,0xd,0xfd,0x44,0x60,0x9e,0x3c,0x2,0xe8,0x3,0x0,0x0])
#writeUART([0xff,0xff,0x0D,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x0D,0x0A])
#readData(12)


ser.close()


