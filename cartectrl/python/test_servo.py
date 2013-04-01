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
# Set servomotor position
# id : ID of servomotor
# pos : Position of servomotor (min = 21 max = 1002)
# time : Time of mouvement
def goPos(id, pos, time=60):
        #pos (min = 21 max = 1002)
        #Set Torque
        sendCommand(id,3,[0x34,0x01,0x60])
        #Go pos
        sendCommand(id,6, [time,(pos & 0xFF),((pos >> 8) & 0xFF),0x0,id])
	
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
# Arbre de Noel
# id1 : ID of servomotor 1
# id2 : ID of servomotor 2
# id3 : ID of servomotor 3
def arbreDeNoel(id1,id2,id3):
        while 1:
        	sendCommand(id1,3, [0x35,0x01,0x01])
        	sendCommand(id2,3, [0x35,0x01,0x02])
        	sendCommand(id3,3, [0x35,0x01,0x04])
        	time.sleep(0.5)
        	sendCommand(id1,3, [0x35,0x01,0x02])
        	sendCommand(id2,3, [0x35,0x01,0x04])
        	sendCommand(id3,3, [0x35,0x01,0x01])
        	time.sleep(0.5)
        	sendCommand(id1,3, [0x35,0x01,0x04])
        	sendCommand(id2,3, [0x35,0x01,0x01])
        	sendCommand(id3,3, [0x35,0x01,0x02])
        	time.sleep(0.5)

#-----------------------------------------------------
# Clear error message
# id : ID of servomotor
def clearErrorMsg(id):
        sendCommand(id,3, [0x30,0x02,0x00,0x00])
        sendCommand(id,3, [0x35,0x01,0x00])

#-----------------------------------------------------
# Change motor ID
# currentID : ID of servomotor
# wantedID : Will set servomotor to this ID
def changeID(currentID, wantedID):
        sendCommand(currentID,1, [0x06,0x01,wantedID])

#-----------------------------------------------------
# Change motor ID
# id : ID of servomotor
def readStatus(id):
        sendCommand(id,7, [])
        readData(9)

#-----------------------------------------------------
# Read RAM adress
# id : ID of servomotor
# adr : adress to read
def readRAM(id, adress):
        sendCommand(id,4, [adress, 0x01])
        readData(12) 
      	
#-----------------------------------------------------
# Main        	
if 'ser' not in globals() or ser.isOpen() == False:
        # Configuration of UART : http://pyserial.sourceforge.net/pyserial_api.html
        # Timeout time is 100 second.
        ser = serial.Serial(2,115200,serial.EIGHTBITS,
                            serial.PARITY_NONE, serial.STOPBITS_ONE, 15);


readStatus(4)
#id = 1;
#end = 15;
#pos = 900;

#while id < end :
#        goPos( id, pos)
#        time.sleep(0.5)
#        id = id + 1

ser.close()


