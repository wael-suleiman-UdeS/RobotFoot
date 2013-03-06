import serial
import time

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

	s = ''.join(chr(i) for i in L)
	
	ser.write( s )


#-----------------------------------------------------
def goPos(id, pos, time=60):
        sendCommand(id,6, [time,(pos & 0xFF),((pos >> 8) & 0xFF),0x0,id])
	
#-----------------------------------------------------

if 'ser' not in globals() or ser.isOpen() == False:
        ser = serial.Serial(2,115200,serial.EIGHTBITS);	

pos   = 512 #min = 21 max = 1002
temps = 60
id = 0xFD


sendCommand(id,3, [0x34,0x01,0x60])
#sendCommand(id,3, [22,2,0xAA, 0])
goPos(id, pos)
time.sleep(1)
sendCommand(id,4, [60, 2])

template = ['header', 'header', 'Size', 'ID', 'cmd', 'check1', 'check2', 'data_0', 'data_1', 'data_2', 'data_3', 'data_4', 'data_5']  

s = ser.read(13)
for i in range(0,13):
	print(hex(ord(s[i])) + ' ' + template[i])

p = ord(s[9]) | (ord(s[10]) << 8);

print('pos = ' + str(p))

#while 1:
#	sendCommand(0xFD,3, [0x35,0x01,0x01])
#	time.sleep(0.5)
#	sendCommand(0xFD,3, [0x35,0x01,0x02])
#	time.sleep(0.5)
#	sendCommand(0xFD,3, [0x35,0x01,0x04])
#	time.sleep(0.5)

#ser.close()


