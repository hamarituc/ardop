#	ARDOPC Makefile

OBJS = Viterbi.o ackbycarrier.o LinSerial.o KISSModule.o pktARDOP.o pktSession.o BusyDetect.o i2cDisplay.o ALSASound.o ARDOPC.o ardopSampleArrays.o ARQ.o FFT.o FEC.o HostInterface.o Modulate.o rs.o berlekamp.o galois.o SoundInput.o TCPHostInterface.o SCSHostInterface.o

# Configuration:
CFLAGS = -DLINBPQ -MMD -g 
CC = gcc

vpath %.c /mnt/Source/TeensyProjectsA3K/TeensyProjects/ARDOP3
vpath %.h /mnt/Source/TeensyProjectsA3K/TeensyProjects/ARDOP3

#vpath %.c /mnt/Source/TeensyProjects/ARDOP3
#vpath %.h /mnt/Source/TeensyProjects/ARDOP3
vpath %.o ./
			                       
all: ardop3
			
ardop3: $(OBJS)
	gcc $(OBJS) -Xlinker -Map=output.map -lrt -lm -lpthread -lasound -o ardop3


-include *.d

clean :
	rm -f ardop3 $(OBJS) output.map
	rm -f *.d
