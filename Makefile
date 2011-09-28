CC = gcc
CFLAGS = -Wall -O3 -msse4.1
LDFLAGS = -lm 

all: mjpeg_encoder

mjpeg_encoder: mjpeg_encoder.o

clean:
	rm -f *.o mjpeg_encoder
