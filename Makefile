CC = gcc
CFLAGS = -Wall -g -O3 -msse
LDFLAGS = -lm

all: mjpeg_encoder

mjpeg_encoder: mjpeg_encoder.o

clean:
	rm -f *.o mjpeg_encoder
