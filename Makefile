CC = gcc
CFLAGS = -Wall -g -O3 -msse4.1
LDFLAGS = -lm -lprofiler

all: mjpeg_encoder

mjpeg_encoder: mjpeg_encoder.o

clean:
	rm -f *.o mjpeg_encoder
