CC=gcc
CFLAGS=-Wall -Wextra -g

default: pcf8574

pcf8574: pcf8574.o
	$(CC) -lmraa -lstdc++ -o pcf8574 pcf8574.o

pcf8574.o: pcf8574.cpp
	$(CC) $(CFLAGS) -c pcf8574.cpp

clean:
	rm -rf *.o pcf8574 *.tar.gz

dist:
	tar -czvf pcf8574.tar.gz pcf8574.cpp Makefile