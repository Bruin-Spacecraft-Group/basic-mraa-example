CC=gcc
CFLAGS=-Wall -Wextra -g

default: i2c_ads1115

i2c_ads1115: i2c_ads1115.o
	$(CC) -lmraa -lstdc++ -o i2c_ads1115 i2c_ads1115.o

i2c_ads1115.o: i2c_ads1115.cpp
	$(CC) $(CFLAGS) -c i2c_ads1115.cpp

clean:
	rm -rf *.o i2c_ads1115 *.tar.gz

dist:
	tar -czvf i2c_ads1115.tar.gz i2c_ads1115.cpp Makefile
