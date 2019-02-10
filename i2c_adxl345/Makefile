CC=gcc
CFLAGS=-Wall -Wextra -g

default: i2c_adxl345

i2c_adxl345: i2c_adxl345.o
	$(CC) -lmraa -lstdc++ -o i2c_adxl345 i2c_adxl345.o

i2c_adxl345.o: i2c_adxl345.cpp
	$(CC) $(CFLAGS) -c i2c_adxl345.cpp

clean:
	rm -rf *.o i2c_adxl345 *.tar.gz

dist:
	tar -czvf i2c_adxl345.tar.gz i2c_adxl345.cpp Makefile