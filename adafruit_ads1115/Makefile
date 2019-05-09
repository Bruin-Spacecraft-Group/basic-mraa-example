CC=gcc
CFLAGS=-Wall -Wextra -g

default: ads1115

ads1115: ads1115.o
	$(CC) -lmraa -lstdc++ -o ads1115 ads1115.o

ads1115.o: ads1115.cpp
	$(CC) $(CFLAGS) -c ads1115.cpp

clean:
	rm -rf *.o ads1115 *.tar.gz

dist:
	tar -czvf ads1115.tar.gz ads1115.cpp Makefile