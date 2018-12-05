CC=gcc
CFLAGS=-Wall -Wextra -g

default: test

test: test.o
	$(CC) -lmraa -o test test.o

test.o: test.c
	$(CC) $(CFLAGS) -c test.c

clean:
	rm -rf *.o test *.tar.gz

dist:
	tar -czvf test_mraa.tar.gz test.c Makefile
	
