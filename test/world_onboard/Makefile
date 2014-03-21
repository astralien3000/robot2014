CC=g++
CFLAGS=--std=c++11 -Wall -Wextra -I../../../aversive++/include/common -I../../src/world -g
LDFLAGS=-lsfml-graphics -lsfml-window -lsfml-system

EXEC=client

all: $(EXEC)

$(EXEC): client.cpp my_world.cpp ../../src/world/*
	$(CC) $(CFLAGS) -o $(EXEC) $^ $(LDFLAGS)

test: $(EXEC)
	./$(EXEC)

clean:
	rm -rf $(shell find . -name "*~" -o -name "#*#" -o -name "*.o")

mrproper: clean
	rm -rf $(EXEC)