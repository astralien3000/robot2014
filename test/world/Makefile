CC=g++
CFLAGS=--std=c++11 -Wall -Wextra -I../../../aversive++/include/common -I../../src/world

EXEC=world

all: $(EXEC)

$(EXEC): main.cpp ../../src/world/*.cpp
	$(CC) $(CFLAGS) -o $(EXEC) main.cpp ../../src/world/*.cpp

test: $(EXEC)
	./$(EXEC)

clean:
	rm -rf $(shell find . -name "*~" -o -name "#*#" -o -name "*.o")

mrproper: clean
	rm -rf $(EXEC)
