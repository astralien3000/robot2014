AVERSIVE_BASE=/home/astralien3000/aversive--

MMCU=atmega2560

CXX=avr-g++
CXX_FLAGS=-Wall -L$(AVERSIVE_BASE)/build/avr -I$(AVERSIVE_BASE)/include/avr -mmcu=$(MMCU) -std=c++11 -fno-exceptions -fno-threadsafe-statics -O3 -D__STDC_LIMIT_MACROS
CXX_LIBS=-laversive_$(MMCU)

OBJCOPY=avr-objcopy

ELF=driver.elf
HEX=driver.hex
DEV=/dev/ttyACM0

all: $(HEX)

load: $(HEX)
#	sudo avarice -j $(DEV) --erase --program -f $(HEX)
#	sudo avrdude -cjtagmkI -p$(MMCU) -P $(DEV) -U flash:w:$(HEX) -D
	sudo avrdude -cwiring -p$(MMCU) -P $(DEV) -U flash:w:$(HEX) -D

clean:
	rm -f $(ELF) $(HEX) *~

$(ELF): main.cpp ziegler_nichols_algo.cpp
	$(CXX) $(CXX_FLAGS) $^ -o $@ $(CXX_LIBS)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $(ELF) $(HEX)
