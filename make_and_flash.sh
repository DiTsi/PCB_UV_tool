#!/bin/bash

NAME="main"
DEVICE="attiny2313"


avr-gcc -gdwarf-2 -Wall -fsigned-char -O1 -mmcu="$DEVICE" -c ./"$NAME".c
avr-gcc -g -mmcu="$DEVICE" -o ./"$NAME".elf ./"$NAME".o

# make flash HEX
avr-objcopy -j .text -j .data -O ihex ./"$NAME".elf ./"$NAME".flash.hex
# make eeprom HEX
#avr-objcopy -j .text -j .eeprom -O ihex ./"$NAME".elf ./"$NAME".eeprom.hex
avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex ./"$NAME".elf ./"$NAME".eeprom.hex

string=$(avr-size -C --mcu="$DEVICE" --target=ihex ./"$NAME".elf)
proc=$(echo $string | sed 's/.*(\([0-9]*\)\.[0-9]% Full.*Data.*/\1/')

echo -e "\n${proc}% of flash used"

# Check size
if [ $proc -ge 100 ]; then
	echo more than 100% flash used!
	exit 1
fi


avrdude -c usbasp -p $DEVICE -u -U flash:w:./"$NAME".flash.hex -F \
	-U eeprom:w:./"$NAME".eeprom.hex -F


exit 0
