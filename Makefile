TARGET=pico
TINEGO=tinygo
NAME=$(shell tinygo list .)
include .env

.PHONY: build all flash wait mon

build:
	mkdir -p build
	$(TINYGO) build -target $(TARGET) -o build/$(NAME).elf .

all: flash wait monitor

flash:
	$(TINYGO) flash -target $(TARGET) .

wait:
	sleep 2

mon:
	$(TINYGO) monitor -target $(TARGET)
