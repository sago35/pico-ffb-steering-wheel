package main

import (
	"machine"
	"machine/usb/hid/keyboard"
	"time"
)

func main() {
	led := machine.LED
	led.Configure(machine.PinConfig{
		Mode: machine.PinOutput,
	})
	kb := keyboard.New()
	for {
		led.Low()
		time.Sleep(time.Second)
		led.High()
		time.Sleep(time.Second)
		println("!!")
		kb.Write([]byte("a"))
	}
}
