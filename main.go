package main

import (
	"fmt"
	"machine"
	"machine/usb/hid/joystick"
	"runtime"
	"time"

	"tinygo.org/x/drivers/mcp2515"
)

var (
	spi   = machine.SPI0
	csPin = machine.GP28
)

func ReadFrame(can *mcp2515.Device) (*mcp2515.CANMsg, error) {
	for !can.Received() {
		runtime.Gosched()
	}
	return can.Rx()
}

func main() {
	for !machine.Serial.DTR() {
		time.Sleep(100 * time.Millisecond)
	}
	led := machine.LED
	led.Configure(machine.PinConfig{
		Mode: machine.PinOutput,
	})
	if err := spi.Configure(
		machine.SPIConfig{
			Frequency: 500000,
			SCK:       machine.GP2,
			SDO:       machine.GP3,
			SDI:       machine.GP4,
			Mode:      0,
		},
	); err != nil {
		fmt.Println(err.Error())
	}

	js := joystick.New(joystick.Definitions{
		ButtonCnt:    16,
		HatSwitchCnt: 1,
		AxisDefs: []joystick.Constraint{
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
			{MinIn: -10000, MaxIn: 10000, MinOut: -32768, MaxOut: 32767},
		},
	})

	/*
		go func() {
			for v := range debug.DebugLog {
				fmt.Printf("%#v\n", v)
			}
		}()
	*/

	for {
		time.Sleep(300 * time.Millisecond)
		js.SetButton(0, true)
		js.SetAxis(1, -100)
		js.SendState()
		b, _ := js.MarshalBinary()
		fmt.Printf("%x\n", b)
		time.Sleep(300 * time.Millisecond)
		js.SetButton(0, false)
		js.SetAxis(1, 100)
		js.SendState()
		b, _ = js.MarshalBinary()
		fmt.Printf("%x\n", b)
	}

	can := mcp2515.New(spi, csPin)
	can.Configure()
	if err := can.Begin(mcp2515.CAN500kBps, mcp2515.Clock8MHz); err != nil {
		fmt.Println(err.Error())
	}
	if err := can.Tx(0x109, 8, []byte{0, 0, 0, 0, 0, 0, 0, 0}); err != nil {
		fmt.Println(err.Error())
	}
	msg, err := ReadFrame(can)
	if err != nil {
		fmt.Println(err.Error())
	}
	fmt.Printf("%#v\n", msg)
	if err := can.Tx(0x106, 8, []byte{0x80, 0, 0, 0, 0, 0, 0, 0}); err != nil {
		fmt.Println(err.Error())
	}
	msg, err = ReadFrame(can)
	if err != nil {
		fmt.Println(err.Error())
	}
	fmt.Printf("%#v\n", msg)
	if err := can.Tx(0x105, 8, []byte{0x00, 0, 0, 0, 0, 0, 0, 0}); err != nil {
		fmt.Println(err.Error())
	}
	msg, err = ReadFrame(can)
	if err != nil {
		fmt.Println(err.Error())
	}
	fmt.Printf("%#v\n", msg)

	for {
		if err := can.Tx(0x107, 8, []byte{0x01, 0x01, 0x02, 0x04, 0x55, 0, 0, 0}); err != nil {
			fmt.Println(err.Error())
		}
		msg, err = ReadFrame(can)
		if err != nil {
			fmt.Println(err.Error())
			continue
		}
		fmt.Printf("%#v\n", msg)
		if err := can.Tx(0x32, 8, []byte{0x00, 0x00, 0, 0, 0, 0, 0, 0}); err != nil {
			fmt.Println(err.Error())
		}
		led.Low()
		time.Sleep(5 * time.Millisecond)
		led.High()
		time.Sleep(5 * time.Millisecond)
	}
}
