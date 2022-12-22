package main

import (
	"bufio"
	"fmt"
	"log"
	"machine"
	"machine/debug"
	"machine/usb/hid/joystick"
	"os"
	"strconv"
	"strings"
	"time"

	"tinygo.org/x/drivers/mcp2515"

	"diy-ffb-wheel/motor"
	"diy-ffb-wheel/utils"
)

const (
	Lock2Lock     = 540
	HalfLock2Lock = Lock2Lock / 2
	MaxAngle      = 32768*HalfLock2Lock/360 - 1
)

var (
	spi   = machine.SPI0
	csPin = machine.GP28
)

func logPrint() {
	select {
	case v := <-debug.DebugLog:
		fmt.Printf("%#v\n", v)
	default:
	}
}

var js *joystick.Joystick

func init() {
	js = joystick.New(joystick.Definitions{
		ButtonCnt:    24,
		HatSwitchCnt: 0,
		AxisDefs: []joystick.Constraint{
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
		},
	})
}

var (
	axMap = map[int]int{
		0: 1,
		1: 2,
		2: 3,
		3: 4,
	}
	shift = [][]int{
		0: {2, 0, 1},
		1: {4, 0, 3},
		2: {6, 0, 5},
		3: {8, 0, 7},
	}
	fitx       = utils.Map(-32767, 32767, 0, 4)
	limitx     = utils.Limit(0, 3)
	fity       = utils.Map(-32767, 32767, 0, 3)
	limity     = utils.Limit(0, 2)
	prev   int = 0
)

func setShift(x, y int32) {
	const begin = 10
	dx, dy := limitx(fitx(x)), limity(fity(y))
	next := shift[dx][dy]
	if next != prev {
		if prev > 0 {
			js.SetButton(prev+begin-1, false)
		}
		if next > 0 {
			debug.Debug("shift", next)
			js.SetButton(next+begin-1, true)
		}
	}
	prev = next
}

func main() {
	log.SetFlags(log.Lmicroseconds)
	for !machine.Serial.DTR() {
		time.Sleep(100 * time.Millisecond)
	}
	if err := spi.Configure(
		machine.SPIConfig{
			Frequency: 500000,
			SCK:       machine.GP2,
			SDO:       machine.GP3,
			SDI:       machine.GP4,
			Mode:      0,
		},
	); err != nil {
		log.Print(err)
	}

	debug.Debug("boot:", "start")
	go func() {
		for v := range debug.DebugLog {
			log.Printf("%s:%v", v.Key, v.Value)
		}
	}()
	can := mcp2515.New(spi, csPin)
	can.Configure()
	if err := can.Begin(mcp2515.CAN500kBps, mcp2515.Clock8MHz); err != nil {
		log.Fatal(err)
	}
	if err := motor.Setup(can); err != nil {
		log.Fatal(err)
	}
	ticker := time.NewTicker(10 * time.Millisecond)
	fit := utils.Map(-MaxAngle, MaxAngle, -32767, 32767)
	limit := utils.Limit(-32767, 32767)
	limit2 := utils.Limit(-500, 500)
	go func() {
		axises := make([]int32, 6)
		scanner := bufio.NewScanner(os.Stdin)
		for scanner.Scan() {
			for i, s := range strings.Split(scanner.Text(), ",") {
				if i >= 6 {
					break
				}
				v, err := strconv.Atoi(s)
				if err != nil {
					break
				}
				axises[i] = int32(v)
			}
			setShift(axises[0], axises[1])
			for i, v := range axises[2:6] {
				js.SetAxis(axMap[i], int(v))
			}
		}
		log.Print(scanner.Err())
	}()
	cnt := 0
	for range ticker.C {
		state, err := motor.GetState(can)
		if err != nil {
			log.Print(err)
		}
		angle := fit(state.Angle)
		output := limit2(-angle) + int32(state.Verocity)*128
		force := js.CalcForces()
		switch {
		case angle > 32767:
			output -= 8 * (angle - 32767)
		case angle < -32767:
			output -= 8 * (angle + 32767)
		}
		output -= force[0]
		if cnt%100 == -1 {
			print(time.Now().UnixMilli(), ": ")
			print("v:", state.Verocity, ", ")
			print("c:", state.Current, ", ")
			print("a:", angle, ", ")
			print("f:", force[0], ", ", force[1], ", ")
			print("o:", output, ", ")
			println()
		}
		cnt++
		if err := motor.Output(can, int16(limit(output))); err != nil {
			log.Print(err)
		}
		js.SetButton(0, int(limit(angle)) > 30000)
		js.SetButton(1, int(limit(angle)) < -30000)
		js.SetAxis(0, int(limit(angle)))
		js.SendState()
	}
}

/* DiRT Rally 2.0
00:01:57.311727 1: &{2 1 0 255 0 0 0 0 0 4 63 0 0 [{0 0 0 0 0 0} {0 0 0 0 0 0}] 0 0 0 0 65535 0 0}
00:01:57.314466 SetEffect:[010101ffff00000000ffff043f0000000000]
00:01:57.314910 StartEffect:[1]
00:01:57.315188 SetCondition:[030100000000000000102710270000]
00:01:57.315484 SetEffect:[01010bffff00000000ffff043f0000000000]
00:01:57.315808 StartEffect:[1]
00:01:57.316065 SetEffect:[010101ffff00000000ffff043f0000000000]
00:01:57.316369 StartEffect:[1]
00:01:57.316617 SetCondition:[030100000000000000102710270000]
00:01:57.317265 SetEffect:[01010bffff00000000ffff043f0000000000]
00:01:57.317596 StartEffect:[1]
00:01:57.317887 SetEffect:[010101ffff00000000ffff043f0000000000]
00:01:57.318205 StartEffect:[1]
00:01:57.318454 SetCondition:[030100000000000000102710270000]
00:01:57.318750 SetEffect:[01010bffff00000000ffff043f0000000000]
00:01:57.319354 StartEffect:[1]
00:01:57.319614 SetEffect:[010101ffff00000000ffff043f0000000000]
*/
