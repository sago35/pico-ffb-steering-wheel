package main

import (
	"bufio"
	"log"
	"machine"
	"machine/usb/joystick"
	"os"
	"strconv"
	"strings"
	"time"

	"tinygo.org/x/drivers/mcp2515"

	"diy-ffb-wheel/motor"
	"diy-ffb-wheel/pid"
	"diy-ffb-wheel/utils"
)

const (
	DEBUG         = false
	Lock2Lock     = 540
	HalfLock2Lock = Lock2Lock / 2
	MaxAngle      = 32768*HalfLock2Lock/360 - 1
)

var (
	spi   = machine.SPI0
	csPin = machine.GP28
)

var (
	js *joystick.Joystick
	ph *pid.PIDHandler
)

func init() {
	ph = pid.NewPIDHandler()
	js = joystick.Enable(joystick.Definitions{
		ReportID:     1,
		ButtonCnt:    24,
		HatSwitchCnt: 0,
		AxisDefs: []joystick.Constraint{
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: 0, MaxIn: 32767, MinOut: 0, MaxOut: 32767},
			{MinIn: 0, MaxIn: 32767, MinOut: 0, MaxOut: 32767},
			{MinIn: 0, MaxIn: 32767, MinOut: 0, MaxOut: 32767},
			{MinIn: 0, MaxIn: 32767, MinOut: 0, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
		},
	}, ph.RxHandler, ph.SetupHandler, pid.Descriptor)
}

var (
	axMap = map[int]int{
		0: 1, // side
		1: 2, // throttle
		2: 4, // brake
		3: 3, // clutch
	}
	shift = [][]int{
		0: {2, 0, 1},
		1: {4, 0, 3},
		2: {6, 0, 5},
		3: {8, 0, 7},
	}
	fitx   = utils.Map(-32767, 32767, 0, 4)
	limitx = utils.Limit(0, 3)
	fity   = utils.Map(-32767, 32767, 0, 3)
	limity = utils.Limit(0, 2)
	prev   = 0
)

func setShift(x, y int32) int {
	const begin = 10
	dx, dy := limitx(fitx(x)), limity(fity(y))
	next := shift[dx][dy]
	if next != prev {
		if prev > 0 {
			js.SetButton(prev+begin-1, false)
		}
		if next > 0 {
			js.SetButton(next+begin-1, true)
		}
	}
	prev = next
	return next
}

func absInt32(n int32) int32 {
	if n < 0 {
		return -n
	}
	return n
}

func main() {
	log.SetFlags(log.Lmicroseconds)
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
	receiver := true
	go func() {
		defer func() { receiver = false }()
		time.Sleep(10 * time.Second)
		axises := make([]int32, 8)
		scanner := bufio.NewScanner(os.Stdin)
		for scanner.Scan() {
			for i, s := range strings.Split(scanner.Text(), ",") {
				if i >= len(axises) {
					break
				}
				v, err := strconv.Atoi(s)
				if err != nil {
					break
				}
				axises[i] = int32(v)
			}
			for i, v := range axises[2:6] {
				js.SetAxis(axMap[i], int(v))
			}
			shift := setShift(axises[0], axises[1])
			// for sequential mode
			switch {
			case axises[7] > 0:
				js.SetButton(8, true)
			case axises[6] > 0:
				js.SetButton(9, true)
			default:
				js.SetButton(8, false)
				js.SetButton(9, false)
			}
			if shift == 0 {
				js.SetButton(0, axises[3] > 8192)
				js.SetButton(1, axises[4] > 8192)
				js.SetButton(5, axises[5] > 8192)
				js.SetButton(6, axises[2] > 8192)
			} else {
				js.SetButton(0, false)
				js.SetButton(1, false)
				js.SetButton(5, false)
				js.SetButton(6, false)
			}
		}
		log.Print(scanner.Err())
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
	limit1 := utils.Limit(-32767, 32767)
	limit2 := utils.Limit(-500, 500)
	cnt := 0
	for range ticker.C {
		state, err := motor.GetState(can)
		if err != nil {
			log.Print(err)
		}
		angle := fit(state.Angle)
		output := limit2(-angle) + int32(state.Verocity)*128
		force := ph.CalcForces()
		switch {
		case angle > 32767:
			output -= 8 * (angle - 32767)
		case angle < -32767:
			output -= 8 * (angle + 32767)
		}
		output -= force[0]
		if DEBUG && cnt%100 == 0 {
			print(time.Now().UnixMilli(), ": ")
			print("v:", state.Verocity, ", ")
			print("c:", state.Current, ", ")
			print("a:", angle, ", ")
			print("f:", force[0], ", ", force[1], ", ")
			print("o:", output, ", ", receiver)
			println()
		}
		cnt++
		if err := motor.Output(can, int16(limit1(output))); err != nil {
			log.Print(err)
		}
		js.SetButton(2, angle > 32767)
		js.SetButton(3, angle < -32767)
		js.SetAxis(0, int(limit1(angle)))
		js.SetAxis(5, int(limit1(angle)))
		js.SendState()
	}
}
