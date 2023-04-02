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
	Lock2Lock     = 540
	HalfLock2Lock = Lock2Lock / 2
	MaxAngle      = 32768*HalfLock2Lock/360 - 1
)

var (
	spi   = machine.SPI0
	csPin = machine.D5
)

var (
	js *joystick.Joystick
	ph *pid.PIDHandler
)

func init() {
	ph = pid.NewPIDHandler()
	//dbg.Printf("1 %p %d\n", &pid.Descriptor, len(pid.Descriptor))
	//uptr := unsafe.Pointer(&(pid.Descriptor))
	//hdr := (*reflect.SliceHeader)(uptr)
	//dbg.Printf("Data: %x\n", hdr.Data)

	js = joystick.Enable(joystick.Definitions{
		ReportID:     1,
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
	fitx       = utils.Map(-32767, 32767, 0, 4)
	limitx     = utils.Limit(0, 3)
	fity       = utils.Map(-32767, 32767, 0, 3)
	limity     = utils.Limit(0, 2)
	prev   int = 0
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

var (
	disableWheel = false
)

func main() {
	initx()

	uart := machine.DefaultUART
	tx := machine.UART_TX_PIN
	rx := machine.UART_RX_PIN
	uart.Configure(machine.UARTConfig{TX: tx, RX: rx})

	//time.Sleep(3 * time.Second)
	log.SetFlags(log.Lmicroseconds)
	err := spi.Configure(
		machine.SPIConfig{
			Frequency: 500000,
			SCK:       machine.SPI0_SCK_PIN,
			SDO:       machine.SPI0_SDO_PIN,
			SDI:       machine.SPI0_SDI_PIN,
			Mode:      0,
		},
	)
	if err != nil {
		log.Fatal(err)
	}
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
			shift := setShift(axises[0], axises[1])
			for i, v := range axises[2:6] {
				js.SetAxis(axMap[i], int(v))
			}
			if shift == 0 {
				js.SetButton(0, axises[3] >= 16384)
				js.SetButton(1, axises[4] >= 16384)
			}
		}
		log.Print(scanner.Err())
	}()
	can := mcp2515.New(spi, csPin)
	can.Configure()
	if err := can.Begin(mcp2515.CAN500kBps, mcp2515.Clock8MHz); err != nil {
		log.Fatal(err)
	}
	if !disableWheel {
		if err := motor.Setup(can); err != nil {
			log.Fatal(err)
		}
	}

	buttons := []machine.Pin{
		machine.A2, machine.A3,
	}
	for _, btn := range buttons {
		btn.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	}

	machine.InitADC()
	accel := machine.ADC{Pin: machine.A0}
	accel.Configure(machine.ADCConfig{})
	cfg := []ADCConfig{
		{Min: 57000, Max: 62500},
		{Min: 8800, Max: 13000},
	}

	brake := machine.ADC{Pin: machine.A1}
	brake.Configure(machine.ADCConfig{})

	ticker := time.NewTicker(10 * time.Millisecond)
	fit := utils.Map(-MaxAngle, MaxAngle, -32767, 32767)
	limit1 := utils.Limit(-32767, 32767)
	limit2 := utils.Limit(-500, 500)
	cnt := 0
	limit3 := utils.Limit(-1000, 1000)
	limit3cnt := 10
	btn0 := false
	uartButtons := make([]byte, 0, 3)
	buttons2 := uint16(0)
	for range ticker.C {
		state := &motor.MotorState{}
		var err error
		if !disableWheel {
			state, err = motor.GetState(can)
			if err != nil {
				log.Print(err)
			}
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
		if limit3cnt > 0 {
			limit3cnt--
			output = limit3(output)
		}
		a := accel.Get()
		b := brake.Get()

		for uart.Buffered() > 0 {
			data, _ := uart.ReadByte()
			uartButtons = append(uartButtons, data)
			if len(uartButtons) == 3 {
				if uartButtons[0] != 0xFF {
					uartButtons[0], uartButtons[1] = uartButtons[1], uartButtons[2]
					uartButtons = uartButtons[:2]
				} else {
					buttons2 = (uint16(uartButtons[1]) << 8) + uint16(uartButtons[2])
					uartButtons = uartButtons[:0]
				}
			}
		}

		if cnt%100 == -1 {
			//btn0 = !btn0
			print(time.Now().UnixMilli(), ": ")
			print("v:", state.Verocity, ", ")
			print("c:", state.Current, ", ")
			print("a:", angle, ", ")
			print("f:", force[0], ", ", force[1], ", ")
			print("o:", output, ", ")
			print("A:", a, ", ")
			print("A:", cfg[0].Convert(a), ", ")
			print("B:", b, ", ")
			print("B:", cfg[1].Convert(b), ", ")
			println()
		}
		cnt++
		if !disableWheel {
			if err := motor.Output(can, int16(limit1(output))); err != nil {
				log.Print(err)
			}
		}
		js.SetButton(1, btn0)
		//js.SetButton(2, angle > 32767)
		//js.SetButton(3, angle < -32767)
		//js.SetButton(4, !buttons[0].Get())
		//js.SetButton(5, !buttons[1].Get())
		if true {
			js.SetButton(2, (buttons2&0x0008) > 0)
			js.SetButton(3, (buttons2&0x0004) > 0)
			js.SetButton(4, (buttons2&0x8000) > 0)
			js.SetButton(5, (buttons2&0x4000) > 0)
			js.SetButton(6, (buttons2&0x2000) > 0)
			js.SetButton(7, (buttons2&0x1000) > 0)
			js.SetButton(8, (buttons2&0x0800) > 0)
			js.SetButton(9, (buttons2&0x0400) > 0)
			js.SetButton(10, (buttons2&0x0200) > 0)
			js.SetButton(11, (buttons2&0x0100) > 0)
			js.SetButton(12, (buttons2&0x0080) > 0)
			js.SetButton(13, (buttons2&0x0040) > 0)
			js.SetButton(14, (buttons2&0x0020) > 0)
			js.SetButton(15, (buttons2&0x0010) > 0)
		}
		js.SetAxis(0, int(limit1(angle)))
		js.SetAxis(5, int(limit1(angle)))
		js.SetAxis(1, cfg[0].Convert(a))
		js.SetAxis(2, cfg[1].Convert(b))
		js.SendState()
	}
}

type ADCConfig struct {
	Min int
	Max int
}

func (c ADCConfig) Convert(x uint16) int {
	ret := 32767 * (int(x) - c.Min) / (c.Max - c.Min)
	if ret < 0 {
		ret = 0
	}
	if 32767 < ret {
		ret = 32767
	}
	return ret
}
