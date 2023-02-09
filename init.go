//go:build sam

package main

import (
	"device/sam"
)

func initx() {
	sam.CMCC.CTRL.SetBits(sam.CMCC_CTRL_CEN)
}
