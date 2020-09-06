// Connects to an MPR121 Capacitive Keypad.
package main

import (
	"machine"
	"time"

	"tinygo.org/x/drivers/mpr121"
)

func main() {
	machine.I2C0.Configure(machine.I2CConfig{})

	keypad := mpr121.New(machine.I2C0, 12)
	keypad.Address = mpr121.Address3Vo
	keypad.Configure(mpr121.Config{
		TouchThreshold:   12,
		ReleaseThreshold: 6,
	})

	var touched uint16
	var i uint8

	for {
		touched = keypad.IsTouched()
		println(touched)

		for i=0;i<12;i++ {
			print(i, keypad.ReadFilteredData(i), " / ")
		}
		println("---")
		for i=0;i<12;i++ {
			print(i, keypad.ReadBaselineData(i), " / ")
		}
		println("--------------")

		time.Sleep(200 * time.Millisecond)
	}
}
