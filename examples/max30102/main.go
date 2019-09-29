package main

import (
	"time"

	"machine"

	"tinygo.org/x/drivers/max30102"
)

func main() {
	machine.I2C0.Configure(machine.I2CConfig{})
	sensor := max30102.New(machine.I2C0)
	sensor.Configure()

	for {
		sensor.Check()
		sensor.NextSample()

		temp, _ := sensor.ReadTemperature()
		println("Temperature:", float32(temp)/1000, "ºC")

		ledvalue := sensor.ReadFIFORed()
		println("Red LED:", float32(ledvalue))

		ledvalue = sensor.ReadFIFOIR()
		println("IR LED:", float32(ledvalue))

		time.Sleep(200 * time.Millisecond)
	}
}
