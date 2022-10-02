package main

import (
	"machine"

	"tinygo.org/x/drivers/sgp30"

	"time"
)

func main() {
	time.Sleep(5000 * time.Millisecond)

	machine.I2C0.Configure(machine.I2CConfig{
		Frequency: 400000,
	})
	sensor := sgp30.New(machine.I2C0)
	connected := sensor.Connected()
	if !connected {
		println("SGP30 device not found")
		return
	}
	println("SGP30 device found")
	sensor.Configure()

	for {
		if !sensor.IAQMeasure() {
			println("Measurement failed")
			continue
		}

		println("TVOC:", sensor.GetTVOC())
		println("eCO2:", sensor.GetECO2())

		if !sensor.IAQMeasureRaw() {
			println("Raw measurement failed")
			continue
		}

		println("Raw H2:", sensor.GetRawH2())
		println("Raw Ethanol:", sensor.GetRawEthanol())

		time.Sleep(1000 * time.Millisecond)
	}
}
