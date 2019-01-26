package main

import (
	"github.com/aykevl/tinygo-drivers/mpu9250"
	"github.com/aykevl/tinygo/src/machine"
	"time"
)

func main() {
	machine.I2C0.Configure(machine.I2CConfig{})
	sensor := mpu9250.New(machine.I2C0)
	sensor.Configure()

	connected := sensor.Connected()
	if !connected {
		println("MPU9250 not detected")
		return
	}
	println("MPU9250 detected")

	for {
		sensor.ReadSensor()
		ax, ay, az := sensor.GetAccelData()
		println("ACCELERATION", ax, ay, az)

		/*rx, ry, rz := sensor.GetGyroData()
		println("ROTATION", rx, ry, rz)

		mx, my, mz := sensor.GetMagData()
		println("MAGNETOMETER", mx, my, mz) */

		time.Sleep(1000 * time.Millisecond)
	}
}
