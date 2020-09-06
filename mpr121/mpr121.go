// Package mpr121 provides a driver for the MPR121 Capacitive Keypad.
//
// Datasheet: https://cdn-shop.adafruit.com/datasheets/MPR121.pdf
//
package mpr121 // import "tinygo.org/x/drivers/mpr121"

import (
	"time"

	"tinygo.org/x/drivers"
)

// Device wraps an I2C connection to a MPR121 device.
type Config struct {
	TouchThreshold   uint8
	ReleaseThreshold uint8
}

// Device wraps an I2C connection to a MPR121 device.
type Device struct {
	bus              drivers.I2C
	Address          uint16
	touchThreshold   uint8
	releaseThreshold uint8
	electrodes       uint8
}

// New creates a new MPR121 connection. The I2C bus must already be
// configured.
//
// This function only creates the Device object, it does not init the device.
// To do that you must call the Configure() method on the Device before using it.
func New(bus drivers.I2C, electrodes uint8) Device {
	if electrodes > 12 {
		electrodes = 12
	}
	return Device{
		bus:        bus,
		Address:    Address,
		electrodes: electrodes,
	}
}

// Configure sets up the device for communication
func (d *Device) Configure(cfg Config) {
	d.touchThreshold = cfg.TouchThreshold
	d.releaseThreshold = cfg.ReleaseThreshold

	d.writeRegister(SOFTRESET, 0x7F)
	time.Sleep(1 * time.Millisecond)
	d.writeRegister(ECR, 0x0)

	data := []byte{0x0}
	d.bus.ReadRegister(uint8(d.Address), CONFIG2, data)
	if data[0] != 0x24 {
		return
	}

	d.SetThresholds(d.touchThreshold, d.releaseThreshold)

	d.writeRegister(MHDR, 0x01)
	d.writeRegister(NHDR, 0x01)
	d.writeRegister(NCLR, 0x0E)
	d.writeRegister(FDLR, 0x00)

	d.writeRegister(MHDF, 0x01)
	d.writeRegister(NHDF, 0x05)
	d.writeRegister(NCLF, 0x01)
	d.writeRegister(FDLF, 0x00)

	d.writeRegister(NHDT, 0x00)
	d.writeRegister(NCLT, 0x00)
	d.writeRegister(FDLT, 0x00)

	d.writeRegister(DEBOUNCE, 0x00)
	d.writeRegister(CONFIG1, 0x10)
	d.writeRegister(CONFIG2, 0x20)

	/* START AUTO CONFIG */
	d.writeRegister(AUTOCONFIG0, 0x0B)
	// correct values for 3.3v
	d.writeRegister(UPLIMIT, 200)     // ((vdd - 0.7) / vdd) * 256
	d.writeRegister(TARGETLIMIT, 180) // UPLIMIT * 0.9
	d.writeRegister(LOWLIMIT, 130)    // UPLIMIT * 0.65
	/* END AUTO CONFIG */

	d.writeRegister(ECR, 128+d.electrodes) // 128 + #electrodes running
}

func (d *Device) SetThresholds(touch, release uint8) {
	d.writeRegister(ECR, 0x0)

	for i := uint8(0); i < 12; i++ {
		d.writeRegister(TOUCHTH_0+2*i, touch)
		d.writeRegister(RELEASETH_0+2*i, release)
	}

	d.writeRegister(ECR, 0x8F)
}

func (d *Device) ReadFilteredData(channel uint8) uint16 {
	if channel > 12 {
		return 0
	}
	data := []byte{0, 0}
	d.bus.ReadRegister(uint8(d.Address), FILTDATA_0L+channel*2, data)
	return (uint16(data[0]) << 8) | uint16(data[1])
}

func (d *Device) ReadBaselineData(channel uint8) uint8 {
	if channel > 12 {
		return 0
	}
	data := []byte{0}
	d.bus.ReadRegister(uint8(d.Address), BASELINE_0+channel, data)
	return data[0]
}

func (d *Device) IsTouched() uint16 {
	data := []byte{0, 0}
	d.bus.ReadRegister(uint8(d.Address), TOUCHSTATUS_L, data)
	return ((uint16(data[0]) << 8) | uint16(data[1])) & 0x0FF
}

func (d *Device) writeRegister(register, value uint8) {
	if register == ECR || (register >= 0x73 && register <= 0x7A) {
		d.bus.WriteRegister(uint8(d.Address), register, []byte{value})
	} else {
		ecrByte := []byte{0x0}
		d.bus.ReadRegister(uint8(d.Address), ECR, ecrByte)

		d.bus.WriteRegister(uint8(d.Address), ECR, []byte{0x00})
		d.bus.WriteRegister(uint8(d.Address), register, []byte{value})
		d.bus.WriteRegister(uint8(d.Address), ECR, ecrByte)
	}
}
