// Package sgp30 provides a driver for the SGP30 environment sensor  (CO2, H2, VOC, TVOC, Ethanol).
//
// Datasheet:
// https://sensirion.com/media/documents/984E0DD5/61644B8B/Sensirion_Gas_Sensors_Datasheet_SGP30.pdf
//
package sgp30 // import "tinygo.org/x/drivers/sgp30"

import (
	"tinygo.org/x/drivers"
)

// Device wraps an I2C connection to a SHT31 device.
type Device struct {
	bus        drivers.I2C
	Address    uint16
	TVOC       uint8
	eCO2       uint8
	TVOCBase   uint8
	eCO2Base   uint8
	rawEthanol uint8
	rawH2      uint8
}

// New creates a new SGP30 connection. The I2C bus must already be
// configured.
//
// This function only creates the Device object, it does not initialize the device.
// You must call Configure() first in order to use the device itself.
func New(bus drivers.I2C) Device {
	return Device{
		bus:     bus,
		Address: Address,
	}
}

// Connected returns whether a VL53L1X has been found.
// It does a "who am I" request and checks the response.
func (d *Device) Connected() bool {
	_, success := d.readWordFromCommand([]byte{0x36, 0x82}, 3)
	if !success {
		return false
	}

	return true
}

// Configure sets up the device for communication
func (d *Device) Configure(use2v8Mode bool) bool {
	if !d.Connected() {
		return false
	}
	data, success := d.readWordFromCommand([]byte{0x20, 0x2F}, 1)
	if !success {
		return false
	}
	if (data[0] & 0xF0) != FEATURESET {
		return false
	}

	if d.initSensor() {
		if !d.IAQInit() {
			return false
		}
	}

	return true
}

func (d *Device) softReset() bool {
	_, success := d.readWordFromCommand([]byte{0x00, 0x06}, 0)
	return success
}

func (d *Device) IAQInit() bool {
	_, success := d.readWordFromCommand([]byte{0x20, 0x03}, 0)
	return success
}

func (d *Device) IAQMeasure() bool {
	data, success := d.readWordFromCommand([]byte{0x20, 0x08}, 2)
	if !success {
		return false
	}
	d.TVOC = data[0]
	d.eCO2 = data[1]
	return true
}

func (d *Device) IAQMeasureRaw() bool {
	data, success := d.readWordFromCommand([]byte{0x20, 0x50}, 2)
	if !success {
		return false
	}
	d.rawEthanol = data[0]
	d.rawH2 = data[1]
	return true
}

func (d *Device) GetIAQBaseline() bool {
	data, success := d.readWordFromCommand([]byte{0x20, 0x15}, 2)
	if !success {
		return false
	}
	d.eCO2Base = data[0]
	d.TVOCBase = data[1]
	return true
}

func (d *Device) SetIAQBaseline() bool {
	data, success := d.readWordFromCommand([]byte{0x20, 0x15}, 2)
	if !success {
		return false
	}
	d.eCO2Base = data[0]
	d.TVOCBase = data[1]
	return true
}

func (d *Device) readWordFromCommand(cmd []byte, readLength uint8) ([]byte, bool) {

	bufferLength := readLength * (WORD_LEN + 1)
	readData := make([]byte, readLength)
	buffer := make([]byte, bufferLength)
	d.bus.Tx(d.Address, cmd, buffer)

	for i := uint8(0); i < readLength; i++ {
		crc := d.generateCRC(buffer[i*3 : i*3+2])
		if crc != buffer[i*3+2] {
			return readData, false
		}
		readData[i] = buffer[i*3]
		readData[i] <<= 8
		readData[i] |= buffer[i*3+1]
	}

	return readData, true
}

func (d *Device) generateCRC(data []byte) byte {
	crc := uint8(CRC8_INIT)
	crc ^= data[0]
	for b := uint8(0); b < 8; b++ {
		if (crc & 0x80) != 0x00 {
			crc = (crc << 1) ^ CRC8_POLYNOMIAL
		} else {
			crc <<= 1
		}
	}
	return crc
}
