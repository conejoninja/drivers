// Package max30102 provides a driver for the MAX30102 heart rate sensor
//
// Datasheet:
// https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf
// Based on the Sparkfun MAX3010x library: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
package max30102 // import "tinygo.org/x/drivers/max30102"

import (
	"machine"
	"time"
)

type Mode uint8

//const BUFFER_LENGTH = 32 // Arduino UNO
const BUFFER_LENGTH = 64 // SAMD21
const STORAGE_SIZE = 4

// Device wraps an I2C connection to a MAX30102 device.
type Device struct {
	bus      machine.I2C
	Address  uint16
	data     []uint8
	fifo     []uint8
	fifoRed  [STORAGE_SIZE]uint32
	fifoIR   [STORAGE_SIZE]uint32
	fifoHead int8
	fifoTail int8
	readPtr  byte
	writePtr byte
}

// New creates a new MAX30102 connection. The I2C bus must already be
// configured.
//
// This function only creates the Device object, it does not touch the device.
func New(bus machine.I2C) Device {
	return Device{
		bus:     bus,
		Address: Address,
	}
}

// Configure sets up the device for communication
func (d *Device) Configure() {
	d.data = make([]uint8, 1)
	d.fifo = make([]uint8, 6)
	d.bus.WriteRegister(uint8(d.Address), INTERRUPT_ENABLE_1, []byte{0XC0})
	d.bus.WriteRegister(uint8(d.Address), INTERRUPT_ENABLE_2, []byte{0X00})
	// Clear FIFO
	d.bus.WriteRegister(uint8(d.Address), FIFO_WRITE, []byte{0X00})
	d.bus.WriteRegister(uint8(d.Address), FIFO_OVERFLOW, []byte{0X00})
	d.bus.WriteRegister(uint8(d.Address), FIFO_READ, []byte{0X00})

	d.bus.WriteRegister(uint8(d.Address), FIFO_CFG, []byte{0X4F}) // 4 samples per avg + rollover disabled + 17 unread samples
	d.bus.WriteRegister(uint8(d.Address), MODE_CFG, []byte{0X03}) // SPO2 mode
	d.bus.WriteRegister(uint8(d.Address), SPO2_CFG, []byte{0X27}) // (LSB 15.63, full scale 4096) + 100 samples/second + pulse width 411 µs / 18bit resolution

	d.bus.WriteRegister(uint8(d.Address), LED1_PA, []byte{0X24}) // 7.0mA
	d.bus.WriteRegister(uint8(d.Address), LED2_PA, []byte{0X24}) // 7.0mA

	//d.bus.WriteRegister(uint8(d.Address), PILOT_PA, []byte{0X7F}) // 25mA
}

// ReadTemperature returns the temperature in celsius milli degrees (ºC/1000).
func (d *Device) ReadTemperature() (temperature int32, err error) {
	d.bus.WriteRegister(uint8(d.Address), DIE_TEMP_CFG, []byte{0X01})

	response := make([]uint8, 1)
	for i := 0; i < 100; i++ {
		err = d.bus.ReadRegister(uint8(d.Address), INTERRUPT_STATUS_2, response)
		if err != nil {
			return 0, err
		}
		if response[0]&DIE_TEMP_RDY_ENABLE > 0 {
			break
		}
		time.Sleep(1 * time.Millisecond)
	}

	err = d.bus.ReadRegister(uint8(d.Address), DIE_TEMP_INTEGER, response)
	if err != nil {
		return 0, err
	}
	temperature = 10000 * int32(response[0])
	err = d.bus.ReadRegister(uint8(d.Address), DIE_TEMP_FRACTION, response)
	if err != nil {
		return 0, err
	}
	// From "Table 5. SpO2 ADC Range Control"
	temperature += int32(response[0]) * 625
	return temperature / 10, nil
}

func (d *Device) ReadRedLED() (value uint32) {
	return d.fifoRed[d.fifoHead]
}

func (d *Device) ReadIRLED() (value uint32) {
	return d.fifoIR[d.fifoHead]
}

func (d *Device) FIFOAvailable() int8 {
	return (STORAGE_SIZE + d.fifoHead - d.fifoTail) % STORAGE_SIZE
}

func (d *Device) ReadFIFORed() (value uint32) {
	return d.fifoRed[d.fifoTail]
}

func (d *Device) ReadFIFOIR() (value uint32) {
	return d.fifoIR[d.fifoTail]
}

func (d *Device) NextSample() {
	if d.FIFOAvailable() > 0 {
		d.fifoTail++
		d.fifoTail = d.fifoTail % STORAGE_SIZE
	}
}

func (d *Device) ReadPartID() (uint8, error) {
	err := d.bus.ReadRegister(uint8(d.Address), PART_ID, d.data)
	if err != nil {
		return 0, err
	}
	return d.data[0], nil
}

func (d *Device) ReadRevisionID() (uint8, error) {
	err := d.bus.ReadRegister(uint8(d.Address), REVISION_ID, d.data)
	if err != nil {
		return 0, err
	}
	return d.data[0], nil
}

func (d *Device) SafeCheck() bool {
	for i := 0; i < 200; i++ {
		if d.Check() > 0 {
			return true
		}
		time.Sleep(1 * time.Millisecond)
	}
	return false
}

func (d *Device) Check() int16 {
	// Read FIFO data
	n := int16(0)
	d.bus.ReadRegister(uint8(d.Address), FIFO_WRITE, d.data)
	d.readPtr = d.data[0]
	d.bus.ReadRegister(uint8(d.Address), FIFO_READ, d.data)
	d.writePtr = d.data[0]

	//if d.readPtr != d.writePtr {
	n = int16(d.writePtr) - int16(d.readPtr)
	if n < 0 { // wrap condition
		n += 32
	}
	// number of active leds (2) * 3 bytes each
	//for n > 0 {
	d.bus.ReadRegister(uint8(d.Address), FIFO_DATA, d.fifo)
	d.fifoHead++
	d.fifoHead = d.fifoHead % STORAGE_SIZE

	d.fifoRed[d.fifoHead] = uint32(d.fifo[0]&0x3)<<16 | uint32(d.fifo[1])<<8 | uint32(d.fifo[2])
	d.fifoIR[d.fifoHead] = uint32(d.fifo[3]&0x3)<<16 | uint32(d.fifo[4])<<8 | uint32(d.fifo[5])
	n--
	//}
	//}
	//println("FIFO", d.readPtr, d.writePtr, n, d.fifoHead, d.fifoRed[0], d.fifo[0], d.fifo[1], d.fifo[2], d.fifo[3], d.fifo[4], d.fifo[5])

	return n
}
