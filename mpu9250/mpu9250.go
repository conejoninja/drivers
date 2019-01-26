// Package mpu9250 provides a driver for the MPU9250 accelerometer, gyroscope and
// magnetometer made by InvenSense.
//
// Datasheets:
// https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
// https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
package mpu9250

import (
	"errors"
	"time"

	"math"

	"github.com/aykevl/tinygo/src/machine"
)

type FIFO struct {
	accel, gyro, mag, temp     bool
	ax, ay, az                 [85]float32
	gx, gy, gz                 [85]float32
	hx, hy, hz                 [73]float32
	t                          [256]float32
	size, frameSize            uint
	aSize, gSize, hSize, tSize uint
}

type Values struct {
	ax, ay, az float32
	gx, gy, gz float32
	hx, hy, hz float32
	t          float32
}

// Device wraps an I2C connection to a MPU9250 device.
type Device struct {
	bus               machine.I2C
	accelScale        float32
	accelRange        uint8
	gyroScale         float32
	gyroRange         uint8
	bandwidth         uint8
	sampleRateDivider uint8
	magXAdjust        float32
	magYAdjust        float32
	magZAdjust        float32
	fifo              FIFO
	ax, ay, az        int32
	gx, gy, gz        int32
	hx, hy, hz        int32
	t                 int32
	bias              Values
	scaleFactor       Values
	buffer            [21]uint8
}

// New creates a new MPU9250 connection. The I2C bus must already be
// configured.
//
// This function only creates the Device object, it does not touch the device.
func New(bus machine.I2C) Device {
	return Device{
		bus: bus,
	}
}

// Connected returns whether a MPU9250 has been found.
// It does a "who am I" request and checks the response.
func (d *Device) Connected() bool {
	data := []byte{0}
	d.bus.ReadRegister(MPU9250_Address, WHO_AM_I, data)
	return data[0] == WHO_AM_I_RESPONSE
}

// Configure sets up the device for communication.
func (d *Device) Configure() {
	//sleep off
	d.writeRegMPU(PWR_MGMT_1, 0x00)
	time.Sleep(100 * time.Millisecond)

	// select best available clock source
	d.writeRegMPU(PWR_MGMT_1, 0x01)
	time.Sleep(200 * time.Millisecond)

	// Enable I2C master mode
	d.writeRegMPU(USER_CTRL, 0x20)

	// Set I2C bus speed to 400 kHz
	d.writeRegMPU(I2C_MST_CTRL, 0x0D)

	// Set AK8963 to Power Down
	d.MagSetMode(MAG_MODE_POWERDOWN)
	// Reset MPU9250
	d.writeRegMPU(PWR_MGMT_1, 0x80)
	// wait for MPU9250 to come back
	time.Sleep(10 * time.Millisecond)
	// Reset AK8963
	d.writeRegAK(CNTL2, 0x01)
	// Set I2C bus speed to 400 kHz
	d.writeRegMPU(I2C_MST_CTRL, 0x0D)
	// Enable accel and gyro
	d.writeRegMPU(PWR_MGMT_2, 0x00)
	// Set accel range to 16G
	d.writeRegMPU(ACCEL_CONFIG, ACCEL_FS_SEL_16G)
	d.accelScale = G * 16.0 / 32757.5
	d.accelRange = ACCEL_RANGE_16G
	// Set gyro range to 2000DPS
	d.writeRegMPU(GYRO_CONFIG, GYRO_FS_SEL_2000DPS)
	d.gyroScale = 2000.0 / 32757.5 * D2R
	d.gyroRange = GYRO_RANGE_2000DPS
	// Set bandwidth to 184kHz
	d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_184)
	d.writeRegMPU(CONFIG, GYRO_DLPF_184)
	d.bandwidth = DLPF_BANDWIDTH_184HZ
	// Set sample rate divider to 0
	d.writeRegMPU(SMPLRT_DIV, 0x00)
	// Enable I2C master mode
	d.writeRegMPU(USER_CTRL, 0x20)

	// Set I2C bus speed to 400 kHz
	d.writeRegMPU(I2C_MST_CTRL, 0x0D)

	// set AK8963 to Power Down
	d.MagSetMode(MAG_MODE_POWERDOWN)
	time.Sleep(100 * time.Millisecond)
	// set AK8963 to fuse rom access
	d.MagSetMode(MAG_MODE_FUSEROM)
	time.Sleep(100 * time.Millisecond)
	// read adjust values and scale them
	data := d.readAK8963Registers(ASAX, 3)
	d.magXAdjust = (1.0 + (float32(uint8(data[0]))-128.0)/256.0) * 4912.0 / 32760.0
	d.magYAdjust = (1.0 + (float32(uint8(data[1]))-128.0)/256.0) * 4912.0 / 32760.0
	d.magZAdjust = (1.0 + (float32(uint8(data[2]))-128.0)/256.0) * 4912.0 / 32760.0
	// set AK8963 to Power Down
	d.MagSetMode(MAG_MODE_POWERDOWN)
	time.Sleep(100 * time.Millisecond)
	// set AK8963 to 16 bit resolution, 100 Hz rate
	d.MagSetMode(MAG_MODE_CONTINUOUS_100HZ)
	time.Sleep(100 * time.Millisecond)
	// select best available clock source
	d.writeRegMPU(PWR_MGMT_1, 0x01)
	time.Sleep(200 * time.Millisecond)

	d.readAK8963Registers(HXL, 7)
	d.calibrateGyro()
	d.scaleFactor.ax = 1
	d.scaleFactor.ay = 1
	d.scaleFactor.az = 1
	d.scaleFactor.hx = 1
	d.scaleFactor.hy = 1
	d.scaleFactor.hz = 1
}

func (d *Device) MagSetMode(mode uint8) {
	d.writeRegAK(CNTL1, mode)
	time.Sleep(10 * time.Millisecond)
}

func (d *Device) SetAccelRange(accelRange uint8) {
	switch accelRange {
	case ACCEL_RANGE_2G:
		d.writeRegMPU(ACCEL_CONFIG, ACCEL_FS_SEL_2G)
		d.accelScale = G * 2.0 / 32757.5
		break
	case ACCEL_RANGE_4G:
		d.writeRegMPU(ACCEL_CONFIG, ACCEL_FS_SEL_4G)
		d.accelScale = G * 4.0 / 32757.5
		break
	case ACCEL_RANGE_8G:
		d.writeRegMPU(ACCEL_CONFIG, ACCEL_FS_SEL_8G)
		d.accelScale = G * 8.0 / 32757.5
		break
	case ACCEL_RANGE_16G:
		d.writeRegMPU(ACCEL_CONFIG, ACCEL_FS_SEL_16G)
		d.accelScale = G * 16.0 / 32757.5
		break
	}
	d.accelRange = accelRange
}

func (d *Device) SetGyroRange(gyroRange uint8) {
	switch gyroRange {
	case GYRO_RANGE_250DPS:
		d.writeRegMPU(GYRO_CONFIG, GYRO_FS_SEL_250DPS)
		d.gyroScale = 250.0 / 32757.5 * D2R
		break
	case GYRO_RANGE_500DPS:
		d.writeRegMPU(GYRO_CONFIG, GYRO_FS_SEL_500DPS)
		d.gyroScale = 500.0 / 32757.5 * D2R
		break
	case GYRO_RANGE_1000DPS:
		d.writeRegMPU(GYRO_CONFIG, GYRO_FS_SEL_1000DPS)
		d.gyroScale = 1000.0 / 32757.5 * D2R
		break
	case GYRO_RANGE_2000DPS:
		d.writeRegMPU(GYRO_CONFIG, GYRO_FS_SEL_2000DPS)
		d.gyroScale = 2000.0 / 32757.5 * D2R
		break
	}
	d.gyroRange = gyroRange
}

func (d *Device) SetDLPFBandwidth(bandwidth uint8) {
	switch bandwidth {
	case DLPF_BANDWIDTH_184HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_184)
		d.writeRegMPU(CONFIG, GYRO_DLPF_184)
		break
	case DLPF_BANDWIDTH_92HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_92)
		d.writeRegMPU(CONFIG, GYRO_DLPF_92)
		break
	case DLPF_BANDWIDTH_41HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_41)
		d.writeRegMPU(CONFIG, GYRO_DLPF_41)
		break
	case DLPF_BANDWIDTH_20HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_20)
		d.writeRegMPU(CONFIG, GYRO_DLPF_20)
		break
	case DLPF_BANDWIDTH_10HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_10)
		d.writeRegMPU(CONFIG, GYRO_DLPF_10)
		break
	case DLPF_BANDWIDTH_5HZ:
		d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_5)
		d.writeRegMPU(CONFIG, GYRO_DLPF_5)
		break
	}
	d.bandwidth = bandwidth
}

func (d *Device) SetSampleRateDivider(srd uint8) {
	d.writeRegMPU(SMPLRT_DIV, 19)
	d.MagSetMode(MAG_MODE_POWERDOWN)
	time.Sleep(100 * time.Millisecond)
	if srd > 9 {
		d.MagSetMode(MAG_MODE_CONTINUOUS_8HZ)
	} else {
		d.MagSetMode(MAG_MODE_CONTINUOUS_100HZ)
	}
	time.Sleep(100 * time.Millisecond)
	// TODO:
	//d.readMagRegisters()
	d.writeRegMPU(SMPLRT_DIV, srd)
	d.sampleRateDivider = srd
}

func (d *Device) EnableDataReadyInterrup() {
	d.writeRegMPU(INT_PIN_CFG, 0x00) // 50 µs pulse
	d.writeRegMPU(INT_ENABLE, 0x01)
}

func (d *Device) DisableDataReadyInterrup() {
	d.writeRegMPU(INT_ENABLE, 0x00)
}

func (d *Device) EnableWakeOnMotion(womThreshold float32, odr uint8) {
	d.writeRegAK(CNTL1, MAG_MODE_POWERDOWN)
	d.writeRegMPU(PWR_MGMT_1, 0x80) // reset
	time.Sleep(10 * time.Millisecond)
	d.writeRegMPU(PWR_MGMT_1, 0x00)
	d.writeRegMPU(PWR_MGMT_2, 0x07) // disable gyro
	d.writeRegMPU(ACCEL_CONFIG2, ACCEL_DLPF_184)
	d.writeRegMPU(INT_ENABLE, 0x40)           // enable wake on motion
	d.writeRegMPU(MOT_DETECT_CTRL, 0x80|0x40) // enable accel hardware intelligence
	// map womThreshold [0:1020] -> [0:255]
	womByte := uint8(255.0 * womThreshold / 1020.0)
	d.writeRegMPU(WOM_THR, womByte)
	d.writeRegMPU(LP_ACCEL_ODR, odr)
	d.writeRegMPU(PWR_MGMT_1, 0x20) // power cycle
}

func (d *Device) EnableFifo(accel bool, gyro bool, mag bool, temp bool) {
	d.writeRegMPU(USER_CTRL, 0x40|0x20)
	var data byte
	d.fifo.frameSize = 0
	if accel {
		data |= FIFO_ACCEL
		d.fifo.frameSize += 6
	}
	if gyro {
		data |= FIFO_GYRO
		d.fifo.frameSize += 6
	}
	if mag {
		data |= FIFO_MAG
		d.fifo.frameSize += 7
	}
	if temp {
		data |= FIFO_TEMP
		d.fifo.frameSize += 2
	}
	d.writeRegMPU(FIFO_EN, data)
	d.fifo.accel = accel
	d.fifo.gyro = gyro
	d.fifo.mag = mag
	d.fifo.temp = temp
}

func (d *Device) ReadSensor() {
	data := make([]byte, 21)
	// read all sensors data at once
	d.bus.ReadRegister(MPU9250_Address, ACCEL_XOUT_H, data)
	d.ax = int32((uint16(data[0]) << 8) | uint16(data[1]))
	d.ay = int32((uint16(data[2]) << 8) | uint16(data[3]))
	d.az = int32((uint16(data[4]) << 8) | uint16(data[5]))
	d.t = int32((uint16(data[6]) << 8) | uint16(data[7]))
	d.gx = int32((uint16(data[8]) << 8) | uint16(data[9]))
	d.gy = int32((uint16(data[10]) << 8) | uint16(data[11]))
	d.gz = int32((uint16(data[12]) << 8) | uint16(data[13]))
	d.hx = int32((uint16(data[15]) << 8) | uint16(data[14]))
	d.hy = int32((uint16(data[17]) << 8) | uint16(data[16]))
	d.hz = int32((uint16(data[19]) << 8) | uint16(data[18]))
	// transforms to proper values
	/*d.ax = (ay*d.accelScale - d.bias.ax) * d.scaleFactor.ax
	d.ay = (ax*d.accelScale - d.bias.ay) * d.scaleFactor.ay
	d.az = (-az*d.accelScale - d.bias.az) * d.scaleFactor.az
	d.gx = (gy*d.accelScale - d.bias.gx) * d.scaleFactor.gx
	d.gy = (gx*d.accelScale - d.bias.gy) * d.scaleFactor.gy
	d.gz = (-gz*d.accelScale - d.bias.gz) * d.scaleFactor.gz
	d.hx = (hx*d.magXAdjust - d.bias.hx) * d.scaleFactor.hx
	d.hy = (hy*d.magYAdjust - d.bias.hy) * d.scaleFactor.hy
	d.hz = (hz*d.magZAdjust - d.bias.hz) * d.scaleFactor.hz
	d.t = d.bias.t + (t-d.bias.t)/d.scaleFactor.t*/
}

// data in m/s²
func (d *Device) GetAccelData() (x int32, y int32, z int32) {
	return d.ax, d.ay, d.az
}

// data in rad/s
func (d *Device) GetGyroData() (x int32, y int32, z int32) {
	return d.gx, d.gy, d.gz
}

// data in µT (T = Tesla)
func (d *Device) GetMagData() (x int32, y int32, z int32) {
	return d.hx, d.hy, d.hz
}

// data in ºC
func (d *Device) GetTemperature() (temperature int32) {
	return d.t
}

// data in m/s²
func (d *Device) GetAccelBias() (x float32, y float32, z float32) {
	return d.bias.ax, d.bias.ay, d.bias.az
}

// data in rad/s
func (d *Device) GetGyroBias() (x float32, y float32, z float32) {
	return d.bias.gx, d.bias.gy, d.bias.gz
}

// data in µT (T = Tesla)
func (d *Device) GetMagBias() (x float32, y float32, z float32) {
	return d.bias.hx, d.bias.hy, d.bias.hz
}

// data in ºC
func (d *Device) GetTemperatureBias() (temperature float32) {
	return d.bias.t
}

// data in m/s²
func (d *Device) GetAccelScaleFactor() (x float32, y float32, z float32) {
	return d.scaleFactor.ax, d.scaleFactor.ay, d.scaleFactor.az
}

// data in rad/s
func (d *Device) GetGyroScaleFactor() (x float32, y float32, z float32) {
	return d.scaleFactor.gx, d.scaleFactor.gy, d.scaleFactor.gz
}

// data in µT (T = Tesla)
func (d *Device) GetMagScaleFactor() (x float32, y float32, z float32) {
	return d.scaleFactor.hx, d.scaleFactor.hy, d.scaleFactor.hz
}

// data in ºC
func (d *Device) GetTemperatureScaleFactor() (temperature float32) {
	return d.scaleFactor.t
}

func (d *Device) calibrateGyro() {
	d.SetGyroRange(GYRO_RANGE_250DPS)
	d.SetDLPFBandwidth(DLPF_BANDWIDTH_20HZ)
	d.SetSampleRateDivider(19)
	var gx, gy, gz float32
	for i := 0; i < 100; i++ {
		d.ReadSensor()
		x, y, z := d.GetGyroData()
		gx += (float32(x) + d.bias.gx) / 100.0
		gy += (float32(y) + d.bias.gy) / 100.0
		gz += (float32(z) + d.bias.gz) / 100.0
		time.Sleep(20 * time.Millisecond)
	}
	d.bias.gx = gx
	d.bias.gy = gy
	d.bias.gz = gz
	d.SetGyroRange(d.gyroRange)
	d.SetDLPFBandwidth(d.bandwidth)
	d.SetSampleRateDivider(d.sampleRateDivider)
}

func (d *Device) calibrateMag() {
	d.SetSampleRateDivider(19)
	x, y, z := d.GetMagData()
	xmax, ymax, zmax := float32(x), float32(y), float32(z)
	xmin, ymin, zmin := xmax, ymax, zmax
	var xfilt, yfilt, zfilt float32
	coeff := float32(8)
	var counter int16
	for counter < 1000 {
		var delta float32
		var frameDelta float32
		d.ReadSensor()
		xfilt = (xfilt*(coeff-1.0) + (float32(d.hx)/d.scaleFactor.hx + d.bias.hx)) / coeff
		yfilt = (yfilt*(coeff-1.0) + (float32(d.hy)/d.scaleFactor.hy + d.bias.hy)) / coeff
		zfilt = (zfilt*(coeff-1.0) + (float32(d.hz)/d.scaleFactor.hz + d.bias.hz)) / coeff
		if xfilt > xmax {
			delta = xfilt - xmax
			xmax = xfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		if yfilt > ymax {
			delta = yfilt - ymax
			ymax = yfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		if zfilt > zmax {
			delta = zfilt - zmax
			zmax = zfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		if xfilt < xmin {
			delta = float32(math.Abs(float64(xfilt - xmin)))
			xmin = xfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		if yfilt < ymin {
			delta = float32(math.Abs(float64(yfilt - ymin)))
			ymin = yfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		if zfilt < zmin {
			delta = float32(math.Abs(float64(zfilt - zmin)))
			zmin = zfilt
		}
		if delta > frameDelta {
			frameDelta = delta
		}
		println("FRAME DELTA", frameDelta)
		if frameDelta > 0.3 { // deltaThreshold
			counter = 0
		} else {
			counter++
		}
		time.Sleep(20 * time.Millisecond)
	}
	// Set magnetometer bias
	d.bias.hx = (xmax - xmin) / 2.0
	d.bias.hy = (ymax - ymin) / 2.0
	d.bias.hz = (zmax - zmin) / 2.0
	// Set magnetometer scale factor
	avg := (d.bias.hx + d.bias.hy + d.bias.hz) / 3.0
	d.scaleFactor.hx = avg / d.bias.hx
	d.scaleFactor.hy = avg / d.bias.hy
	d.scaleFactor.hz = avg / d.bias.hz
	d.SetSampleRateDivider(d.sampleRateDivider)
}

func (d *Device) calibrateAccel() {
	// TODO
	d.scaleFactor.ax = 1
	d.scaleFactor.ay = 1
	d.scaleFactor.az = 1
}

func (d *Device) SetGyroXBias(bias float32) {
	d.bias.gx = bias
}

func (d *Device) SetGyroYBias(bias float32) {
	d.bias.gy = bias
}

func (d *Device) SetGyroZBias(bias float32) {
	d.bias.gz = bias
}

func (d *Device) SetMagCalX(bias float32, scaleFactor float32) {
	d.bias.hx = bias
	d.scaleFactor.hx = scaleFactor
}

func (d *Device) SetMagCalY(bias float32, scaleFactor float32) {
	d.bias.hy = bias
	d.scaleFactor.hy = scaleFactor
}

func (d *Device) SetMagCalZ(bias float32, scaleFactor float32) {
	d.bias.hz = bias
	d.scaleFactor.hz = scaleFactor
}

func (d *Device) writeRegMPU(reg byte, data byte) error {
	return d.bus.WriteRegister(MPU9250_Address, reg, []byte{data})
}

func (d *Device) writeRegAK(reg byte, data byte) error {
	d.writeRegMPU(I2C_SLV0_ADDR, AK8963_Address)
	d.writeRegMPU(I2C_SLV0_REG, reg)
	d.writeRegMPU(I2C_SLV0_DO, data)
	d.writeRegMPU(I2C_SLV0_CTRL, 0x80|1)
	readData := d.readAK8963Registers(reg, 1)
	if readData[0] != data {
		return errors.New("wrong data at AK8963")
	}
	return nil
}

func (d *Device) readAK8963Registers(address byte, count uint8) []byte {
	data := make([]byte, count)
	d.writeRegMPU(I2C_SLV0_ADDR, AK8963_Address|0x80) //0x80 READ FLAG
	d.writeRegMPU(I2C_SLV0_REG, address)
	d.writeRegMPU(I2C_SLV0_CTRL, 0x80|count)
	time.Sleep(1 * time.Millisecond)
	d.bus.ReadRegister(MPU9250_Address, EXT_SENS_DATA_00, data)
	return data
}
