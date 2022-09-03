package sgp30

// Constants/addresses used for I2C.

// The I2C address which this device listens to.
const (
	Address = 0x58

	WHO_AM_I        = 0x3682
	FEATURESET      = 0x0020
	CRC8_POLYNOMIAL = 0x31
	CRC8_INIT       = 0xFF
	WORD_LEN        = 2
)
