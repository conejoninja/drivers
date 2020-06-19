package huskylens

// Constants/addresses used for I2C.

// The I2C address which this device listens to.
const Address = 0x32

// Registers. Names, addresses and comments copied from the datasheet.
const (
	REQUEST                = 0x20
	REQUEST_BLOCKS         = 0x21
	REQUEST_ARROWS         = 0x22
	REQUEST_LEARNED        = 0x23
	REQUEST_BLOCKS_LEARNED = 0x24
	REQUEST_ARROWS_LEARNED = 0x25
	REQUEST_BY_ID          = 0x26
	REQUEST_BLOCKS_BY_ID   = 0x27
	REQUEST_ARROWS_BY_ID   = 0x28

	RETURN_INFO  = 0x29
	RETURN_BLOCK = 0x30
	RETURN_ARROW = 0x31

	REQUEST_KNOCK     = 0x32
	REQUEST_ALGORITHM = 0x33

	RETURN_OK = 0x34

	REQUEST_LEARN  = 0x35
	REQUEST_FORGET = 0x36

	REQUEST_SENSOR = 0x37
)
