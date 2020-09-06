package mpr121

const Address = 0x5A
const Address3Vo = 0x5B
const AddressSDA = 0x5C
const AddressSCL = 0x5D

const (
	TOUCHSTATUS_L = 0x00
	TOUCHSTATUS_H = 0x01
	FILTDATA_0L   = 0x04
	FILTDATA_0H   = 0x05
	BASELINE_0    = 0x1E
	MHDR          = 0x2B
	NHDR          = 0x2C
	NCLR          = 0x2D
	FDLR          = 0x2E
	MHDF          = 0x2F
	NHDF          = 0x30
	NCLF          = 0x31
	FDLF          = 0x32
	NHDT          = 0x33
	NCLT          = 0x34
	FDLT          = 0x35

	TOUCHTH_0    = 0x41
	RELEASETH_0  = 0x42
	DEBOUNCE     = 0x5B
	CONFIG1      = 0x5C
	CONFIG2      = 0x5D
	CHARGECURR_0 = 0x5F
	CHARGETIME_1 = 0x6C
	ECR          = 0x5E
	AUTOCONFIG0  = 0x7B
	AUTOCONFIG1  = 0x7C
	UPLIMIT      = 0x7D
	LOWLIMIT     = 0x7E
	TARGETLIMIT  = 0x7F

	GPIODIR    = 0x76
	GPIOEN     = 0x77
	GPIOSET    = 0x78
	GPIOCLR    = 0x79
	GPIOTOGGLE = 0x7A

	SOFTRESET = 0x80
)
