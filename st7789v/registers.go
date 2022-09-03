package st7789v

// Registers
const (
	SWRESET  = 0x01
	TEOFF    = 0x34
	TEON     = 0x35
	MADCTL   = 0x36
	COLMOD   = 0x3A
	GCTRL    = 0xB7
	VCOMS    = 0xBB
	LCMCTRL  = 0xC0
	VDVVRHEN = 0xC2
	VRHS     = 0xC3
	VDVS     = 0xC4
	FRCTRL2  = 0xC6
	PWCTRL1  = 0xD0
	PORCTRL  = 0xB2
	GMCTRP1  = 0xE0
	GMCTRN1  = 0xE1
	INVOFF   = 0x20
	SLPOUT   = 0x11
	DISPON   = 0x29
	GAMSET   = 0x26
	DISPOFF  = 0x28
	RAMWR    = 0x2C
	INVON    = 0x21
	CASET    = 0x2A
	RASET    = 0x2B
	PWMFRSEL = 0xCC

	ROW_ORDER   MadCtl = 0b10000000 // 0x80
	COL_ORDER   MadCtl = 0b01000000 // 0x40
	SWAP_XY     MadCtl = 0b00100000 // 0x20
	SCAN_ORDER  MadCtl = 0b00010000 // 0x10
	RGB         MadCtl = 0b00001000 // 0x08
	HORIZ_ORDER MadCtl = 0b00000100 // 0x04

	NO_ROTATION  Rotation = 0
	ROTATION_90  Rotation = 1 // 90 degrees clock-wise rotation
	ROTATION_180 Rotation = 2
	ROTATION_270 Rotation = 3
)
