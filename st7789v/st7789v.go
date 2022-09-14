// Package st7789 implements a driver for the ST7789V TFT displays, it comes in various screen sizes.
//
// Datasheets: https://cdn-shop.adafruit.com/product-files/3787/3787_tft_QT154H2201__________20190228182902.pdf
//             http://www.newhavendisplay.com/appnotes/datasheets/LCDs/ST7789V.pdf
//
package st7789v // import "tinygo.org/x/drivers/st7789v"

import (
	"device"
	"device/rp"
	"image/color"
	"machine"
	"time"
)

// Rotation controls the rotation used by the display.
type Rotation uint8

type MadCtl uint8

// Device wraps an SPI connection.
type Device struct {
	csPin       machine.Pin
	dcPin       machine.Pin
	wrSckPin    machine.Pin
	rdSckPin    machine.Pin
	dataPins    [8]machine.Pin
	blPin       machine.Pin
	width       int16
	height      int16
	rotation    Rotation
	batchLength int32
	isBGR       bool
	buffer      []uint8
}

// Config is the configuration for the display
type Config struct {
	Width    int16
	Height   int16
	Rotation Rotation
}

// New creates a new ST7789V connection. The SPI wire must already be configured.
func New(csPin, dcPin, wrSckPin, rdSckPin, blPin machine.Pin, dataPins [8]machine.Pin) Device {
	csPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	dcPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	wrSckPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	rdSckPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	for i := 0; i < 8; i++ {
		dataPins[i].Configure(machine.PinConfig{Mode: machine.PinOutput})
	}
	blPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	return Device{
		csPin:    csPin,
		dcPin:    dcPin,
		wrSckPin: wrSckPin,
		rdSckPin: rdSckPin,
		dataPins: dataPins,
		blPin:    blPin,
	}
}

// Configure initializes the display with default configuration
func (d *Device) Configure(cfg Config) {
	if cfg.Width != 0 {
		d.width = cfg.Width
	} else {
		d.width = 320
	}
	if cfg.Height != 0 {
		d.height = cfg.Height
	} else {
		d.height = 240
	}

	d.buffer = make([]uint8, d.width*d.height*2)
	d.rotation = cfg.Rotation

	d.rdSckPin.High()

	d.batchLength = int32(d.width)
	if d.height > d.width {
		d.batchLength = int32(d.height)
	}
	d.batchLength += d.batchLength & 1

	// Common initialization
	d.Command(SWRESET)
	time.Sleep(150 * time.Millisecond)

	d.Command(TEON)
	d.CommandWithData(COLMOD, []byte{0x05})

	d.CommandWithData(PORCTRL, []byte{0x0c, 0x0c, 0x00, 0x33, 0x33})
	d.CommandWithData(LCMCTRL, []byte{0x2c})
	d.CommandWithData(VDVVRHEN, []byte{0x01})
	d.CommandWithData(VRHS, []byte{0x12})
	d.CommandWithData(VDVS, []byte{0x20})
	d.CommandWithData(PWCTRL1, []byte{0xa4, 0xa1})
	d.CommandWithData(FRCTRL2, []byte{0x0f})

	// specific for 320x240 displays
	d.CommandWithData(GCTRL, []byte{0x35})
	d.CommandWithData(VCOMS, []byte{0x1f})
	d.CommandWithData(0xd6, []byte{0xa1})
	d.CommandWithData(GMCTRP1, []byte{0xd0, 0x08, 0x11, 0x08, 0x0c, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2d})
	d.CommandWithData(GMCTRN1, []byte{0xd0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0b, 0x16, 0x14, 0x2f, 0x31})

	d.Command(INVON)
	d.Command(SLPOUT)
	d.Command(DISPON)

	time.Sleep(100 * time.Millisecond)

	// specific for 320x240 displays
	d.CommandWithData(CASET, []byte{uint8(0 >> 8), uint8(0), uint8((d.width - 1) >> 8), uint8(d.width - 1)})
	d.CommandWithData(RASET, []byte{uint8(0 >> 8), uint8(0), uint8((d.height - 1) >> 8), uint8(d.height - 1)})
	d.CommandWithData(MADCTL, []byte{uint8(COL_ORDER | SWAP_XY | SCAN_ORDER)})

	d.blPin.High() // Backlight ON
}

// Display updates the display
func (d *Device) Display() {
	d.dcPin.Low()
	d.csPin.Low()

	d.Tx([]byte{RAMWR})

	d.dcPin.Low()
	d.Tx(d.buffer)

	d.csPin.High()
}

// SetPixel sets a pixel in the screen
func (d *Device) SetPixel(x int16, y int16, c color.RGBA) {
	if x < 0 || y < 0 ||
		(((d.rotation == NO_ROTATION || d.rotation == ROTATION_180) && (x >= d.width || y >= d.height)) ||
			((d.rotation == ROTATION_90 || d.rotation == ROTATION_270) && (x >= d.height || y >= d.width))) {
		return
	}
	//d.FillRectangle(x, y, 1, 1, c)
	cint := RGBATo565(c)
	d.buffer[2*(x+y*d.width)] = uint8(cint >> 8)
	d.buffer[2*(x+y*d.width)+1] = uint8(cint)
}

/*
// setWindow prepares the screen to be modified at a given rectangle
func (d *Device) setWindow(x, y, w, h int16) {
	x += d.columnOffset
	y += d.rowOffset
	d.CommandWithData(CASET, []byte{uint8(x >> 8), uint8(x), uint8((x + w - 1) >> 8), uint8(x + w - 1)})
	d.CommandWithData(RASET, []byte{uint8(y >> 8), uint8(y), uint8((y + h - 1) >> 8), uint8(y + h - 1)})
	d.Command(RAMWR)
} */

/*
// FillRectangle fills a rectangle at a given coordinates with a color
func (d *Device) FillRectangle(x, y, width, height int16, c color.RGBA) error {
	k, i := d.Size()
	if x < 0 || y < 0 || width <= 0 || height <= 0 ||
		x >= k || (x+width) > k || y >= i || (y+height) > i {
		return errors.New("rectangle coordinates outside display area")
	}
	d.setWindow(x, y, width, height)
	c565 := RGBATo565(c)
	c1 := uint8(c565 >> 8)
	c2 := uint8(c565)

	data := make([]uint8, d.batchLength*2)
	for i := int32(0); i < d.batchLength; i++ {
		data[i*2] = c1
		data[i*2+1] = c2
	}
	j := int32(width) * int32(height)
	for j > 0 {
		if j >= d.batchLength {
			d.Tx(data, false)
		} else {
			d.Tx(data[:j*2], false)
		}
		j -= d.batchLength
	}
	return nil
}

// FillRectangleWithBuffer fills buffer with a rectangle at a given coordinates.
func (d *Device) FillRectangleWithBuffer(x, y, width, height int16, buffer []color.RGBA) error {
	i, j := d.Size()
	if x < 0 || y < 0 || width <= 0 || height <= 0 ||
		x >= i || (x+width) > i || y >= j || (y+height) > j {
		return errors.New("rectangle coordinates outside display area")
	}
	if int32(width)*int32(height) != int32(len(buffer)) {
		return errors.New("buffer length does not match with rectangle size")
	}
	d.setWindow(x, y, width, height)

	k := int32(width) * int32(height)
	data := make([]uint8, d.batchLength*2)
	offset := int32(0)
	for k > 0 {
		for i := int32(0); i < d.batchLength; i++ {
			if offset+i < int32(len(buffer)) {
				c565 := RGBATo565(buffer[offset+i])
				c1 := uint8(c565 >> 8)
				c2 := uint8(c565)
				data[i*2] = c1
				data[i*2+1] = c2
			}
		}
		if k >= d.batchLength {
			d.Tx(data, false)
		} else {
			d.Tx(data[:k*2], false)
		}
		k -= d.batchLength
		offset += d.batchLength
	}
	return nil
}

// DrawFastVLine draws a vertical line faster than using SetPixel
func (d *Device) DrawFastVLine(x, y0, y1 int16, c color.RGBA) {
	if y0 > y1 {
		y0, y1 = y1, y0
	}
	d.FillRectangle(x, y0, 1, y1-y0+1, c)
}

// DrawFastHLine draws a horizontal line faster than using SetPixel
func (d *Device) DrawFastHLine(x0, x1, y int16, c color.RGBA) {
	if x0 > x1 {
		x0, x1 = x1, x0
	}
	d.FillRectangle(x0, y, x1-x0+1, 1, c)
}

// FillScreen fills the screen with a given color
func (d *Device) FillScreen(c color.RGBA) {
	if d.rotation == NO_ROTATION || d.rotation == ROTATION_180 {
		d.FillRectangle(0, 0, d.width, d.height, c)
	} else {
		d.FillRectangle(0, 0, d.height, d.width, c)
	}
}  */

/*// SetRotation changes the rotation of the device (clock-wise)
func (d *Device) SetRotation(rotation Rotation) {
	madctl := uint8(0)
	switch rotation % 4 {
	case 0:
		madctl = MADCTL_MX | MADCTL_MY
		d.rowOffset = d.rowOffsetCfg
		d.columnOffset = d.columnOffsetCfg
		break
	case 1:
		madctl = MADCTL_MY | MADCTL_MV
		d.rowOffset = d.columnOffsetCfg
		d.columnOffset = d.rowOffsetCfg
		break
	case 2:
		d.rowOffset = 0
		d.columnOffset = 0
		break
	case 3:
		madctl = MADCTL_MX | MADCTL_MV
		d.rowOffset = 0
		d.columnOffset = 0
		break
	}
	if d.isBGR {
		madctl |= MADCTL_BGR
	}
	d.Command(MADCTL)
	d.Data(madctl)
}*/

// Command sends a command to the display.
func (d *Device) Command(command uint8) {
	d.dcPin.Low()
	d.csPin.Low()

	d.Tx([]byte{command})

	d.csPin.High()
}

// CommandWithData sends a command and data to the display.
func (d *Device) CommandWithData(command uint8, data []byte) {
	d.dcPin.Low()
	d.csPin.Low()

	d.Tx([]byte{command})

	d.dcPin.Low()
	d.Tx(data)

	d.csPin.High()
}

// Tx sends data to the display
func (d *Device) Tx(data []byte) {
	// TODO
	l := len(data)
	for i := 0; i < l/2; i++ {
		data32 := uint32(0xffff) //data[i])
		d.wrSckPin.Low()
		rp.SIO.GPIO_OUT_CLR.Set(0xFFFF << uint8(d.dataPins[0]))
		rp.SIO.GPIO_OUT_SET.Set(data32 << uint8(d.dataPins[0]))
		device.Asm(`nop`)
		d.wrSckPin.High()
		device.Asm(`nop`)

	}
}

// Size returns the current size of the display.
func (d *Device) Size() (w, h int16) {
	if d.rotation == NO_ROTATION || d.rotation == ROTATION_180 {
		return d.width, d.height
	}
	return d.height, d.width
}

// EnableBacklight enables or disables the backlight
func (d *Device) EnableBacklight(enable bool) {
	if enable {
		d.blPin.High()
	} else {
		d.blPin.Low()
	}
}

// InvertColors inverts the colors of the screen
func (d *Device) InvertColors(invert bool) {
	if invert {
		d.Command(INVON)
	} else {
		d.Command(INVOFF)
	}
}

// IsBGR changes the color mode (RGB/BGR)
func (d *Device) IsBGR(bgr bool) {
	d.isBGR = bgr
}

/*
// SetScrollArea sets an area to scroll with fixed top and bottom parts of the display.
func (d *Device) SetScrollArea(topFixedArea, bottomFixedArea int16) {
	d.Command(VSCRDEF)
	d.Tx([]uint8{
		uint8(topFixedArea >> 8), uint8(topFixedArea),
		uint8(d.height - topFixedArea - bottomFixedArea>>8), uint8(d.height - topFixedArea - bottomFixedArea),
		uint8(bottomFixedArea >> 8), uint8(bottomFixedArea)},
		false)
}

// SetScroll sets the vertical scroll address of the display.
func (d *Device) SetScroll(line int16) {
	d.Command(VSCRSADD)
	d.Tx([]uint8{uint8(line >> 8), uint8(line)}, false)
}

// StopScroll returns the display to its normal state.
func (d *Device) StopScroll() {
	d.Command(NORON)
}*/

// RGBATo565 converts a color.RGBA to uint16 used in the display
func RGBATo565(c color.RGBA) uint16 {
	r, g, b, _ := c.RGBA()
	return uint16((r & 0xF800) +
		((g & 0xFC00) >> 5) +
		((b & 0xF800) >> 11))
}
