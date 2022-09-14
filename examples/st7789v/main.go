package main

import (
	"machine"

	"tinygo.org/x/drivers/st7789v"

	"image/color"
)

func main() {
	//csPin, dcPin, wrSckPin, rdSckPin, blPin machine.Pin, dataPins [8]machine.Pin
	display := st7789v.New(machine.LCD_CS,
		machine.LCD_DC,
		machine.LCD_WR,
		machine.LCD_RD,
		machine.LCD_BACKLIGHT,
		[8]machine.Pin{machine.LCD_DB0, machine.LCD_DB1, machine.LCD_DB2, machine.LCD_DB3, machine.LCD_DB4, machine.LCD_DB5, machine.LCD_DB6, machine.LCD_DB7},
	)

	display.Configure(st7789v.Config{})

	//width, height := display.Size()

	white := color.RGBA{255, 255, 255, 255}
	red := color.RGBA{255, 0, 0, 255}
	blue := color.RGBA{0, 0, 255, 255}
	green := color.RGBA{0, 255, 0, 255}
	black := color.RGBA{0, 0, 0, 255}

	/*display.FillScreen(black)

	display.FillRectangle(0, 0, width/2, height/2, white)
	display.FillRectangle(width/2, 0, width/2, height/2, red)
	display.FillRectangle(0, height/2, width/2, height/2, green)
	display.FillRectangle(width/2, height/2, width/2, height/2, blue)
	display.FillRectangle(width/4, height/4, width/2, height/2, black)*/
	display.SetPixel(0, 0, white)
	display.SetPixel(0, 1, red)
	display.SetPixel(0, 2, blue)
	display.SetPixel(0, 3, green)
	display.SetPixel(0, 4, black)
}
