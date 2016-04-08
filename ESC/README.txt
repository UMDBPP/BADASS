This is the documentation for getting the ESC to work.

For preparing the firmware:
	Download simonk firmware: http://github.com/sim-/tgy
	In tgy.asm
		Modify line 188:	Set RC_PULS_REVERSE to 1.
		Modify line 189:	Set RC_CALIBRATION to 0
	Use AVRStudio to assemble the program.
		Copy tgy.asm into a new Atmel Studio project.
			When creating a new project, be sure to select "AVR Assembler" for the project type.
			Select ATmega8 for the type of MCU
		Once you've copied in the code from tgy.asm, right click the project in the Solution Explorer and select "Properties".
			In the toolchain tab, select "General" under "AVR Assembler".
			In "Other optimization flags", enter "-D bs_nfet_esc" without the quotes
		Copy all the .inc files from the tgy directory into the project
		Build the project by selecting "Build">"Build <project name>".
		The output file is in your <project location>/<project name>/Debug. The name of the file should be <project name>.hex
			For example, my project was called tgy and tgy.hex was in Documents/Atmel Studio 7/tgy/tgy/Debug/

For flashing firmware:
	Transfer the generated .hex file to a linux system with avrdude installed.
	Attach ESC to an AVR programmer and that to the system.
	Run: "sudo avrdude -c usbtiny -p m8"
		If this fails with rc=-1, ensure that the ESC is connected to a power source. If the error persists, flip the 6 pin connector on the programmer head.
	Run: "sudo avrdude -c usbtiny -p m8 -U flash:w:tgy.he:i"
		This should run fine. It may throw an abort error at the end. If it does, scroll up and if the error is from it trying to call free(), it has flashed successfully and so ignore the error.

Interface with Arduino
	This is rather simple. Connect the brown wire from the ESC to ground, and the yellow/orange (note this is not the dark orange/red) to the pin defined in the arduino program.
	Interface with the ESC as if it is a servo in the Arduino program.

ESC Calibration
	On poweron, you should hear three increasing beeps and then a fourth. Then calibrate by setting the value to the off position: 1460.
		Lack of a fourth beep means that the ESC is not detecting the input pulse. Try going through all values and try powering off and on again. If this fails, there is another problem, possibly wiht the ESC itselft. We had this error when a resistor failed. If this happens, call Camden.
	After that, it may or may not require going to the max and min values. Calibration is done at this point.
