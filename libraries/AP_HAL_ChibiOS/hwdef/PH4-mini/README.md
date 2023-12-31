# Pixhawk4-Mini Flight Controller

The Pixhawk4-Mini flight controller is sold by [Holybro](http://www.holybro.com/product/64)

## Features

 - STM32F765 microcontroller
 - Two IMUs: ICM20689 and BMI055
 - MS5611 SPI barometer
 - builtin I2C IST8310 magnetometer
 - microSD card slot
 - 5 UARTs plus USB
 - 8 PWM outputs
 - Two I2C and one CAN ports
 - External Buzzer
 - external safety Switch
 - dedicated power input port for external power bricks

## Pinout

![Pixhawk4 Mini Board](PH4-mini-pinout.jpg "Pixhawk4 Mini")

## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART2 (Telem1)
 - SERIAL2 -> UART4 (Telem2)
 - SERIAL3 -> UART1 (GPS)
 - SERIAL4 -> UART6 (RCIN port)
 - SERIAL5 -> UART7 (debug)

The Telem1 port has RTS/CTS pins, the other UARTs do not have RTS/CTS.

The RCIN port can be used as RX or TX as a general UART using the
SERIAL5_OPTIONS bits to swap pins. It is not used for RC input (the
PPM pin is used for RC input)

The UART7 connector is inside the case and labelled as debug, but is
available as a general purpose UART with ArduPilot.

## RC Input
 
RC input is configured on the port marked PPM. This connector supports
all RC protocols (including SBUS, DSM, ST24, SRXL and PPM). The RCIN
port is not used for RC input.

## PWM Output

The Pixhawk4-Mini supports up to 11 PWM outputs. All 11 outputs
support all normal PWM output formats. All outputs except numbers 7
and 8 support DShot.

The first 8 outputs are labelled "MAIN OUT" on the case. The next 3
outputs are labelled CAP1 to CAP3 on the case. The CAP4 pin cannot be
used as a PWM output.

In order to use outputs 9, 10 and 11 you need to change BRD_PWM_COUNT
to 11.

The 11 PWM outputs are in 4 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5 and 6 in group2
 - PWM 7 and 8 in group3
 - PWM 9, 10 and 11 in group4

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a dedicated power monitor port with a 6 pin
connector. The correct battery setting parameters are dependent on
the type of power brick which is connected.

## Compass

The Pixhawk4-Mini has a builtin IST8310 compass. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 11 PWM ports plus the CAP4 port can be used as GPIOs (relays,
buttons, RPM etc). To use them you need to limit the number of these
pins that is used for PWM by setting the BRD_PWM_COUNT to a number
less than 11. For example if you set BRD_PWM_COUNT to 8 then CAP1,
CAP2 and CAP3 will be available for use as GPIOs.

The numbering of the GPIOs for PIN variables in ArduPilot is:

 - PWM1 50
 - PWM2 51
 - PWM3 52
 - PWM4 53
 - PWM5 54
 - PWM6 55
 - PWM7 56
 - PWM8 57

In addition the 4 pins on the servo rail marked CAP1 to CAP4 can be
used as GPIOs. These are available as GPIOs in ArduPilot using the
following GPIO numbers:

 - CAP1 58
 - CAP2 59
 - CAP3 60
 - CAP4 61

## Analog inputs

The Pixhawk4-Mini has 4 analog inputs

 - ADC Pin0 -> Battery Voltage
 - ADC Pin1 -> Battery Current Sensor
 - ADC Pin10 -> ADC 5V Sense
 - ADC Pin11 -> ADC 3.3V Sense

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.

## Acknowledgements

Thanks to
[PX4](https://docs.px4.io/en/flight_controller/pixhawk4_mini.html) for
images used under the [CC-BY 4.0 license](https://creativecommons.org/licenses/by/4.0/)
