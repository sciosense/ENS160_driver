# ScioSense ENS160
Arduino library for the ENS160 digital four channel MOX gas sensor with I2C interface from ScioSense

## Introduction
This project is an Arduino *library*. It implements a driver with examples for the ENS160.
The ENS160 chip is a digital gas sensor for TVOC and eCO2 with an I2C interface.
The driver in this Arduino library is based on the code supplied by *Sciosense*, the manufacturer of the chip.

Note that the ENS160 requires a supply voltage of 1.71V .. 1.98V.
The ENS160 also requires a IO voltage of 1.71V .. 3.6V.

## Links
The ENS160 is made by [Sciosense](http://www.sciosense.com).
 - The datasheet of the ENS160 is not yet released but can be provided on request

## Prerequisites
It is assumed that
 - The Arduino IDE has been installed.
   If not, refer to "Install the Arduino Desktop IDE" on the
   [Arduino site](https://www.arduino.cc/en/Guide/HomePage).
 - The library directory is at its default location.
   For me, Christoph, that is `C:\Users\christoph\Documents\Arduino\libraries`.

## Build an example
To build an example sketch
 - (Re)start Arduino.
 - Open File > Example > Examples from Custom Libraries > ENS160 > ENS160basic
 - Make sure Tools > Board lists the correct board.
 - Select Sketch > Verify/Compile.