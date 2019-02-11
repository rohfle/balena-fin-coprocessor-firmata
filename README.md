# BalenaFin Co-Processor Firmata (beta)

This is an implementation of the [Firmata](https://github.com/firmata/protocol) protocol for SiliconLabs BGM111. It is compatible with standard Firmata 2.5.8. Please be aware this **project is in beta release** and is subject to change frequently.

### Installation

#### Dockerfile

The easiest way to install the Firmata application onto your board is to run the [balena application]() provided. This targets the latest verision of the Firmata application. This balena application will run and install [OpenOCD](http://openocd.org/) on your Fin in order to provision the Coprocessor with both a bootloader and the Firmata application.

#### Build & Manually Flash

It is also possible to build the source and manually flash the Coprocessor. At this time, it is recommended that you download and install [Simplicity Studio](https://www.silabs.com/products/development-tools/software/simplicity-studio) in order to build and compile the source. Import this repository into Simplicity and build (the part and board type should already be set). Please use the included Gecko SDK in this repo as it includes additional libraries beyond the standard SDK.

### Firmata Protocol v2.5.8

| type                  | command | MIDI channel | first byte          | second byte     | support              |
| --------------------- | ------- | ------------ | ------------------- | --------------- | -------------------- |
| analog I/O message    | 0xE0    | pin #        | LSB(bits 0-6)       | MSB(bits 7-13)  |          ✅          |
| digital I/O message   | 0x90    | port         | LSB(bits 0-6)       | MSB(bits 7-13)  |          ✅          |
| report analog pin     | 0xC0    | pin #        | disable/enable(0/1) | - n/a -         |          ✅          |
| report digital port   | 0xD0    | port         | disable/enable(0/1) | - n/a -         |          ✅          |
|                       |         |              |                     |                 |                      |
| start sysex           | 0xF0    |              |                     |                 |          ✅          |
| set pin mode(I/O)     | 0xF4    |              | pin # (0-127)       | pin mode        |          ✅          |
| set digital pin value | 0xF5    |              | pin # (0-127)       | pin value(0/1)  |          ✅          |
| sysex end             | 0xF7    |              |                     |                 |          ✅          |
| protocol version      | 0xF9    |              | major version       | minor version   |          ✅          |
| system reset          | 0xFF    |              |                     |                 |          ✅          |

Sysex-based sub-commands (0x00 - 0x7F) are used for an extended command set.

| type                  | sub-command | first byte       | second byte   | ...            | support             |
| --------------------- | -------     | ---------------  | ------------- | -------------- | --------------------|
| string                | 0x71        | char *string ... |               |                |          ✅          |
| firmware name/version | 0x79        | major version    | minor version | char *name ... |          ✅          |

### Planned Features

- [ ] I2C Support
- [ ] Support for RTC control of Compute Module Power Rails
- [ ] SPI Support
- [ ] Custom Client Library for balenaFin features
