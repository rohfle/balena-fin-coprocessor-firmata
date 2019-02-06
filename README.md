# BalenaFin Co-Processor Firmata

*Please note that in it's current state, this **does not build** due to a number of wrappers still to be implemented*

This is an implementation of the Firmata firmware for SiliconLabs BGM111. It is compatible with Firmata 2.5.8.

- [x] Target and test async DMA UART communication between Coprocessor and CM3
- [x] Create base balena library for wrapping BGM111 complexity
- [x] Verify compatibility of Firmata Marshaller and Parser Libs
- [x] Finish wrapper for Arduino-style *Serial* Library
- [ ] Add custom firmata handlers for CM3 Power Rails, etc.
- [x] Add BGM111 pin mappings/create config file?
- [ ] Document Flashing/Firmware Modification Proceedure

### Firmata Protocol Support

| type                  | command | MIDI channel | first byte          | second byte     | currently supported? |
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

| type                  | sub-command | first byte       | second byte   | ...            | currently supported? |
| --------------------- | -------     | ---------------  | ------------- | -------------- | -------------------- |
| string                | 0x71        | char *string ... |               |                |          ✅          |
| firmware name/version | 0x79        | major version    | minor version | char *name ... |          ✅          |
