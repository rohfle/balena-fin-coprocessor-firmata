# BalenaFin Co-Processor Firmata

*Please note that in it's current state, this **does not build** due to a number of wrappers still to be implemented*

This is an implementation of the Firmata firmware for SiliconLabs BGM111. It is compatible with Firmata 2.5.8.

- [x] Target and test async DMA UART communication between Coprocessor and CM3
- [x] Create base balena library for wrapping BGM111 complexity
- [x] Verify compatibility of Firmata Marshaller and Parser Libs
- [ ] Finish wrapper for Arduino-style *Serial* Library
- [ ] Finish wrapper for Arduino-style *Stream* Library
- [ ] Add custom firmata handlers for CM3 Power Rails, etc.
- [ ] Add BGM111 pin mappings/create config file?
- [ ] Document Flashing/Firmware Modification Proceedure
- [ ] Add support for BLE (what features?)
- [ ] Adjust license to adhere to balena and firmata protocol

