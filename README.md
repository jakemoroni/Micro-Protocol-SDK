# Micro-Protocol-SDK

Bootloaders and example applications implementing Micro Protocol

Micro Protocol is a pure Ethernet protocol that defines a set of low level functionalities for microcontrollers with an Ethernet connection. The primary purpose is to enable remote firmware updates as well as other system-level functions such as remote reboot commands, console tunnelling, general data transfers, and querying all devices on the network. The protocol was designed to be as simple as possible for easy integration into microcontroller projects. The complete protocol is defined in a single header file (*micro_protocol.h*) and all Micro Protocol messages are of a fixed 64 byte length (68 bytes including the Ethernet FCS). See *micro_protocol.h* for more detailed info.

## Implementation

A full Micro Protocol implementation consists of two parts:

- A bootloader implementing the Micro Protocol firmware update process

- *Optional* hooks in the main application to accept Micro Protocol commands during runtime. If these hooks aren't implemented, then updates can only be performed by physically resetting the device, which allows it to enter the bootloader and receive the update command. By implementing Micro Protocol support in the application, an *UPDATE_REQUEST* message can be received at any time which will allow the device to be updated without any physical interaction.

## Hardware

Micro Protocol itself is intended to be hardware agnostic, but an example bootloader and application is provided that supports the following platforms:

- Arduino UNO (or compatible) + W5500 Ethernet Shield

- Arduino MEGA (or compatible) + W5500 Ethernet Shield

## Building and Flashing the AVR Bootloader

### Prerequisites

- gcc-avr

- avr-libc

- avrdude

- make

On Ubuntu, these can be installed with:

- sudo apt install gcc-avr avr-libc avrdude make

### Building

- Edit the Makefile in bootloader_avr_w5500 and set/modify the following variables at the top of the file:
  
  - TARGET: Set to either TARGET_UNO or TARGET_MEGA depending on the platform.
  
  - AVRDUDE_BINARY: Set to the avrdude binary to use for programming. This can probably just be set to avrdude to use the system's avrdude utility, but I've had issues when using the Atmel ICE that required me to build the latest avrdude, so this allowed me to use that instead.
  
  - AVRDUDE_PROGRAMMER: Uncomment the appropriate line for whichever programming interface you're using. The Pololu programmer requires that the serial port be specified. This can be discovered via trial and error.
    
    - **NOTE: The only programmer that was able to program my ATmega2560 board was the genuine Atmel ICE. However, they all worked for the ATmega238P.**

- *Optionally* create a "HWADDR.txt" file containing the MAC address in the form of XX:XX:XX:XX:XX:XX in the bootloader_avr_w5500 directory. If present, this is the MAC address that will get programmed into the last 6 bytes of the EEPROM on the device and will be used as the source MAC for all Micro Protocol messages originating from the device. If no HWADDR.txt file is present, then one will be automatically created that contains a locally administered MAC with the last 3 octets being random values.

- make

### Flashing

- make flash
  
  - This programs the entire device, including the EEPROM.
  
  - The device is now ready to accept a firmware update via Micro Protocol.

## Building the AVR Application

TODO

## Building the Micro Protocol PC Utilities

TODO

## Performing Remote Updates

TODO

## Console Tunnelling

TODO

## Querying Devices

TODO

## License

All files in this repository are licensed under the GPL v3.0 or Later. See LICENSE.txt.
