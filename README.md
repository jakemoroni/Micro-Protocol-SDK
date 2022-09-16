# Micro-Protocol-SDK

Bootloaders and example applications implementing Micro Protocol

Micro Protocol is an Ethernet protocol that defines a set of low level functionalities for microcontrollers with an Ethernet connection. The primary purpose is to enable remote firmware updates as well as other system-level functions such as remote reboot commands, console tunnelling, general data transfers, and querying all devices on the network. The protocol was designed to be as simple as possible for easy integration into microcontroller projects. The complete protocol is defined in a single header file (*micro_protocol.h*) and all Micro Protocol messages are of a fixed 64 byte length (68 bytes including the Ethernet FCS). See *micro_protocol.h* for more detailed info.

## Rationale

Micro Protocol and the example AVR bootloader/application(s) were created to simplify the development and deployment of projects based on the Arduino UNO/MEGA (or compatible) and W5500 Ethernet Shield. The goal was to create a framework that can serve as the base for any type of microcontroller project. While originally designed with the Arduino in mind, the protocol itself, as well as the associated PC utilities don't depend on any specific hardware.

The original intent was just to enable firmware updates over Ethernet, but other features quickly became useful. For example, a simple identification scheme was implemented that allows for a host (running Linux) to query all of the Micro Protocol devices on the network. Additionally, Micro Protocol supports transmitting console data over the network. In the case of the AVR example application, this is used to redirect stdout (i.e., printf, etc.) to a Linux PC running the *mp_console* utility. The end result is identical to if you were using the UART and observing the output with the Arduino IDE serial monitor, except it operates over Ethernet and requires nothing other than a Linux PC with an Ethernet connection.

**It is important to note that Micro Protocol is a pure Ethernet protocol - no IP involved.** This greatly simplifies device configuration by avoiding things like DHCP or static IP address assignment. Instead, the only requirement is that every device has a unique MAC address and is reachable on the network. The downside is that it will not readily operate over IP networks (i.e., the Internet). That said, in the majority of cases, the typical home network is one giant L2 network anyway (even WiFi).

### FAQ

- **Q**: Why use the Arduino UNO or MEGA? They're 8 bit AVRs and you can get ARM MCUs that are much faster for cheaper...

- **A**: They get the job done and they're everywhere, plus there are a ton of aftermarket shields to choose from, and 5V IO powerful enough to light an LED without extra circuitry is convenient. Since this is just for hobby projects, cost isn't a huge factor either.

- **Q**: There are already some Ethernet bootloaders for the AVR. Why re-invent the wheel?

- **A**: Most of the existing bootloaders that I found rely on a TFTP server, which is sort of a pain to set up. Plus, it requires either a DHCP server or some way to statically assign an IP address to the device. With the Micro Protocol bootloader, updating the firmware on a device requires nothing other than a Linux computer and an Ethernet connection. The *mp_update* binary has no dependencies.

## Implementation

A full Micro Protocol implementation consists of two parts:

- A bootloader implementing the Micro Protocol firmware update process.

- *Optional* hooks in the main application to accept Micro Protocol commands during runtime. If these hooks aren't implemented, then updates can only be performed by physically resetting the device, which allows it to enter the bootloader and receive the update command. By implementing Micro Protocol support in the application, an *UPDATE_REQUEST* message can be received at any time which will allow the device to be updated without any physical interaction.

## Hardware

Micro Protocol and the associated PC utilities do not require any specific hardware, but example bootloaders and applications are provided for the following platforms:

- Arduino UNO (or compatible) + W5500 Ethernet Shield

- Arduino MEGA (or compatible) + W5500 Ethernet Shield

## Building and Flashing the AVR Bootloader

### Prerequisites

- gcc-avr

- avr-libc

- avrdude

- make

On Ubuntu, these can be installed with:

- *sudo apt install gcc-avr avr-libc avrdude make*

### Building

- Edit the Makefile in bootloader_avr_w5500 and set/modify the following variables at the top of the file:
  
  - TARGET: Set to either TARGET_UNO or TARGET_MEGA depending on the platform.
  
  - AVRDUDE_BINARY: Set to the avrdude binary to use for programming. This can probably just be set to avrdude to use the system's avrdude utility, but I've had issues when using the Atmel ICE that required me to build the latest avrdude, so this allowed me to use that instead.
  
  - AVRDUDE_PROGRAMMER: Uncomment the appropriate line for whichever programming interface you're using. The Pololu programmer requires that the serial port be specified. This can be discovered via trial and error.
    
    - **NOTE: The only programmer that was able to program my ATmega2560 board was the genuine Atmel ICE. However, they all worked for the ATmega238P.**

- *Optionally* create a "HWADDR.txt" file containing the MAC address in the form of XX:XX:XX:XX:XX:XX in the bootloader_avr_w5500 directory. If present, this is the MAC address that will get programmed into the last 6 bytes of the EEPROM on the device and will be used as the source MAC for all Micro Protocol messages originating from the device. If no HWADDR.txt file is present, then one will be automatically created that contains a locally administered MAC with the last 3 octets being random values.

- Run *make* in the bootloader_avr_w5500 directory.

### Flashing

- Run *make flash* in the bootloader_avr_w5500 directory.
  
  - This programs the entire device, including the EEPROM.
  
  - The device is now ready to accept a firmware update via Micro Protocol.

## Building the AVR Application

Similar to building the bootloader, all that is required is to set the TARGET variable in the Makefile and run *make*. The resulting update file will be placed in the application_avr_w5500 directory and will be called *application.update*.

## Micro Protocol PC Utilities

There are three utilities provided that are designed to run on a Linux PC and interact with Micro Protocol devices on the network:

- **mp_query**: This tool is used to query Micro Protocol devices on the network. If no MAC address is provided, then the query is sent to the broadcast address and any devices should respond. If a MAC is provided, then the query is sent only to that device, but since other devices are free to autonomously generate IDENTIFY_NOTICE messages, you may observe responses from other devices on the network.

- **mp_console**: This tool is receives console data from a Micro Protocol device and writes it to standard output. In essence, it allows for printf() to be used on the micocontroller with the output appearing on a local terminal window on the Linux PC. For now, this only supports console data from target->host. Support for host->target could be added in the future, but special care must be taken to ensure pacing such that messages aren't lost.

- **mp_update**: This tool is used to update the firmware on a Micro Protocol device (see "Performing Remote Updates").

**NOTE: All of these utilities require the ability to open a raw socket. The easiest way to do this is to run them as root via sudo. Alternatively, you can the cap_net_raw capability to the executable one time to avoid needing to use sudo every time. For example:**

- *sudo setcap cap_net_raw,cap_net_admin=eip ./mp_query*

### Building the Utilities

Run make in the utilities directory. The utilities will be placed in the root of the utilities directory.

### Performing Remote Updates

- Build the application for the target device (see "Building the AVR Application").

- Note the target device's MAC. If unknown, the mp_query utility can be used.

- Run the mp_update utility as follows:
  
  - mp_update interface target_mac application.update
    
    - *interface*: the network interface name that is connected to the target device's network
    
    - *target_mac*: the MAC address of the target device, in the form of XX:XX:XX:XX:XX:XX
    
    - *application.update*: the full path to the update file that was generated when the application was built

The update takes a few seconds for ATmega328P boards, and around 30 seconds for a MEGA board. It is normal for there to be a few retransmissions during and update - the protocol is designed to handle this. This will particularly be the case when updating over WiFi, etc. The device bootloaders are designed with 1 second timeouts, so if there isn't a successful transfer for 1 second, then the update process automatically restarts.

If the update happens to fail while in progress (due to power loss, connection interruption, etc.), then the devices will automatically exit the update process and remain in the bootloader, continuously transmitting *IDENTIFY_NOTICE* messages with the *crc_bad* field set to 1, indicating that the firmware image calculated CRC does not match the value stored at the end of the image. This allows the update to be retried.

**It therefore essentially impossible to "brick" the device by performing an update.**

In the case where an update image is flashed that behaves in an unexpected way such that the application doesn't respond to Micro Protocol messages anymore, then a manual reset or power cycle may be required to get the device to enter into the bootloader and accept the update request. In this case, it's recommended to actually start the update utility *before* resetting the device so that it will be able to catch the device before it boots into the faulty application. The device only stays in the bootloader for ~8 seconds after a reset.

## License

All files in this repository are licensed under the GPL v3.0 or Later. See LICENSE.txt.
