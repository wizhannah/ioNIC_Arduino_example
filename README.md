# W55RP20-S2E Arduino Example

This project provides example codes for controlling the W55RP20-S2E (Serial-to-Ethernet) module using a Raspberry Pi Pico.

The Pico transmits data and commands via UART or SPI, allowing the W55RP20-S2E to be configured through AT commands or to run network applications.

For more details about W55RP20-S2E, please refer to the official documentation:

https://docs.wiznet.io/Product/Chip/MCU/Pre-programmed-MCU/W55RP20-S2E/overview-en

## Example List
| Name       | Description     | Interface |   
|---------------|---------------|----------------------------------------|
| at_command | Sends AT commands & checks responses | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/at_command/at_command_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/at_command/at_command_spi) |
| loopback/server |  TCP Server Loopback | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/server/loopback_server_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/server/loopback_server_spi) |
| loopback/client |  TCP Client Loopback | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/client/loopback_client_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/client/loopback_client_spi) |
| loopback/udp |  UDP Loopback | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/udp/loopback_udp_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/loopback/udp/loopback_udp_spi) |
| http client | HTTP GET Request | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/http/http_client_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/http/http_client_spi) |
| webserver | Web Server Example | [UART](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/webserver/webserver_uart) / [SPI](https://github.com/wizhannah/ioNIC_Arduino_example/tree/main/webserver/webserver_spi) |
| (To be added...)   |

## Hardware Setup
### W55RP20 Interface Selection
| Pin   | State  |  Desc  | 
|------------|------------|------------|
| GPIO13 | LOW(GND) | UART mode(default) |
| GPIO13 | HIGH(3.3V) | SPI mode |

### UART example
| Pico   | W55RP20  |  Desc  | 
|------------|------------|------------|
| GPIO4(UART_TX) | GPIO5(UART_RX) | |
| GPIO5(UART_RX) | GPIO4(UART_TX) | |
| GND | GND | |


### SPI example
| Pico   | W55RP20  |  Desc  | 
|------------|------------|------------|
| GPIO2(SPI_CLK) | GPIO2(SPI_CLK) | CLK |
| GPIO3(SPI_TX) | GPIO4(SPI_RX) | MOSI |
| GPIO4(SPI_RX) | GPIO3(SPI_TX) | MISO |
| GPIO5(SPI_CS) | GPIO5(SPI_CS) | CS |
| GPIO26(SPI_INT) | GPIO26(SPI_INT) | INT |
| GND | GND | |