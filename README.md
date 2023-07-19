# compressor_control
Project for control of hydrogen filling station.

Initially control of 3 stage gas booster compression system (to 700 bar) via touch screen GUI using Python + Tk

Control(via pneumatic 24v relays x 5 expandable to 8)of:
hydrogen input valve
Air to each booster
High pressure H2 vent (NO) (for emergencies)

Monitoring of H2 pressures (4-20 mA x 4, expandable to 8)
input
output
interstage (x2)

Monitoring of Temperatures (themocouples i2c interface x 4, expandable to 8)
input
output
interstage (x2)

Monitoring of car IR data via RS485 serial link

For future use:
i2c 4- 20 mA ~24v transmitter - for control of proportional H2 valve
2 x SPDT relays.
Additional temperature sensors DS18B20

Hardware:

RPi 4 in Argon case with 250 Gb SSHD
OS bullseye version ID 11
Python 3.9.2
I2C on pins GPIO2 /SDA1 and GPIO3 /SDL1 

For current loops: 
DFROBOT Gravity: Analog Current to Voltage Converter + 8 channel, 12c, Pi ADC 17 bit,(2 x 4 channel  MCP3424 A/D 12c address 0x68, 0x69)
(Channel 8 for hydrogen alarm)

For temperature:
Adafruit MCP9600 thermocouple breakout 12c (Address 0x67 - 0x60)

Additional Temperature measurement with DS18B20 sensors on GPIO 22

For 24v pneumatic relay control:
Jee Labs Output plug - i2c, 8 channel, uses MCP23008 + darlington array ULN2803 (i2c Address 0x26 / 0x27)

For current loop transmitter:
DFROBOT Gravity: I2C 4-20mA DAC Modul, uses chip  GP8302 (i2c Address: 0x58)

For 5v relays:
R1 GPIO 17
R2 GPIO 27

For RS485 (Ir serial from car):
RS485 to ttl board MAX 3485 (NB 5v to 3.3 - need voltage divider) RPi Tx GPIO 14, Rx GPIO 15
