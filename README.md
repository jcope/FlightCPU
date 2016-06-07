# FlightCPU
An embedded data logger for high altitude balloons

The Flight CPU is designed as a data logger for High Altitude Balloons. It is designed around the ATMEGA328P so it can easily be developed using Arduino tools. Most sensor modules communicate with the uC using I2C with the exception of the Humidity/Temp sensor which uses SPI. The data is then logged on the MicroSD card. 

Sensors include: Gyroscope, Compass, Temp/Humidity, Barometer.
