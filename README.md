# BME680
MicroPython module for work with BME680 pressure&temperature&humidity&gas sensor from Bosch Sensortec.

Just connect your BME680 board to Arduino, ESP or any other board with MicroPython firmware.

Supply voltage BME680 3.3 Volts only!
1. VCC
2. GND
3. SDA
4. SCL

Upload MicroPython firmware to the NANO(ESP, etc) board, and then two files: main.py, bme680.py and sensor_pack folder. 
Then open main.py in your IDE and run it.

# Pictures

## IDE
![alt text](https://github.com/octaprog7/BME680/blob/master/bme680ide.png)
## Breadboard
![alt text](https://github.com/octaprog7/BME680/blob/master/bme680board.jpg)

# Index Air Quality (IAQ)
Модуль IAQ_tracker.py основан на коде raspi-bme680-iaq. https://github.com/thstielow/raspi-bme680-iaq. 
Для получения правильных значений IAQ, необходима длительная и сложная калибровка. 
Читайте readme по ссылке: https://github.com/thstielow/raspi-bme680-iaq.

The IAQ_tracker.py module is based on the raspi-bme680-iaq code. https://github.com/thstielow/raspi-bme680-iaq. 
In order to obtain the correct values of IAQ, a lengthy and complicated calibration is necessary. 
Read the readme at: https://github.com/thstielow/raspi-bme680-iaq.