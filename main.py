from machine import I2C, Pin
import bme680
import time
from sensor_pack.bus_service import I2cAdapter


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) № для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    # Warning!!!
    # Replace id=1 with id=0 if you are using the first I2C port !!!
    i2c = I2C(id=1, freq=400_000)  # on Arduino Nano RP2040 Connect tested
    adaptor = I2cAdapter(i2c)
    # bme - sensor
    bme = bme680.BME680bosh(adaptor)
    chip_id = bme.get_id()
    print(f"chip_id: {hex(chip_id)}")
    l = len(bme._calibration_data)
    print(f"calibration data array len: {l}")
    for i in range(l):
        print(f"data[{i}] = {bme.get_calibration_data(i)}")

    # set oversampling for relative humidity, temperature, air pressure
    bme.set_oversampling(0, 1)  # relative humidity
    bme.set_oversampling(2, 2)  # temperature
    bme.set_oversampling(1, 5)  # air pressure
    #
    bme.set_iir_filter(3)
    # set mode
    # bme.set_mode(True)

    for i in range(100):
        bme.set_mode(True)
        time.sleep_ms(1500)
        t = bme.get_temperature()
        h = bme.get_humidity()
        p = bme.get_pressure()
        print(f"T={t}; H={h}; P={p}")
