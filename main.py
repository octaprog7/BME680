from machine import I2C, Pin
import bme680
import time
from sensor_pack.bus_service import I2cAdapter
from sensor_pack import converter


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
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
    bgas = True
    default_gas_id = 0
    bme.heater_set_point(id=default_gas_id, current=5, wait_time=40, hot_plate_temperature=300)
    # set oversampling for relative humidity, temperature, air pressure
    # никогда не пропускайте измерения температуры!
    bme.set_oversamplings(osrs_temp=2, osrs_hum=1, osrs_press=1)
    bme.set_iir_filter(2)
    if bgas:
        bme.heater_enable_set_point(id=default_gas_id, run_gas_conversion=True)
    
    print("Without using an iterator")
    for _ in range(10):
        if bgas:
            bme.heater_enable_set_point(id=default_gas_id, run_gas_conversion=True)
        # set mode
        bme.set_mode(True)
        time.sleep_ms(1500)
        t = bme.get_temperature()   # вызов метода обязателен! Вызывать первым!
        h = bme.get_humidity()
        pas = bme.get_pressure()
        print(f"T={t} Celsius; H={h} %; P={pas} Pa; mm Hg={converter.pa_mmhg(pas)} ")
        if bgas:
            gas_valid, heat_stab = bme.get_gas_valid_status()
            if heat_stab and gas_valid:
                gas = bme.get_gas()
                print(f"get_gas: {gas}")

    print("Using an iterator")
    for hum, press, temp, gas in bme:
        bme.set_mode(True)
        time.sleep_ms(1500)
        print(f"T={temp} Celsius; H={hum} %; mm Hg={converter.pa_mmhg(press)}; Gas={gas}")