from sensor_pack import bus_service, base_sensor
from sensor_pack.base_sensor import Device, Iterator
from sensor_pack import bitfield
import struct
import array


# gas_range_r	const_array1_int	const_array2_int
# ------------------------------------------------
# 0  	    2 ** 31	- 1		    4096000000
# 1		    2 ** 31	- 1		    2048000000
# 2		    2 ** 31	- 1		    1024000000
# 3		    2 ** 31	- 1		    512000000
# 4		    2 ** 31	- 1		    255744255 *
# 5		!   2126008810		    127110228 *
# 6		    2 ** 31	- 1		    64000000
# 7		    2130303777		    32258064 *
# 8		    2 ** 31	- 1		    16016016 *
# 9		    2 ** 31	- 1		    8000000
# 10	    2143188679		    4000000
# 11	    2136746228		    2000000
# 12	    2 ** 31	- 1		    1000000
# 13	!   2126008810		    500000
# 14	    2 ** 31	- 1		    250000
# 15	    2 ** 31	- 1		    125000

class BME680bosh(Device, Iterator):
    """Class for work with Bosh BME680 sensor"""
    def __init__(self, adapter: bus_service.BusAdapter, address=0x77):
        super().__init__(adapter, address, False)
        # osrs_id = 0     относительная влажность
        # osrs_id = 1     давление
        # osrs_id = 2     температура
        self.osrs = bytearray(3)
        self.IIR_filter = 0
        self.mode = False
        # self.enable_gas_conversion = True
        self._i2c_mode = True
        self._calibration_data = array.array("l")  # signed long elements
        #
        self.read_calibration_data()

    @staticmethod
    def get_array_item(id: int, index: int) -> int:
        """Return item from array of const (pls see documentation).
        id = 0, from const_array1_int,
        id = 1, from const_array2_int.
        index must by in range 0..15 (range(16))"""
        base_sensor.check_value(id, range(2), f"Invalid id value: {id}")
        base_sensor.check_value(index, range(16), f"Invalid index value: {index}")
        ct = 2 ** 31 - 1, 4096000000
        t = (5, 7, 10, 11, 13), (4, 5, 7, 8)
        arr = (2126008810, 2130303777, 2143188679, 2136746228, 2126008810), (255744255, 127110228, 32258064, 16016016)

        if index in t[id]:
            return arr[id][t[id].index(index)]
        return ct[id] if 0 == id else ct[id] // (2 ** index)

    @staticmethod
    def _check_calibration_value(value: int, address: int):
        if value == 0xFFFF:		# value == 0x00 or 
            raise ValueError(f"Invalid register value {hex(value)} by addr: {hex(address)}!")

    #   index   param_name
    #   0       par_t2
    #   1       par_t3
    #   2       par_p1
    #   3       par_p2
    #   4       par_p3
    #   5       par_p4
    #   6       par_p5
    #   7       par_p7
    #   8       par_p6
    #   9       par_p8
    #   10      par_p9
    #   11      par_p10
    #   12      par_h3
    #   13      par_h4
    #   14      par_h5
    #   15      par_h6
    #   16      par_h7
    #   17      par_t1
    #   18      par_g2
    #   19      par_g1
    #   20      par_g3
    #   21      par_h1
    #   22      par_h2
    def get_calibration_data(self, index: int) -> int:
        """возвращает калибровочный коэффициент по его индексу.
        returns the calibration coefficient by its index"""
        base_sensor.check_value(index, range(0, 23), f"Invalid index value: {index}")
        return self._calibration_data[index]

    def read_calibration_data(self) -> int:
        if self._calibration_data:
            raise ValueError(f"calibration data array already filled!")
        """Read calibration data and store in array"""
        # Типы значений:
        # 0 - int8_t
        # 1 - uint8_t
        # 2 - int16_t
        # 3 - uint16_t
        address = 0x8A
        tov = "hbHhbhhbbhhBbbbBbHhbb"              # len = 21, value format
        offset = 0, 2, 2, 2, 2, 2, 2, 2, 1, 3, 2, 2, 68, 1, 1, 1, 1, 1, 2, 2, 1  # len = 21
        for typ, offs in zip(tov, offset):
            address += offs
            size = struct.calcsize(typ)

            reg_val = self._read_register(address, size)
            rv = self.unpack(typ, reg_val)[0]
            # check
            BME680bosh._check_calibration_value(rv, address)
            self._calibration_data.append(rv)
            # print(f"address: {hex(address)}; {typ}; size: {size}; value: {rv}")

        # par_h1 read !
        b = self._read_register(0xE1, 3)    # read 0xE1, 0xE2, 0xE3
        rv = (b[2] << 4) | (b[1] & 0x0F)    # par_h1
        BME680bosh._check_calibration_value(rv, 0xE2)
        self._calibration_data.append(rv)
        # print(f"address: 0xE3; H; size: 2; value: {rv}\tpar_h1")
        # par_h2 read !
        rv = (b[0] << 4) | ((b[1] & 0xF0) >> 4)  # par_h2
        BME680bosh._check_calibration_value(rv, 0xE3)
        self._calibration_data.append(rv)
        # print(f"address: 0xE1; H; size: 2; value: {rv}\tpar_h2")

        return len(self._calibration_data)

    @staticmethod
    def _get_raw_wt(val: int) -> tuple:
        """
        Return raw wait time for write in register (multiplier, value_in_ms)
        :param val: 0..4094 [ms] wait time for gas sensor setpoint
        :return: tuple(multiplier, wait_time)
        """
        i, v = 0, 0
        for i in range(4):
            v = val // 4 ** i
            if v <= 64:
                break
        return 4 ** i, v

    @staticmethod
    def _check_gas_id(id: int) -> int:
        return base_sensor.check_value(id, range(10), f"Invalid id value: {id}")

        # BaseSensor
    def _read_register(self, reg_addr, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

        # BaseSensor
    def _write_register(self, reg_addr, value: [int, bytes, bytearray], bytes_count=2) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных"""
        byte_order = self._get_byteorder_as_str()[0]
        return self.adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def set_mode(self, forced_mode: bool = True):
        """Управление режимом работы датчика.
        Если forced_mode Истина, то датчик переходит в режим Forced mode,
        иначе датчик переходит в режим сна (Sleep mode).
        Sensor operation mode control.
        If forced_mode is True, then the sensor goes into Forced mode,
        otherwise the sensor goes into sleep mode (Sleep mode)"""
        addr, lshift = 0x74, 0
        bf = bitfield.BitField(alias=None, start=lshift, stop=1 + lshift)
        val = self._read_register(addr, 1)[0]
        self._write_register(0x74, bf.put(val, int(forced_mode)), 1)
        self.mode = forced_mode

    def set_oversampling(self, osrs_id: int, osrs_value: int = 2):
        """Устанавливает значение передискретизации для:
        osrs_id = 0     относительная влажность
        osrs_id = 1     давление
        osrs_id = 2     температура

        Sets the oversampling value for:
         osrs_id = 0 relative humidity
         osrs_id = 1 pressure
         osrs_id = 2 temperature
        """
        base_sensor.check_value(osrs_id, range(3), f"Invalid osrs_id value: {osrs_id}")
        base_sensor.check_value(osrs_value, range(6), f"id: {osrs_id}. Invalid osrs_value: {osrs_value}")
        # (address, left shift count)
        reg_dat = ((0x72, 0), (0x74, 2), (0x74, 5))
        addr, lshift = reg_dat[osrs_id]
        val = self._read_register(addr, 1)[0]
        bf = bitfield.BitField(start=lshift, stop=2 + lshift, alias=None)
        self._write_register(addr, bf.put(val, osrs_value), 1)
        self.osrs[osrs_id] = osrs_value

    def get_oversampling(self, osrs_id: int) -> int:
        """Return oversampling value.
        osrs_id = 0     relative humidity
        osrs_id = 1     atmosphere pressure
        osrs_id = 2     temperature"""
        base_sensor.check_value(osrs_id, range(3), f"Invalid osrs_id value: {osrs_id}")
        # (address, left shift count)
        reg_dat = ((0x72, 0), (0x74, 2), (0x74, 5))
        addr, lshift = reg_dat[osrs_id]
        x = self._read_register(addr, 1)[0]
        bf = bitfield.BitField(start=lshift, stop=2 + lshift, alias=None)
        return bf.get(x)

    def set_iir_filter(self, iir_value: int = 2):
        """
        IIR-фильтр применяется к данным о температуре и давлении, но не к данным о влажности и газе!
        Данные, поступающие от АЦП, фильтруются, а затем загружаются в регистры данных.
        Регистры результатов температуры и давления обновляются одновременно в конце измерения.

        IIR filter applies to temperature and pressure data but not to humidity and gas data.
        The data coming from the ADC are filtered and then loaded into the data registers.
        The temperature and pressure result registers are updated together at the same time at the end
        of the measurement."""
        base_sensor.check_value(iir_value, range(8), f"Invalid iir_value value: {iir_value}")
        addr, lshift = 0x75, 2
        val = self._read_register(addr, 1)[0]
        bf = bitfield.BitField(alias=None, start=lshift, stop=2 + lshift)
        self._write_register(addr, bf.put(val, iir_value), 1)
        self.IIR_filter = iir_value

    def get_id(self):
        """Возвращает идентификатор датчика и его revision ID.
        Правильное значение: 0x61
        Returns the ID and revision ID of the sensor."""
        return self._read_register(0xD0 if self._i2c_mode else 0x50, 1)[0]

    def soft_reset(self):
        """Software reset"""
        self._write_register(0xE0 if self._i2c_mode else 0x60, 0xB6, 1)

    # GAS
    def heater_set_current(self, id: int, current: int):
        """
        Setup sensor gas heater with current
        :param current: 0..15 [mA]
        :param id: 0..9
        :return: None
        """
        BME680bosh._check_gas_id(id)
        base_sensor.check_value(current, range(17), f"Invalid current value: {current}")
        raw_curr = (current << 3) - 1
        self._write_register(0x50 + id, raw_curr << 1, 1)   # bit 7..1 used, see documentation

    def heater_set_resistance(self, id: int, value: int):
        """
        Setup sensor gas heater with resistance
        :param id: 0..9
        :param value: Decimal value that needs to be stored for achieving target heater resistance
        :return: None
        """
        BME680bosh._check_gas_id(id)
        base_sensor.check_value(value, range(0x100), f"Invalid current value: {value}")
        self._write_register(0x5A + id, value, 1)

    def heater_set_wait_time(self, id: int, wait_time: int):
        """
        Setup sensor gas heater with wait_time
        :param id: 0..9
        :param wait_time: 0..4096 [ms]
        :return: wait time in [ms]
        """
        BME680bosh._check_gas_id(id)
        base_sensor.check_value(wait_time, range(4097), f"Invalid id value: {id}")
        t = BME680bosh._get_raw_wt(wait_time)
        self._write_register(0x64 + id, (t[0] << 6) | t[1], 1)

    def heater_set_point(self, id: int, current: int, resistance: int, wait_time: int):
        """
        Setup sensor gas heater with current, resistance, wait_time
        :param id: 0..9
        :param current: 0..15 [mA]
        :param resistance: Decimal value that needs to be stored for achieving target heater resistance
        :param wait_time: 0..4096 [ms]
        :return:
        """
        self.heater_set_current(id, current)
        self.heater_set_resistance(id, resistance)
        self.heater_set_wait_time(id, wait_time)

    def heater_enable_set_point(self, id: int, run_gas_conversion: bool):
        BME680bosh._check_gas_id(id)
        self._write_register(0x71, (int(run_gas_conversion) << 4) | id, 1)

    def heater_on(self, on: bool):
        """Turn on/off current injected to heater"""
        self._write_register(0x70, (int(not on) << 3), 1)

    # Data registers
    def _get_3x_data(self, start_addr: int) -> int:
        msb, lsb, xlsb = self._read_register(start_addr, 3)
        return (msb << 12) | (lsb << 4) | (xlsb & 0xF0) >> 4

    def _get_press(self) -> int:
        """return raw pressure"""
        return self._get_3x_data(0x1F)

    def _get_temp(self) -> int:
        """return raw pressure"""
        return self._get_3x_data(0x22)

    def _get_hum(self) -> int:
        """return raw humidity"""
        b = self._read_register(0x25, 2)
        return self.unpack("H", b)[0]

    def _get_gas_resistance_data(self) -> tuple:
        """Return (range_of_measured_gas_sensor_resistance, gas_sensor_resistance_data)"""
        msb, lsb = self._read_register(0x2A, 2)
        return lsb & 0x0F, (msb << 2) | ((lsb & 0xC0) >> 6)

    def get_gas_meas_status(self) -> tuple:
        """Return tuple(new_data_flag, Gas_measuring_status_flag, Measuring_status_flag, Gas_measurement_index)"""
        reg_val = self._read_register(0x1D, 1)[0]
        # new_data_flag, Gas_measuring_status_flag, Measuring_status_flag, Gas_measurement_index
        return bool(reg_val & 0x80), bool(reg_val & 0x40), bool(reg_val & 0x20), reg_val & 0x0F

    def get_gas_valid_status(self) -> tuple:
        """Return tuple(gas_valid_r, heat_stab_r)"""
        reg_val = self._read_register(0x2B, 1)[0]
        # gas_valid_r, heat_stab_r
        return bool(reg_val & 0x20), bool(reg_val & 0x10)

    def __iter__(self):
        return self

    def __next__(self):
        ...
