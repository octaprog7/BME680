# ***
"""Код в этом файле появился благодаря репозиторию `raspi-bme680-iaq`.
Я избавился от использования numpy (слишком жирен для MCU) и сделал код более удобным для понимания
https://github.com/thstielow/raspi-bme680-iaq.

The code in this file comes from the `raspi-bme680-iaq` repository.
I got rid of using numpy and made the code easier to understand.
https://github.com/thstielow/raspi-bme680-iaq"""
from IAQ import IAQ_utils


class IAQTracker:
    """Index Air Quality calculation class.
    Based on `raspi-bme680-iaq`. https://github.com/thstielow/raspi-bme680-iaq"""
    def __init__(self, burn_in_cycles: int = 300, gas_recal_period: int = 3600, ph_slope: float = 0.03):
        """burn_in_cycles: determines burn-in-time, usually 5 minutes, equal to 300 cycles of 1s duration.
        gas_recal_period: number of cycles after which to drop last entry of the gas calibration list. Here: 1h.
        ph_slope` defines the slope of the linear compensation of the logatihmic gas resistance by the present humidity.
        This parameter strongly depends on the heating profile and polling frequency and you may have to adapt
        it for your specific setup. The default value was determined experimentally by running the sensor in
        controlled environment (see below) and fitting the humidity-resistance plot."""
        self.slope = ph_slope
        self.burn_in_cycles = burn_in_cycles
        self.gas_cal_data = list()
        self.gas_ceil = 0
        self.gas_recal_period = gas_recal_period
        self.gas_recal_step = 0

    def get_index_air_quality(self, temperature: float, relative_humidity: float,
                              pressure: float, gas_resistance: float) -> [float, None]:
        """Return Index Air Quality.
        If it returns None, you need to wait a while!"""
        # calculate saturation density and absolute humidity
        rho_max = IAQ_utils.get_water_sat_density(temperature)
        hum_abs = IAQ_utils.get_absolute_humidity(relative_humidity, rho_max)
        # compensate exponential impact of humidity on resistance
        comp_gas = IAQ_utils.compensate_gas_res(gas_resistance, self.slope, hum_abs)

        if self.burn_in_cycles > 0:
            # check if burn-in-cycles are recorded
            self.burn_in_cycles -= 1  # count down cycles
            if comp_gas > self.gas_ceil:  # if value exceeds current ceiling, add to calibration list and update ceiling
                self.gas_cal_data = [comp_gas]
                self.gas_ceil = comp_gas
            return None  # return None type as sensor burn-in is not yet completed
        else:   # self.burn_in_cycles <= 0
            lst = self.gas_cal_data  # ref
            # adapt calibration
            if comp_gas > self.gas_ceil:
                lst.append(comp_gas)
                if len(lst) > 100:
                    del lst[0]
                self.gas_ceil = IAQ_utils.get_mean(lst)

            # calculate and print relative air quality on a scale of 0-100%
            # use quadratic ratio for steeper scaling at high air quality
            # clip air quality at 100%
            air_quality = 100 * min((comp_gas / self.gas_ceil) ** 2, 1)
            print(f"IAQ debug: {comp_gas}, {self.gas_ceil}")

            # for compensating negative drift (dropping resistance) of the gas sensor:
            # delete oldest value from calibration list and add current value
            self.gas_recal_step += 1
            if self.gas_recal_step >= self.gas_recal_period:
                self.gas_recal_step = 0
                lst.append(comp_gas)
                del lst[0]
                self.gas_ceil = IAQ_utils.get_mean(lst)

        return air_quality
