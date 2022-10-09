# ***
import math


def get_water_sat_density(temperature: float) -> float:
    """calculates the saturation water density of air at the current temperature (in °C)
    return the saturation density rho_max in kg/m^3	this is equal to a relative humidity of 100% at
    the current temperature.
    Вычисляет плотность насыщения водой воздуха при текущей температуре (в °C) возвращает плотность насыщения
    rho_max в кг/м^3 это равно относительной влажности 100% при текущей температуре"""
    zero_kelvin = 273.15
    rho_max = (6.112 * 100 * math.exp((17.62 * temperature) / (243.12 + temperature))) / (
            461.52 * (temperature + zero_kelvin))
    return rho_max


def get_absolute_humidity(relative_humidity: float, water_sat_density: float) -> float:
    """Return absolute humidity in g/m^3.
    Возвращает абсолютную влажность в г/м^3"""
    return 10 * relative_humidity * water_sat_density


def compensate_gas_res(gas_resistance: float, slope: float, absolute_humidity: float) -> float:
    """Compensate exponential impact of humidity on resistance"""
    return gas_resistance * math.exp(slope * absolute_humidity)


def get_mean(iterable) -> float:
    """Возвращает среднее арифметическое последовательности"""
    _len = len(iterable)
    if 0 == _len:
        return float("inf")
    return sum(iterable) / _len
