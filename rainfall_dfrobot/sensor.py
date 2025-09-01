import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["i2c"]

CONF_RAIN_HOUR = "rain_hour"
CONF_TOTAL_RAIN = "total_rain_mm"
CONF_HOUR_RAIN = "hour_rain_mm"
CONF_RAW_TIPS = "raw_tips"
CONF_WORKING_HOURS = "working_hours"

rainfall_ns = cg.esphome_ns.namespace("rainfall_dfrobot")
DFRobotRainfall = rainfall_ns.class_("DFRobotRainfall", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DFRobotRainfall),
    cv.Optional(CONF_RAIN_HOUR, default=1): cv.int_range(min=1, max=24),
    cv.Optional(CONF_TOTAL_RAIN): sensor.sensor_schema(unit_of_measurement="mm"),
    cv.Optional(CONF_HOUR_RAIN): sensor.sensor_schema(unit_of_measurement="mm"),
    cv.Optional(CONF_RAW_TIPS): sensor.sensor_schema(unit_of_measurement="count"),
    cv.Optional(CONF_WORKING_HOURS): sensor.sensor_schema(unit_of_measurement="h"),
}).extend(cv.polling_component_schema("30s")).extend(i2c.i2c_device_schema(0x1D))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_rain_hour(config[CONF_RAIN_HOUR]))

    if CONF_TOTAL_RAIN in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_RAIN])
        cg.add(var.set_total_rain_sensor(sens))
    if CONF_HOUR_RAIN in config:
        sens = await sensor.new_sensor(config[CONF_HOUR_RAIN])
        cg.add(var.set_hour_rain_sensor(sens))
    if CONF_RAW_TIPS in config:
        sens = await sensor.new_sensor(config[CONF_RAW_TIPS])
        cg.add(var.set_raw_tips_sensor(sens))
    if CONF_WORKING_HOURS in config:
        sens = await sensor.new_sensor(config[CONF_WORKING_HOURS])
        cg.add(var.set_working_hours_sensor(sens))
