import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_KILOMETER,
)
from esphome import pins

CODEOWNERS = ["@yourname"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "text_sensor"]

as3935_ns = cg.esphome_ns.namespace("as3935_dfrobot")
AS3935DFRobot = as3935_ns.class_(
    "AS3935DFRobot", cg.PollingComponent, i2c.I2CDevice
)

CONF_INDOOR = "indoor"
CONF_MASK_DISTURBERS = "mask_disturbers"
CONF_NOISE_LEVEL = "noise_level"
CONF_SPIKE_REJECTION = "spike_rejection"
CONF_WATCHDOG_THRESHOLD = "watchdog_threshold"
CONF_LIGHTNING_THRESHOLD = "lightning_threshold"
CONF_DISTANCE = "distance"
CONF_ENERGY = "energy"
CONF_EVENT = "event"
CONF_IRQ_PIN = "irq_pin"
CONF_TUNING_CAP = "tuning_cap"
CONF_TUNE_ANTENNA = "tune_antenna"
CONF_LCO_TARGET_KHZ = "lco_target_khz"
CONF_LCO_FDIV = "lco_fdiv"

DISTANCE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_KILOMETER,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_DISTANCE,
    state_class=STATE_CLASS_MEASUREMENT,
    icon="mdi:map-marker-distance",
)

ENERGY_SENSOR_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=0,
    icon="mdi:flash",
    state_class=STATE_CLASS_MEASUREMENT,
)

EVENT_TEXT_SENSOR_SCHEMA = text_sensor.text_sensor_schema(icon="mdi:weather-lightning")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS3935DFRobot),
            cv.Optional(CONF_ADDRESS, default=0x03): cv.hex_int,
            cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INDOOR, default=False): cv.boolean,
            cv.Optional(CONF_MASK_DISTURBERS, default=False): cv.boolean,
            cv.Optional(CONF_NOISE_LEVEL, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_SPIKE_REJECTION, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_WATCHDOG_THRESHOLD, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_LIGHTNING_THRESHOLD, default=1): cv.one_of(1, 5, 9, 16, int=True),
            cv.Optional(CONF_TUNING_CAP, default=12): cv.int_range(min=0, max=15),
            cv.Optional(CONF_TUNE_ANTENNA, default=False): cv.boolean,
            cv.Optional(CONF_LCO_TARGET_KHZ, default=500): cv.int_range(min=100, max=1000),
            cv.Optional(CONF_LCO_FDIV, default=16): cv.one_of(16, 32, 64, 128, int=True),
            cv.Optional(CONF_DISTANCE): DISTANCE_SENSOR_SCHEMA,
            cv.Optional(CONF_ENERGY): ENERGY_SENSOR_SCHEMA,
            cv.Optional(CONF_EVENT): EVENT_TEXT_SENSOR_SCHEMA,
        }
    )
    .extend(i2c.i2c_device_schema(0x03))
    .extend(cv.polling_component_schema("50ms"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    irq = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(irq))

    cg.add(var.set_indoor(config[CONF_INDOOR]))
    cg.add(var.set_mask_disturbers(config[CONF_MASK_DISTURBERS]))
    cg.add(var.set_noise_level(config[CONF_NOISE_LEVEL]))
    cg.add(var.set_spike_rejection(config[CONF_SPIKE_REJECTION]))
    cg.add(var.set_watchdog_threshold(config[CONF_WATCHDOG_THRESHOLD]))
    cg.add(var.set_lightning_threshold(config[CONF_LIGHTNING_THRESHOLD]))
    cg.add(var.set_tuning_cap(config[CONF_TUNING_CAP]))
    cg.add(var.set_tune_antenna(config[CONF_TUNE_ANTENNA]))
    cg.add(var.set_lco_target_khz(config[CONF_LCO_TARGET_KHZ]))
    cg.add(var.set_lco_fdiv(config[CONF_LCO_FDIV]))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY])
        cg.add(var.set_energy_sensor(sens))

    if CONF_EVENT in config:
        ts = await text_sensor.new_text_sensor(config[CONF_EVENT])
        cg.add(var.set_event_text_sensor(ts))
