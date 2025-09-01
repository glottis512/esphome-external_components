import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, binary_sensor, text_sensor
from esphome import pins

CODEOWNERS = ["@your-handle"]

as3935_ns = cg.esphome_ns.namespace("as3935_dfrobot")
AS3935Component = as3935_ns.class_(
    "AS3935DFRobot",
    cg.PollingComponent,
    i2c.I2CDevice,
)

CONF_IRQ_PIN = "irq_pin"
CONF_INDOOR = "indoor"
CONF_NOISE = "noise_level"
CONF_SPIKE = "spike_rejection"
CONF_WATCHDOG = "watchdog_threshold"
CONF_MIN_STRIKES = "min_strikes"
CONF_DISTURBER = "disturber_detection"
CONF_TUNING_CAPS = "tuning_caps"
CONF_USE_INTERRUPT = "use_interrupt"

CONF_DISTANCE = "distance"
CONF_ENERGY = "energy"
CONF_LIGHTNING = "lightning"
CONF_NOISE_BI = "noise"
CONF_DISTURBER_BI = "disturber"
CONF_LAST_EVENT = "last_event"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS3935Component),
            cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INDOOR, default=False): cv.boolean,
            cv.Optional(CONF_NOISE, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_SPIKE, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_WATCHDOG, default=2): cv.int_range(min=0, max=7),
            cv.Optional(CONF_MIN_STRIKES, default=1): cv.one_of(1, 5, 9, 16, int=True),
            cv.Optional(CONF_DISTURBER, default=True): cv.boolean,
            cv.Optional(CONF_TUNING_CAPS, default=0): cv.int_range(min=0, max=120),
            cv.Optional(CONF_USE_INTERRUPT, default=True): cv.boolean,

            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement="km",
                icon="mdi:map-marker-distance",
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                icon="mdi:flash",
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_LIGHTNING): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_NOISE_BI): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_DISTURBER_BI): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_LAST_EVENT): text_sensor.text_sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("200ms"))
    .extend(i2c.i2c_device_schema(0x03))
)


async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    irq = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(irq))

    cg.add(var.set_indoor(config[CONF_INDOOR]))
    cg.add(var.set_noise_floor(config[CONF_NOISE]))
    cg.add(var.set_spike_rejection(config[CONF_SPIKE]))
    cg.add(var.set_watchdog_threshold(config[CONF_WATCHDOG]))
    cg.add(var.set_min_strikes(config[CONF_MIN_STRIKES]))
    cg.add(var.set_disturber_detection(config[CONF_DISTURBER]))
    cg.add(var.set_tuning_caps(config[CONF_TUNING_CAPS]))
    cg.add(var.set_use_interrupt(config[CONF_USE_INTERRUPT]))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY])
        cg.add(var.set_energy_sensor(sens))

    if CONF_LIGHTNING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_LIGHTNING])
        cg.add(var.set_lightning_binary(bs))

    if CONF_NOISE_BI in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_NOISE_BI])
        cg.add(var.set_noise_binary(bs))

    if CONF_DISTURBER_BI in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_DISTURBER_BI])
        cg.add(var.set_disturber_binary(bs))

    if CONF_LAST_EVENT in config:
        ts = await text_sensor.new_text_sensor(config[CONF_LAST_EVENT])
        cg.add(var.set_last_event_text(ts))
