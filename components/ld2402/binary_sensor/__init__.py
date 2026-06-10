import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_FILTERS,
    CONF_HAS_MOVING_TARGET,
    CONF_HAS_STILL_TARGET,
    CONF_HAS_TARGET,
    CONF_ID,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_OCCUPANCY,
)

from .. import CONF_LD2402_ID, LD2402Component, ld2402_ns

LD2402BinarySensor = ld2402_ns.class_(
    "LD2402BinarySensor", cg.Component
)

ICON_MEDITATION = "mdi:meditation"
ICON_SHIELD_ACCOUNT = "mdi:shield-account"
ICON_TARGET_ACCOUNT = "mdi:target-account"

CONF_PRESENCE_SETTLE = "presence_settle"
CONF_MOVING_TARGET_SETTLE = "moving_target_settle"
CONF_STILL_TARGET_SETTLE = "still_target_settle"

def _validate_settle(value):
    value = cv.time_period(value)
    ms = value.total_milliseconds
    if ms < 0 or ms > 60000:
        raise cv.Invalid("Settle must be between 0ms and 60s")
    return value


_SETTLE_SCHEMA = _validate_settle


def _entity_with_settle(entity_config, settle):
    if entity_config is None:
        return None
    if CONF_FILTERS in entity_config:
        return entity_config
    return {**entity_config, CONF_FILTERS: [{"settle": settle}]}


CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(LD2402BinarySensor),
            cv.GenerateID(CONF_LD2402_ID): cv.use_id(LD2402Component),
            cv.Optional(CONF_PRESENCE_SETTLE, default="1s"): _SETTLE_SCHEMA,
            cv.Optional(CONF_MOVING_TARGET_SETTLE, default="1s"): _SETTLE_SCHEMA,
            cv.Optional(CONF_STILL_TARGET_SETTLE, default="1s"): _SETTLE_SCHEMA,
            cv.Optional(CONF_HAS_TARGET): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
                icon=ICON_SHIELD_ACCOUNT,
            ),
            cv.Optional(CONF_HAS_MOVING_TARGET): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_MOTION,
                icon=ICON_TARGET_ACCOUNT,
            ),
            cv.Optional(CONF_HAS_STILL_TARGET): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
                icon=ICON_MEDITATION,
            ),
        }
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if CONF_HAS_TARGET in config:
        sens = await binary_sensor.new_binary_sensor(
            _entity_with_settle(config[CONF_HAS_TARGET], config[CONF_PRESENCE_SETTLE])
        )
        cg.add(var.set_presence_sensor(sens))
    if CONF_HAS_MOVING_TARGET in config:
        sens = await binary_sensor.new_binary_sensor(
            _entity_with_settle(
                config[CONF_HAS_MOVING_TARGET], config[CONF_MOVING_TARGET_SETTLE]
            )
        )
        cg.add(var.set_moving_target_sensor(sens))
    if CONF_HAS_STILL_TARGET in config:
        sens = await binary_sensor.new_binary_sensor(
            _entity_with_settle(
                config[CONF_HAS_STILL_TARGET], config[CONF_STILL_TARGET_SETTLE]
            )
        )
        cg.add(var.set_still_target_sensor(sens))
    ld2402 = await cg.get_variable(config[CONF_LD2402_ID])
    cg.add(ld2402.register_listener(var))
