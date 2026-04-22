import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MOVE_THRESHOLD,
    CONF_STILL_THRESHOLD,
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_CONFIG,
    ICON_MOTION_SENSOR,
    ICON_SCALE,
    ICON_TIMELAPSE,
    UNIT_SECOND,
)

from .. import CONF_LD2402_ID, LD2402Component, ld2402_ns

LD2402TimeoutNumber = ld2402_ns.class_("LD2402TimeoutNumber", number.Number)
LD2402MoveSensFactorNumber = ld2402_ns.class_(
    "LD2402MoveSensFactorNumber", number.Number
)
LD2402StillSensFactorNumber = ld2402_ns.class_(
    "LD2402StillSensFactorNumber", number.Number
)
LD2402MaxDistanceNumber = ld2402_ns.class_("LD2402MaxDistanceNumber", number.Number)
LD2402GateSelectNumber = ld2402_ns.class_("LD2402GateSelectNumber", number.Number)
LD2402MoveThresholdNumbers = ld2402_ns.class_(
    "LD2402MoveThresholdNumbers", number.Number
)
LD2402StillThresholdNumbers = ld2402_ns.class_(
    "LD2402StillThresholdNumbers", number.Number
)
CONF_MAX_GATE_DISTANCE = "max_gate_distance"
CONF_GATE_MOVE_SENSITIVITY = "gate_move_sensitivity"
CONF_GATE_STILL_SENSITIVITY = "gate_still_sensitivity"
CONF_GATE_SELECT = "gate_select"
CONF_PRESENCE_TIMEOUT = "presence_timeout"
GATE_GROUP = "gate_group"
TIMEOUT_GROUP = "timeout_group"


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2402_ID): cv.use_id(LD2402Component),
        cv.Inclusive(CONF_PRESENCE_TIMEOUT, TIMEOUT_GROUP): number.number_schema(
            LD2402TimeoutNumber,
            unit_of_measurement=UNIT_SECOND,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_TIMELAPSE,
        ),
        cv.Inclusive(CONF_MAX_GATE_DISTANCE, TIMEOUT_GROUP): number.number_schema(
            LD2402MaxDistanceNumber,
            device_class=DEVICE_CLASS_DISTANCE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_MOTION_SENSOR,
        ),
        cv.Inclusive(CONF_GATE_SELECT, GATE_GROUP): number.number_schema(
            LD2402GateSelectNumber,
            device_class=DEVICE_CLASS_DISTANCE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_MOTION_SENSOR,
        ),
        cv.Inclusive(CONF_STILL_THRESHOLD, GATE_GROUP): number.number_schema(
            LD2402StillThresholdNumbers,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_MOTION_SENSOR,
        ),
        cv.Inclusive(CONF_MOVE_THRESHOLD, GATE_GROUP): number.number_schema(
            LD2402MoveThresholdNumbers,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_MOTION_SENSOR,
        ),
        cv.Optional(CONF_GATE_MOVE_SENSITIVITY): number.number_schema(
            LD2402MoveSensFactorNumber,
            device_class=DEVICE_CLASS_DISTANCE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_SCALE,
        ),
        cv.Optional(CONF_GATE_STILL_SENSITIVITY): number.number_schema(
            LD2402StillSensFactorNumber,
            device_class=DEVICE_CLASS_DISTANCE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_SCALE,
        ),
    }
)
CONFIG_SCHEMA = CONFIG_SCHEMA.extend(
    {
        cv.Optional(f"gate_{x}"): (
            {
                cv.Required(CONF_MOVE_THRESHOLD): number.number_schema(
                    LD2402MoveThresholdNumbers,
                    entity_category=ENTITY_CATEGORY_CONFIG,
                    icon=ICON_MOTION_SENSOR,
                ),
                cv.Required(CONF_STILL_THRESHOLD): number.number_schema(
                    LD2402StillThresholdNumbers,
                    entity_category=ENTITY_CATEGORY_CONFIG,
                    icon=ICON_MOTION_SENSOR,
                ),
            }
        )
        for x in range(16)
    }
)


async def to_code(config):
    LD2402_component = await cg.get_variable(config[CONF_LD2402_ID])
    if gate_timeout_config := config.get(CONF_PRESENCE_TIMEOUT):
        n = await number.new_number(
            gate_timeout_config, min_value=30, max_value=120, step=5
        )
        await cg.register_parented(n, config[CONF_LD2402_ID])
        cg.add(LD2402_component.set_gate_timeout_number(n))
    if max_distance_gate_config := config.get(CONF_MAX_GATE_DISTANCE):
        n = await number.new_number(
            max_distance_gate_config, min_value=7, max_value=100, step=1
        )
        await cg.register_parented(n, config[CONF_LD2402_ID])
        cg.add(LD2402_component.set_max_gate_distance_number(n))
    if gate_move_sensitivity_config := config.get(CONF_GATE_MOVE_SENSITIVITY):
        n = await number.new_number(
            gate_move_sensitivity_config, min_value=0.05, max_value=1, step=0.025
        )
        await cg.register_parented(n, config[CONF_LD2402_ID])
        cg.add(LD2402_component.set_gate_move_sensitivity_factor_number(n))
    if gate_still_sensitivity_config := config.get(CONF_GATE_STILL_SENSITIVITY):
        n = await number.new_number(
            gate_still_sensitivity_config, min_value=0.05, max_value=1, step=0.025
        )
        await cg.register_parented(n, config[CONF_LD2402_ID])
        cg.add(LD2402_component.set_gate_still_sensitivity_factor_number(n))
    if config.get(CONF_GATE_SELECT):
        if gate_number := config.get(CONF_GATE_SELECT):
            n = await number.new_number(gate_number, min_value=0, max_value=15, step=1)
            await cg.register_parented(n, config[CONF_LD2402_ID])
            cg.add(LD2402_component.set_gate_select_number(n))
        if gate_still_threshold := config.get(CONF_STILL_THRESHOLD):
            n = cg.new_Pvariable(gate_still_threshold[CONF_ID])
            await number.register_number(
                n, gate_still_threshold, min_value=0, max_value=65535, step=25
            )
            await cg.register_parented(n, config[CONF_LD2402_ID])
            cg.add(LD2402_component.set_gate_still_threshold_numbers(0, n))
        if gate_move_threshold := config.get(CONF_MOVE_THRESHOLD):
            n = cg.new_Pvariable(gate_move_threshold[CONF_ID])
            await number.register_number(
                n, gate_move_threshold, min_value=0, max_value=65535, step=25
            )
            await cg.register_parented(n, config[CONF_LD2402_ID])
            cg.add(LD2402_component.set_gate_move_threshold_numbers(0, n))
    else:
        for x in range(16):
            if gate_conf := config.get(f"gate_{x}"):
                move_config = gate_conf[CONF_MOVE_THRESHOLD]
                n = cg.new_Pvariable(move_config[CONF_ID], x)
                await number.register_number(
                    n, move_config, min_value=0, max_value=65535, step=25
                )
                await cg.register_parented(n, config[CONF_LD2402_ID])
                cg.add(LD2402_component.set_gate_move_threshold_numbers(x, n))

                still_config = gate_conf[CONF_STILL_THRESHOLD]
                n = cg.new_Pvariable(still_config[CONF_ID], x)
                await number.register_number(
                    n, still_config, min_value=0, max_value=65535, step=25
                )
                await cg.register_parented(n, config[CONF_LD2402_ID])
                cg.add(LD2402_component.set_gate_still_threshold_numbers(x, n))
