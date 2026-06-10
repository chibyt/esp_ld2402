import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@descipher"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

ld2402_ns = cg.esphome_ns.namespace("ld2402")
LD2402Component = ld2402_ns.class_("LD2402Component", cg.Component, uart.UARTDevice)

CONF_LD2402_ID = "ld2402_id"
CONF_AUTO_TRIGGER_COEFFICIENT = "auto_trigger_coefficient"
CONF_AUTO_HOLD_COEFFICIENT = "auto_hold_coefficient"
CONF_AUTO_MICRO_COEFFICIENT = "auto_micro_coefficient"
CONF_PRESENCE_TIMEOUT = "presence_timeout"
CONF_MAX_GATE_DISTANCE = "max_gate_distance"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2402Component),
            cv.Optional(CONF_AUTO_TRIGGER_COEFFICIENT, default=3.0): cv.float_range(
                min=1.0, max=20.0
            ),
            cv.Optional(CONF_AUTO_HOLD_COEFFICIENT, default=3.0): cv.float_range(
                min=1.0, max=20.0
            ),
            cv.Optional(CONF_AUTO_MICRO_COEFFICIENT, default=3.0): cv.float_range(
                min=1.0, max=20.0
            ),
            cv.Optional(CONF_PRESENCE_TIMEOUT, default=30): cv.All(
                cv.int_, cv.Range(min=30, max=120)
            ),
            cv.Optional(CONF_MAX_GATE_DISTANCE, default=85): cv.All(
                cv.int_, cv.Range(min=7, max=100)
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2402_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_auto_trigger_coefficient(config[CONF_AUTO_TRIGGER_COEFFICIENT]))
    cg.add(var.set_auto_hold_coefficient(config[CONF_AUTO_HOLD_COEFFICIENT]))
    cg.add(var.set_auto_micro_coefficient(config[CONF_AUTO_MICRO_COEFFICIENT]))
    cg.add(var.set_presence_timeout(config[CONF_PRESENCE_TIMEOUT]))
    cg.add(var.set_max_gate_distance(config[CONF_MAX_GATE_DISTANCE]))
