import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ICON, ENTITY_CATEGORY_DIAGNOSTIC

from .cover import CONF_DOOYA_ID, DooyaCover

DEPENDENCIES = ["cover.dooya"]

ICON_INFORMATION_OUTLINE = "mdi:information-outline"

CONFIG_SCHEMA = text_sensor.text_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.GenerateID(CONF_DOOYA_ID): cv.use_id(DooyaCover),
        cv.Optional(CONF_ICON, default=ICON_INFORMATION_OUTLINE): cv.icon,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DOOYA_ID])
    var = await text_sensor.new_text_sensor(config)

    cg.add(parent.set_address_change_status_text_sensor(var))
