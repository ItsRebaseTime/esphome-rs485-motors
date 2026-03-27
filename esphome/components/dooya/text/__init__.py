import esphome.codegen as cg
from esphome.components import text
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from ..cover import CONF_DOOYA_ID, DooyaCover, dooya_ns

DEPENDENCIES = ["cover.dooya"]

DooyaAddressText = dooya_ns.class_(
    "DooyaAddressText",
    text.Text,
    cg.Component,
    cg.Parented.template(DooyaCover),
)

CONFIG_SCHEMA = text.text_schema(
    DooyaAddressText,
    entity_category=ENTITY_CATEGORY_CONFIG,
    mode="TEXT",
).extend(
    {
        cv.GenerateID(CONF_DOOYA_ID): cv.use_id(DooyaCover),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DOOYA_ID])
    var = await text.new_text(config, min_length=4, max_length=32)

    await cg.register_component(var, config)
    await cg.register_parented(var, parent)
    cg.add(parent.set_address_change_text(var))
