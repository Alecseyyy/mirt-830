"""Mirtek CC1101 — ESPHome 2026.5.x, uses spi::SPIDevice"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ENERGY, DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER, DEVICE_CLASS_FREQUENCY, DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_TEMPERATURE, DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY, DEVICE_CLASS_SAFETY,
    STATE_CLASS_MEASUREMENT, STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS, UNIT_VOLT, UNIT_AMPERE, UNIT_WATT,
    UNIT_HERTZ, UNIT_CELSIUS, UNIT_EMPTY, UNIT_PERCENT,
    UNIT_KILOVOLT_AMPS_REACTIVE, UNIT_KILOVOLT_AMPS_REACTIVE_HOURS,
    UNIT_VOLT_AMPS, ICON_FLASH, ICON_THERMOMETER,
)

CODEOWNERS   = []
DEPENDENCIES = ["spi", "sensor", "text_sensor", "binary_sensor"]
AUTO_LOAD    = ["sensor", "text_sensor", "binary_sensor"]

CONF_GDO0_PIN   = "gdo0_pin"
CONF_METER_ADDR = "meter_address"

CONF_S_SUM,  CONF_S_T1,   CONF_S_T2,   CONF_S_T3   = "energy_sum",    "energy_t1",   "energy_t2",   "energy_t3"
CONF_S_SR,   CONF_S_T1R,  CONF_S_T2R,  CONF_S_T3R  = "reactive_sum",  "reactive_t1", "reactive_t2", "reactive_t3"
CONF_S_KW,   CONF_S_KVAR, CONF_S_FREQ, CONF_S_COS  = "power_active",  "power_reactive", "frequency", "power_factor"
CONF_S_V1,   CONF_S_V2,   CONF_S_V3                = "voltage_1",     "voltage_2",   "voltage_3"
CONF_S_I1,   CONF_S_I2,   CONF_S_I3                = "current_1",     "current_2",   "current_3"
CONF_S_PA,   CONF_S_PB,   CONF_S_PC                = "power_a",       "power_b",     "power_c"
CONF_S_QA,   CONF_S_QB,   CONF_S_QC                = "reactive_a",    "reactive_b",  "reactive_c"
CONF_S_SA,   CONF_S_SB,   CONF_S_SC                = "apparent_a",    "apparent_b",  "apparent_c"
CONF_S_CA,   CONF_S_CB,   CONF_S_CC                = "pf_a",          "pf_b",        "pf_c"
CONF_S_TEMP, CONF_S_BAT                            = "temperature",   "battery"

CONF_T_TARIFF, CONF_T_RELAY,  CONF_T_SEAL,  CONF_T_TYPE  = "tariff",      "relay_state",  "seal_state",   "meter_type"
CONF_T_FW,     CONF_T_DATE,   CONF_T_TIME,  CONF_T_WORK  = "fw_version",  "meter_date",   "meter_time",   "uptime_meter"
CONF_T_SYNC,   CONF_T_SER,    CONF_T_ABON,  CONF_T_STAT  = "last_sync",   "serial_number","subscriber",   "status"

CONF_B_3PH, CONF_B_RELAY, CONF_B_SEAL, CONF_B_CC = "three_phase", "relay_on", "seal_ok", "cc1101_ok"

mirtek_ns    = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_("MirtekCC1101", cg.PollingComponent, spi.SPIDevice)

def _s(unit, dec, dc=None, sc=STATE_CLASS_MEASUREMENT, icon=None):
    kw = dict(unit_of_measurement=unit, accuracy_decimals=dec, state_class=sc)
    if dc:   kw["device_class"] = dc
    if icon: kw["icon"] = icon
    return sensor.sensor_schema(**kw)

def _energy(unit=UNIT_KILOWATT_HOURS, icon=ICON_FLASH):
    return sensor.sensor_schema(
        unit_of_measurement=unit, accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING, icon=icon)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID():               cv.declare_id(MirtekCC1101),
    # ── ИСПРАВЛЕНИЕ: используем pins.gpio_input_pin_schema вместо raw int ──
    cv.Required(CONF_GDO0_PIN):    pins.gpio_input_pin_schema,
    cv.Required(CONF_METER_ADDR):  cv.int_range(min=1, max=65000),

    cv.Optional(CONF_S_SUM):  _energy(),
    cv.Optional(CONF_S_T1):   _energy(icon="mdi:weather-sunny"),
    cv.Optional(CONF_S_T2):   _energy(icon="mdi:weather-night"),
    cv.Optional(CONF_S_T3):   _energy(icon="mdi:weather-sunset"),
    cv.Optional(CONF_S_SR):   _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T1R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T2R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T3R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),

    cv.Optional(CONF_S_KW):   _s(UNIT_WATT,  0, DEVICE_CLASS_POWER,     icon=ICON_FLASH),
    cv.Optional(CONF_S_KVAR): _s(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
    cv.Optional(CONF_S_FREQ): _s(UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY),
    cv.Optional(CONF_S_COS):  _s(UNIT_EMPTY, 3, DEVICE_CLASS_POWER_FACTOR),
    cv.Optional(CONF_S_V1):   _s(UNIT_VOLT,   1, DEVICE_CLASS_VOLTAGE),
    cv.Optional(CONF_S_V2):   _s(UNIT_VOLT,   1, DEVICE_CLASS_VOLTAGE),
    cv.Optional(CONF_S_V3):   _s(UNIT_VOLT,   1, DEVICE_CLASS_VOLTAGE),
    cv.Optional(CONF_S_I1):   _s(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    cv.Optional(CONF_S_I2):   _s(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    cv.Optional(CONF_S_I3):   _s(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    cv.Optional(CONF_S_PA):   _s(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    cv.Optional(CONF_S_PB):   _s(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    cv.Optional(CONF_S_PC):   _s(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    cv.Optional(CONF_S_QA):   _s(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    cv.Optional(CONF_S_QB):   _s(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    cv.Optional(CONF_S_QC):   _s(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    cv.Optional(CONF_S_SA):   _s(UNIT_VOLT_AMPS, 0),
    cv.Optional(CONF_S_SB):   _s(UNIT_VOLT_AMPS, 0),
    cv.Optional(CONF_S_SC):   _s(UNIT_VOLT_AMPS, 0),
    cv.Optional(CONF_S_CA):   _s(UNIT_EMPTY, 3),
    cv.Optional(CONF_S_CB):   _s(UNIT_EMPTY, 3),
    cv.Optional(CONF_S_CC):   _s(UNIT_EMPTY, 3),
    cv.Optional(CONF_S_TEMP): _s(UNIT_CELSIUS, 0, DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_S_BAT):  _s(UNIT_PERCENT,  0, DEVICE_CLASS_BATTERY),

    cv.Optional(CONF_T_TARIFF): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_RELAY):  text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SEAL):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_TYPE):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_FW):     text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_DATE):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_TIME):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_WORK):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SYNC):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SER):    text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_ABON):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_STAT):   text_sensor.text_sensor_schema(),

    cv.Optional(CONF_B_3PH):  binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_B_RELAY):binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_POWER),
    cv.Optional(CONF_B_SEAL): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_SAFETY),
    cv.Optional(CONF_B_CC):   binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_CONNECTIVITY),
# spi_device_schema добавляет cs_pin + spi_id
}).extend(spi.spi_device_schema(cs_pin_required=True)).extend(cv.polling_component_schema("300s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    # ── ИСПРАВЛЕНИЕ: передаём config[CONF_GDO0_PIN] напрямую — ──────────────
    # он уже прошёл через pins.gpio_input_pin_schema и содержит поле 'id'
    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0))
    cg.add(var.set_meter_address(config[CONF_METER_ADDR]))

    SENSORS = [
        CONF_S_SUM, CONF_S_T1,  CONF_S_T2,  CONF_S_T3,
        CONF_S_SR,  CONF_S_T1R, CONF_S_T2R, CONF_S_T3R,
        CONF_S_KW,  CONF_S_KVAR,CONF_S_FREQ,CONF_S_COS,
        CONF_S_V1,  CONF_S_V2,  CONF_S_V3,
        CONF_S_I1,  CONF_S_I2,  CONF_S_I3,
        CONF_S_PA,  CONF_S_PB,  CONF_S_PC,
        CONF_S_QA,  CONF_S_QB,  CONF_S_QC,
        CONF_S_SA,  CONF_S_SB,  CONF_S_SC,
        CONF_S_CA,  CONF_S_CB,  CONF_S_CC,
        CONF_S_TEMP,CONF_S_BAT,
    ]
    for idx, key in enumerate(SENSORS):
        if cfg := config.get(key):
            s = await sensor.new_sensor(cfg)
            cg.add(var.set_sensor(idx, s))

    TEXT = [CONF_T_TARIFF, CONF_T_RELAY, CONF_T_SEAL, CONF_T_TYPE,
            CONF_T_FW,     CONF_T_DATE,  CONF_T_TIME, CONF_T_WORK,
            CONF_T_SYNC,   CONF_T_SER,   CONF_T_ABON, CONF_T_STAT]
    for idx, key in enumerate(TEXT):
        if cfg := config.get(key):
            ts = await text_sensor.new_text_sensor(cfg)
            cg.add(var.set_text_sensor(idx, ts))

    BINARY = [CONF_B_3PH, CONF_B_RELAY, CONF_B_SEAL, CONF_B_CC]
    for idx, key in enumerate(BINARY):
        if cfg := config.get(key):
            bs = await binary_sensor.new_binary_sensor(cfg)
            cg.add(var.set_binary_sensor(idx, bs))
