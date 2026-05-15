#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/spi/spi.h"

#include <vector>
#include <string>

namespace esphome {
namespace mirtek_cc1101 {

// CRC8 calculation with polynomial 0xA9
static uint8_t crc8_a9(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t idx = 0; idx < len; idx++) {
    uint8_t b = data[idx];
    for (int i = 0; i < 8; i++) {
      if (((b ^ crc) & 0x80) == 0) {
        crc = (uint8_t)(crc << 1);
      } else {
        crc = (uint8_t)((crc << 1) ^ 0xA9);
      }
      b = (uint8_t)(b << 1);
    }
  }
  return crc;
}

// CC1101 SPI register addresses
static const uint8_t CC1101_IOCFG2   = 0x00;
static const uint8_t CC1101_IOCFG0   = 0x02;
static const uint8_t CC1101_FIFOTHR  = 0x03;
static const uint8_t CC1101_SYNC1    = 0x04;
static const uint8_t CC1101_SYNC0    = 0x05;
static const uint8_t CC1101_PKTLEN   = 0x06;
static const uint8_t CC1101_PKTCTRL1 = 0x07;
static const uint8_t CC1101_PKTCTRL0 = 0x08;
static const uint8_t CC1101_ADDR     = 0x09;
static const uint8_t CC1101_CHANNR   = 0x0A;
static const uint8_t CC1101_FSCTRL1  = 0x0B;
static const uint8_t CC1101_FSCTRL0  = 0x0C;
static const uint8_t CC1101_FREQ2    = 0x0D;
static const uint8_t CC1101_FREQ1    = 0x0E;
static const uint8_t CC1101_FREQ0    = 0x0F;
static const uint8_t CC1101_MDMCFG4  = 0x10;
static const uint8_t CC1101_MDMCFG3  = 0x11;
static const uint8_t CC1101_MDMCFG2  = 0x12;
static const uint8_t CC1101_MDMCFG1  = 0x13;
static const uint8_t CC1101_MDMCFG0  = 0x14;
static const uint8_t CC1101_DEVIATN  = 0x15;
static const uint8_t CC1101_MCSM2    = 0x16;
static const uint8_t CC1101_MCSM1    = 0x17;
static const uint8_t CC1101_MCSM0    = 0x18;
static const uint8_t CC1101_FOCCFG   = 0x19;
static const uint8_t CC1101_BSCFG    = 0x1A;
static const uint8_t CC1101_AGCCTRL2 = 0x1B;
static const uint8_t CC1101_AGCCTRL1 = 0x1C;
static const uint8_t CC1101_AGCCTRL0 = 0x1D;
static const uint8_t CC1101_WOREVT1  = 0x1E;
static const uint8_t CC1101_WOREVT0  = 0x1F;
static const uint8_t CC1101_WORCTRL  = 0x20;
static const uint8_t CC1101_FREND1   = 0x21;
static const uint8_t CC1101_FREND0   = 0x22;
static const uint8_t CC1101_FSCAL3   = 0x23;
static const uint8_t CC1101_FSCAL2   = 0x24;
static const uint8_t CC1101_FSCAL1   = 0x25;
static const uint8_t CC1101_FSCAL0   = 0x26;
static const uint8_t CC1101_RCCTRL1  = 0x27;
static const uint8_t CC1101_RCCTRL0  = 0x28;
static const uint8_t CC1101_FSTEST   = 0x29;
static const uint8_t CC1101_PTEST    = 0x2A;
static const uint8_t CC1101_AGCTEST  = 0x2B;
static const uint8_t CC1101_TEST2    = 0x2C;
static const uint8_t CC1101_TEST1    = 0x2D;
static const uint8_t CC1101_TEST0    = 0x2E;

// CC1101 Status registers (read-only, access via 0xC0 | addr)
static const uint8_t CC1101_PARTNUM  = 0x30;
static const uint8_t CC1101_VERSION  = 0x31;
static const uint8_t CC1101_RXBYTES  = 0x3B;
static const uint8_t CC1101_TXBYTES  = 0x3A;

// CC1101 command strobes
static const uint8_t CC1101_SRES  = 0x30;
static const uint8_t CC1101_SCAL  = 0x33;
static const uint8_t CC1101_SRX   = 0x34;
static const uint8_t CC1101_STX   = 0x35;
static const uint8_t CC1101_SIDLE = 0x36;
static const uint8_t CC1101_SFRX  = 0x3A;
static const uint8_t CC1101_SFTX  = 0x3B;

// CC1101 FIFO access
static const uint8_t CC1101_TXFIFO = 0x3F;
static const uint8_t CC1101_RXFIFO = 0x3F;
static const uint8_t CC1101_BURST  = 0x40;
static const uint8_t CC1101_READ   = 0x80;
static const uint8_t CC1101_PA_TABLE = 0x3E;

// Mirtek RF settings for 433 MHz
static const uint8_t MIRTEK_RF_SETTINGS[] = {
  0x0D,  // IOCFG2
  0x2E,  // IOCFG1
  0x06,  // IOCFG0
  0x4F,  // FIFOTHR
  0xD3,  // SYNC1
  0x91,  // SYNC0
  0x3C,  // PKTLEN
  0x00,  // PKTCTRL1
  0x41,  // PKTCTRL0
  0x00,  // ADDR
  0x16,  // CHANNR
  0x0F,  // FSCTRL1
  0x00,  // FSCTRL0
  0x10,  // FREQ2
  0x8B,  // FREQ1
  0x54,  // FREQ0
  0xD9,  // MDMCFG4
  0x83,  // MDMCFG3
  0x13,  // MDMCFG2
  0xD2,  // MDMCFG1
  0xAA,  // MDMCFG0
  0x31,  // DEVIATN
  0x07,  // MCSM2
  0x0C,  // MCSM1
  0x08,  // MCSM0
  0x16,  // FOCCFG
  0x6C,  // BSCFG
  0x03,  // AGCCTRL2
  0x40,  // AGCCTRL1
  0x91,  // AGCCTRL0
  0x87,  // WOREVT1
  0x6B,  // WOREVT0
  0xF8,  // WORCTRL
  0x56,  // FREND1
  0x10,  // FREND0
  0xE9,  // FSCAL3
  0x2A,  // FSCAL2
  0x00,  // FSCAL1
  0x1F,  // FSCAL0
  0x41,  // RCCTRL1
  0x00,  // RCCTRL0
  0x59,  // FSTEST
  0x59,  // PTEST
  0x3F,  // AGCTEST
  0x81,  // TEST2
  0x35,  // TEST1
  0x09,  // TEST0
};

class MirtekCC1101 : public PollingComponent {
 public:
  void set_cs_pin(GPIOPin *cs) { cs_pin_ = cs; }
  void set_gdo0_pin(GPIOPin *gdo0) { gdo0_pin_ = gdo0; }
  void set_meter_address(uint16_t addr) { meter_address_ = addr; }
  void set_poll_interval_min(uint32_t min) { poll_interval_min_ = min; }

  // Energy sensors (3 tariffs + sum)
  void set_energy_sum(sensor::Sensor *s) { energy_sum_ = s; }
  void set_energy_t1(sensor::Sensor *s) { energy_t1_ = s; }
  void set_energy_t2(sensor::Sensor *s) { energy_t2_ = s; }
  void set_energy_t3(sensor::Sensor *s) { energy_t3_ = s; }

  // Instantaneous measurement sensors
  void set_voltage_a(sensor::Sensor *s) { voltage_a_ = s; }
  void set_voltage_b(sensor::Sensor *s) { voltage_b_ = s; }
  void set_voltage_c(sensor::Sensor *s) { voltage_c_ = s; }
  void set_current_a(sensor::Sensor *s) { current_a_ = s; }
  void set_current_b(sensor::Sensor *s) { current_b_ = s; }
  void set_current_c(sensor::Sensor *s) { current_c_ = s; }
  void set_power_active(sensor::Sensor *s) { power_active_ = s; }
  void set_power_reactive(sensor::Sensor *s) { power_reactive_ = s; }
  void set_frequency(sensor::Sensor *s) { frequency_ = s; }
  void set_power_factor(sensor::Sensor *s) { power_factor_ = s; }
  void set_temperature(sensor::Sensor *s) { temperature_ = s; }

  // Per-phase power sensors
  void set_power_a(sensor::Sensor *s) { power_a_ = s; }
  void set_power_b(sensor::Sensor *s) { power_b_ = s; }
  void set_power_c(sensor::Sensor *s) { power_c_ = s; }
  void set_reactive_a(sensor::Sensor *s) { reactive_a_ = s; }
  void set_reactive_b(sensor::Sensor *s) { reactive_b_ = s; }
  void set_reactive_c(sensor::Sensor *s) { reactive_c_ = s; }
  void set_apparent_a(sensor::Sensor *s) { apparent_a_ = s; }
  void set_apparent_b(sensor::Sensor *s) { apparent_b_ = s; }
  void set_apparent_c(sensor::Sensor *s) { apparent_c_ = s; }
  void set_pf_a(sensor::Sensor *s) { pf_a_ = s; }
  void set_pf_b(sensor::Sensor *s) { pf_b_ = s; }
  void set_pf_c(sensor::Sensor *s) { pf_c_ = s; }

  // Text sensors
  void set_tariff_text(text_sensor::TextSensor *s) { tariff_text_ = s; }
  void set_relay_state(text_sensor::TextSensor *s) { relay_state_ = s; }
  void set_seal_state(text_sensor::TextSensor *s) { seal_state_ = s; }
  void set_hw_version(text_sensor::TextSensor *s) { hw_version_ = s; }
  void set_sw_version(text_sensor::TextSensor *s) { sw_version_ = s; }
  void set_meter_type(text_sensor::TextSensor *s) { meter_type_ = s; }
  void set_meter_datetime(text_sensor::TextSensor *s) { meter_datetime_ = s; }
  void set_last_status(text_sensor::TextSensor *s) { last_status_ = s; }

  // Binary sensors (seals / status flags)
  void set_seal_cover(binary_sensor::BinarySensor *s) { seal_cover_ = s; }
  void set_seal_terminal(binary_sensor::BinarySensor *s) { seal_terminal_ = s; }
  void set_seal_module(binary_sensor::BinarySensor *s) { seal_module_ = s; }
  void set_magnet_dc(binary_sensor::BinarySensor *s) { magnet_dc_ = s; }
  void set_magnet_ac(binary_sensor::BinarySensor *s) { magnet_ac_ = s; }
  void set_relay_on(binary_sensor::BinarySensor *s) { relay_on_ = s; }
  void set_current_imbalance(binary_sensor::BinarySensor *s) { current_imbalance_ = s; }
  void set_time_synced(binary_sensor::BinarySensor *s) { time_synced_ = s; }

  void setup() override;
  void update() override;
  void dump_config() override;

  // Public data for display use
  float energy_sum_val{NAN}, energy_t1_val{NAN}, energy_t2_val{NAN}, energy_t3_val{NAN};
  float va{NAN}, vb{NAN}, vc{NAN};
  float ia{NAN}, ib{NAN}, ic{NAN};
  float p_total{NAN}, q_total{NAN}, freq{NAN}, pf{NAN};
  float temp{NAN};
  float pa{NAN}, pb{NAN}, pc{NAN};
  std::string tariff_str{"N/A"};
  std::string relay_str{"N/A"};
  std::string seal_str{"N/A"};
  std::string hw_str{"N/A"};
  std::string sw_str{"N/A"};
  std::string meter_type_str{"Не определён"};
  std::string datetime_str{"N/A"};
  bool three_phase{true};

 protected:
  // SPI helpers
  void spi_begin_();
  void spi_end_();
  void cc1101_reset_();
  void cc1101_write_reg_(uint8_t addr, uint8_t val);
  void cc1101_write_burst_(uint8_t addr, const uint8_t *data, size_t len);
  uint8_t cc1101_read_reg_(uint8_t addr);
  uint8_t cc1101_strobe_(uint8_t cmd);
  bool cc1101_init_();
  bool cc1101_check_();

  // RF packet helpers
  void build_short_packet_(uint8_t cmd, uint8_t packet_type);
  void build_long_packet_(uint8_t cmd, uint8_t sub, uint8_t packet_type);
  void build_long2b_packet_(uint8_t cmd, uint8_t sub1, uint8_t sub2, uint8_t packet_type);
  bool send_packet_();
  bool receive_packet_();
  void apply_byte_stuffing_(const uint8_t *in, size_t in_len, std::vector<uint8_t> &out);
  bool remove_byte_stuffing_(const uint8_t *in, size_t in_len, std::vector<uint8_t> &out);

  // Parsers
  bool parse_datetime_();
  bool parse_energy_();
  bool parse_voltage_current_();
  bool parse_power_();
  bool parse_extended_info_();

  // Publish helpers
  void publish_sensor_(sensor::Sensor *s, float val);
  void publish_text_(text_sensor::TextSensor *s, const std::string &val);
  void publish_binary_(binary_sensor::BinarySensor *s, bool val);

  // Internal state
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *gdo0_pin_{nullptr};
  uint16_t meter_address_{1};
  uint32_t poll_interval_min_{5};

  uint8_t send_buf_[20]{};
  uint8_t recv_buf_[60]{};
  uint8_t result_buf_[60]{};
  size_t result_len_{0};
  uint8_t packet_type_{0};
  uint8_t my_crc_{0};

  uint32_t extended_poll_count_{0};

  // Sensors
  sensor::Sensor *energy_sum_{nullptr};
  sensor::Sensor *energy_t1_{nullptr};
  sensor::Sensor *energy_t2_{nullptr};
  sensor::Sensor *energy_t3_{nullptr};
  sensor::Sensor *voltage_a_{nullptr};
  sensor::Sensor *voltage_b_{nullptr};
  sensor::Sensor *voltage_c_{nullptr};
  sensor::Sensor *current_a_{nullptr};
  sensor::Sensor *current_b_{nullptr};
  sensor::Sensor *current_c_{nullptr};
  sensor::Sensor *power_active_{nullptr};
  sensor::Sensor *power_reactive_{nullptr};
  sensor::Sensor *frequency_{nullptr};
  sensor::Sensor *power_factor_{nullptr};
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *power_a_{nullptr};
  sensor::Sensor *power_b_{nullptr};
  sensor::Sensor *power_c_{nullptr};
  sensor::Sensor *reactive_a_{nullptr};
  sensor::Sensor *reactive_b_{nullptr};
  sensor::Sensor *reactive_c_{nullptr};
  sensor::Sensor *apparent_a_{nullptr};
  sensor::Sensor *apparent_b_{nullptr};
  sensor::Sensor *apparent_c_{nullptr};
  sensor::Sensor *pf_a_{nullptr};
  sensor::Sensor *pf_b_{nullptr};
  sensor::Sensor *pf_c_{nullptr};

  text_sensor::TextSensor *tariff_text_{nullptr};
  text_sensor::TextSensor *relay_state_{nullptr};
  text_sensor::TextSensor *seal_state_{nullptr};
  text_sensor::TextSensor *hw_version_{nullptr};
  text_sensor::TextSensor *sw_version_{nullptr};
  text_sensor::TextSensor *meter_type_{nullptr};
  text_sensor::TextSensor *meter_datetime_{nullptr};
  text_sensor::TextSensor *last_status_{nullptr};

  binary_sensor::BinarySensor *seal_cover_{nullptr};
  binary_sensor::BinarySensor *seal_terminal_{nullptr};
  binary_sensor::BinarySensor *seal_module_{nullptr};
  binary_sensor::BinarySensor *magnet_dc_{nullptr};
  binary_sensor::BinarySensor *magnet_ac_{nullptr};
  binary_sensor::BinarySensor *relay_on_{nullptr};
  binary_sensor::BinarySensor *current_imbalance_{nullptr};
  binary_sensor::BinarySensor *time_synced_{nullptr};
};

}  // namespace mirtek_cc1101
}  // namespace esphome
