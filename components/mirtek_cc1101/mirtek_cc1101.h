#pragma once
// =============================================================================
// Mirtek CC1101 ESPHome Component — ESPHome 2026.5.x
// Протокол: Star v1.20 (Mirtek Star 104 / Star 304)
// SPI: bit-bang через GPIOPin — не требует Arduino SPI.h
// =============================================================================
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// ─── CRC8 poly 0xA9 ──────────────────────────────────────────────────────────
static uint8_t crc8_mirtek(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    for (int j = 0; j < 8; j++) {
      c = ((b ^ c) & 0x80) ? (uint8_t)((c << 1) ^ 0xA9) : (uint8_t)(c << 1);
      b <<= 1;
    }
  }
  return c;
}

// ─── CC1101 strobes / registers ──────────────────────────────────────────────
static const uint8_t CC_SRES   = 0x30;
static const uint8_t CC_SCAL   = 0x33;
static const uint8_t CC_SRX    = 0x34;
static const uint8_t CC_STX    = 0x35;
static const uint8_t CC_SIDLE  = 0x36;
static const uint8_t CC_SFRX   = 0x3A;
static const uint8_t CC_SFTX   = 0x3B;
static const uint8_t CC_BURST  = 0x40;
static const uint8_t CC_READ   = 0x80;
static const uint8_t CC_TXFIFO = 0x3F;
static const uint8_t CC_RXFIFO = 0x3F;
static const uint8_t CC_RXBYTES = 0x3B;
static const uint8_t CC_VERSION = 0x31;

// RF config for 433 MHz / FSK / Mirtek protocol
static const uint8_t RF_CFG[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,
  0x41,0x00,0x16,0x0F,0x00,0x10,0x8B,0x54,
  0xD9,0x83,0x13,0xD2,0xAA,0x31,0x07,0x0C,
  0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,
  0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,
  0x00,0x59,0x59,0x3F,0x81,0x35,0x09
};

// ─── Sensor indices (must match SENSORS list in __init__.py to_code) ──────────
enum SI {
  SI_SUM=0, SI_T1, SI_T2, SI_T3,
  SI_SUM_R, SI_T1_R, SI_T2_R, SI_T3_R,
  SI_KW, SI_KVAR, SI_FREQ, SI_COS,
  SI_V1, SI_V2, SI_V3,
  SI_I1, SI_I2, SI_I3,
  SI_PA, SI_PB, SI_PC,
  SI_QA, SI_QB, SI_QC,
  SI_SA, SI_SB, SI_SC,
  SI_CA, SI_CB, SI_CC,
  SI_TEMP, SI_BAT,
  SI_COUNT
};
enum TI { TI_TARIFF=0, TI_RELAY, TI_SEAL, TI_TYPE, TI_FW, TI_DATE, TI_TIME, TI_WORK, TI_SYNC, TI_SERIAL, TI_ABON, TI_STATUS, TI_COUNT };
enum BI { BI_3PHASE=0, BI_RELAY, BI_SEAL_OK, BI_CC1101, BI_COUNT };

// ─── Component ───────────────────────────────────────────────────────────────
class MirtekCC1101 : public PollingComponent {
 public:
  // setters called from __init__.py to_code()
  void set_cs_pin(GPIOPin *p)    { cs_   = p; }
  void set_gdo0_pin(GPIOPin *p)  { gdo0_ = p; }
  void set_clk_pin(GPIOPin *p)   { clk_  = p; }
  void set_mosi_pin(GPIOPin *p)  { mosi_ = p; }
  void set_miso_pin(GPIOPin *p)  { miso_ = p; }
  void set_meter_address(int a)  { addr_ = (uint16_t)a; }
  void set_sensor(int i, sensor::Sensor *s)              { if (i < SI_COUNT) ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s){ if (i < TI_COUNT) ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s){ if (i < BI_COUNT) bs_[i] = s; }

  // ── Setup ──────────────────────────────────────────────────────────────────
  void setup() override {
    ESP_LOGI(TAG, "Setup CC1101, meter addr=%u", addr_);

    cs_->setup();   cs_->digital_write(true);
    clk_->setup();  clk_->digital_write(false);
    mosi_->setup(); mosi_->digital_write(false);
    miso_->setup();
    if (gdo0_) gdo0_->setup();

    bool ok = cc_init_();
    pub_bin_(BI_CC1101, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");
    if (!ok) {
      ESP_LOGE(TAG, "CC1101 not detected! Check wiring.");
    } else {
      ESP_LOGI(TAG, "CC1101 ready. Poll every %u s", get_update_interval() / 1000);
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway:");
    ESP_LOGCONFIG(TAG, "  Meter address : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Poll interval : %u ms", get_update_interval());
    LOG_PIN("  CS   pin: ", cs_);
    LOG_PIN("  GDO0 pin: ", gdo0_);
    LOG_PIN("  CLK  pin: ", clk_);
    LOG_PIN("  MOSI pin: ", mosi_);
    LOG_PIN("  MISO pin: ", miso_);
  }

  void update() override { poll_all(); }

  void poll_all() {
    ESP_LOGI(TAG, "=== Poll addr=%u ===", addr_);
    poll_count_++;

    bool ok1 = do_cmd_(0x1C, -1, -1, 3) && parse_datetime_();
    bool ok2 = do_cmd_(0x05, 0x04, -1, 4) && parse_energy_();
    bool ok3 = do_cmd_(0x2B, 0x00, -1, 4) && parse_instant_();
    bool ok4 = true;
    if (three_phase_) ok4 = do_cmd_(0x2B, 0x10, -1, 4) && parse_phase_();
    if (poll_count_ % 12 == 1) {
      do_cmd_(0x01, -1, -1, 5) && parse_ident_();
      do_cmd_(0x1E, -1, -1, 3) && parse_battery_();
    }

    pub_txt_(TI_STATUS, (ok1 && ok2 && ok3 && ok4) ? "OK" : "PARTIAL");
  }

  void relay_on()  { do_cmd_(0x3A, 0x08, -1, 2); }
  void relay_off() { do_cmd_(0x3A, 0x09, -1, 2); }

 protected:
  GPIOPin *cs_{nullptr}, *gdo0_{nullptr};
  GPIOPin *clk_{nullptr}, *mosi_{nullptr}, *miso_{nullptr};
  uint16_t addr_{1};
  bool three_phase_{true};
  uint32_t poll_count_{0};

  sensor::Sensor          *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};

  uint8_t sbuf_[20]{};
  uint8_t rbuf_[64]{};
  size_t  rlen_{0};

  // ── Bit-bang SPI (MSB first, Mode 0) ─────────────────────────────────────
  void cs_low_()  { cs_->digital_write(false); delayMicroseconds(2); }
  void cs_high_() { cs_->digital_write(true);  delayMicroseconds(2); }

  uint8_t spi_xfer_(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
      mosi_->digital_write((out >> i) & 1);
      clk_->digital_write(true);
      delayMicroseconds(1);
      in = (in << 1) | (miso_->digital_read() ? 1 : 0);
      clk_->digital_write(false);
      delayMicroseconds(1);
    }
    return in;
  }

  void cc_strobe_(uint8_t cmd) {
    cs_low_(); spi_xfer_(cmd); cs_high_();
  }
  void cc_wreg_(uint8_t addr, uint8_t val) {
    cs_low_(); spi_xfer_(addr & 0x3F); spi_xfer_(val); cs_high_();
  }
  void cc_wburst_(uint8_t addr, const uint8_t *d, size_t n) {
    cs_low_(); spi_xfer_((addr & 0x3F) | CC_BURST);
    for (size_t i = 0; i < n; i++) spi_xfer_(d[i]);
    cs_high_();
  }
  uint8_t cc_rreg_(uint8_t addr) {
    cs_low_(); spi_xfer_(CC_READ | (addr & 0x3F)); uint8_t v = spi_xfer_(0); cs_high_();
    return v;
  }
  uint8_t cc_rstat_(uint8_t addr) {
    cs_low_(); spi_xfer_(CC_READ | CC_BURST | (addr & 0x3F)); uint8_t v = spi_xfer_(0); cs_high_();
    return v;
  }

  bool cc_init_() {
    cs_high_(); delayMicroseconds(5);
    cs_low_();  delayMicroseconds(10);
    cs_high_(); delayMicroseconds(41);
    cc_strobe_(CC_SRES); delay(10);
    cc_wburst_(0x00, RF_CFG, sizeof(RF_CFG));
    cc_strobe_(CC_SCAL); delay(2);
    cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
    uint8_t ver = cc_rstat_(CC_VERSION);
    ESP_LOGD(TAG, "CC1101 version=0x%02X", ver);
    return (ver == 0x14 || ver == 0x04);
  }

  // ── Byte stuffing ─────────────────────────────────────────────────────────
  void stuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    out.push_back(in[0]); out.push_back(in[1]); out.push_back(in[2]);
    for (size_t i = 3; i < n - 1; i++) {
      if      (in[i] == 0x55) { out.push_back(0x73); out.push_back(0x11); }
      else if (in[i] == 0x73) { out.push_back(0x73); out.push_back(0x22); }
      else                      out.push_back(in[i]);
    }
    out.push_back(in[n - 1]);
  }
  void destuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    for (size_t i = 0; i < n; i++) {
      if (in[i] == 0x73 && i + 1 < n) {
        if      (in[i+1] == 0x11) { out.push_back(0x55); i++; }
        else if (in[i+1] == 0x22) { out.push_back(0x73); i++; }
        else                         out.push_back(in[i]);
      } else { out.push_back(in[i]); }
    }
  }

  // ── Build request packet ──────────────────────────────────────────────────
  size_t build_pkt_(uint8_t cmd, int sub1, int sub2) {
    uint8_t *b = sbuf_;
    uint8_t dlen = (sub1 < 0) ? 0 : (sub2 < 0) ? 1 : 2;
    b[0] = 0x0F + dlen;
    b[1] = 0x73; b[2] = 0x55;
    b[3] = 0x20 + dlen;
    b[4] = 0x00;
    b[5] = addr_ & 0xFF; b[6] = (addr_ >> 8) & 0xFF;
    b[7] = 0xFE; b[8] = 0xFF;
    b[9] = cmd;
    b[10] = b[11] = b[12] = b[13] = 0x00;
    size_t pos = 14;
    if (sub1 >= 0) b[pos++] = (uint8_t)sub1;
    if (sub2 >= 0) b[pos++] = (uint8_t)sub2;
    b[pos] = crc8_mirtek(b + 3, pos - 3);
    b[pos + 1] = 0x55;
    return pos + 2;
  }

  // ── Send & receive ────────────────────────────────────────────────────────
  bool do_cmd_(uint8_t cmd, int sub1, int sub2, int exp_pkts) {
    size_t raw_len = build_pkt_(cmd, sub1, sub2);
    std::vector<uint8_t> stuffed;
    stuff_(sbuf_, raw_len, stuffed);
    stuffed[0] = (uint8_t)(stuffed.size() - 1);

    cc_strobe_(CC_SIDLE);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SFRX);

    cs_low_();
    spi_xfer_(CC_TXFIFO | CC_BURST);
    for (uint8_t b : stuffed) spi_xfer_(b);
    cs_high_();

    cc_strobe_(CC_STX);

    // Wait TX done via GDO0
    if (gdo0_) {
      uint32_t t0 = millis();
      while (!gdo0_->digital_read() && millis() - t0 < 200) delayMicroseconds(100);
      while ( gdo0_->digital_read() && millis() - t0 < 500) delayMicroseconds(100);
    } else {
      delay(100);
    }

    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SRX);

    std::vector<uint8_t> raw_all;
    int got = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 1200 && got < exp_pkts) {
      uint8_t rxb = cc_rstat_(CC_RXBYTES);
      if (rxb > 0 && rxb < 64) {
        got++;
        cs_low_();
        spi_xfer_(CC_RXFIFO | CC_READ | CC_BURST);
        uint8_t lb = spi_xfer_(0);
        if (lb > 0 && lb < 60) {
          for (uint8_t i = 1; i < lb; i++) raw_all.push_back(spi_xfer_(0));
        }
        cs_high_();
        cc_strobe_(CC_SIDLE);
        cc_strobe_(CC_SFRX);
        cc_strobe_(CC_SFTX);
        cc_strobe_(CC_SRX);
      }
      delayMicroseconds(500);
    }

    if (raw_all.empty()) {
      ESP_LOGW(TAG, "cmd=0x%02X no response", cmd);
      return false;
    }

    std::vector<uint8_t> ds;
    destuff_(raw_all.data(), raw_all.size(), ds);
    if (ds.size() > sizeof(rbuf_)) { ESP_LOGW(TAG, "overflow"); return false; }
    memcpy(rbuf_, ds.data(), ds.size());
    rlen_ = ds.size();

    if (rlen_ < 5 || rbuf_[0] != 0x73 || rbuf_[1] != 0x55) {
      ESP_LOGW(TAG, "bad header"); return false;
    }
    if (rbuf_[4] != (addr_ & 0xFF) || rbuf_[5] != ((addr_ >> 8) & 0xFF)) {
      ESP_LOGW(TAG, "addr mismatch"); return false;
    }
    return true;
  }

  // ── Parse helpers ─────────────────────────────────────────────────────────
  float u16_(size_t i) { return (float)((uint16_t)rbuf_[i] | ((uint16_t)rbuf_[i+1] << 8)); }
  float u32_(size_t i) { return (float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)rbuf_[i+2]<<16) | ((uint32_t)rbuf_[i+3]<<24)); }
  float u24_(size_t i) { return (float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)rbuf_[i+2]<<16)); }
  float s16m_(size_t i, float div) {
    bool neg = rbuf_[i+1] >= 128;
    return ((float)((uint16_t)rbuf_[i] | ((uint16_t)(rbuf_[i+1] & 0x7F) << 8)) / div) * (neg ? -1.f : 1.f);
  }
  float s24m_(size_t i, float div) {
    bool neg = rbuf_[i+2] >= 128;
    return ((float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)(rbuf_[i+2]&0x7F)<<16)) / div) * (neg ? -1.f : 1.f);
  }

  void pub_s_(int i, float v)              { if (ss_[i]) ss_[i]->publish_state(v); }
  void pub_txt_(int i, const std::string &v){ if (ts_[i]) ts_[i]->publish_state(v); }
  void pub_bin_(int i, bool v)             { if (bs_[i]) bs_[i]->publish_state(v); }

  // ── Parsers ───────────────────────────────────────────────────────────────
  bool parse_datetime_() {
    if (rlen_ < 20 || rbuf_[6] != 0x1C) return false;
    uint8_t tp = rbuf_[7];
    three_phase_ = !((tp == 0x98) || (tp == 0x99));
    char tb[32];
    switch (tp) {
      case 0xA8: case 0xA9: snprintf(tb,32,"3ph transf act-react"); break;
      case 0x88: case 0x89: snprintf(tb,32,"3ph transf act bidi");  break;
      case 0x80: case 0x81: snprintf(tb,32,"3ph act bidi");         break;
      case 0x68: case 0x69: snprintf(tb,32,"3ph transf act");       break;
      case 0x98: case 0x99: snprintf(tb,32,"1ph act-react");        break;
      default:               snprintf(tb,32,"type 0x%02X", tp);
    }
    pub_txt_(TI_TYPE, tb);
    pub_bin_(BI_3PHASE, three_phase_);
    if (rlen_ >= 18) {
      char dt[12], tm[10];
      snprintf(tm, 10, "%02d:%02d:%02d", rbuf_[13], rbuf_[12], rbuf_[11]);
      snprintf(dt, 12, "%02d.%02d.%02d",  rbuf_[14], rbuf_[15], rbuf_[16]);
      pub_txt_(TI_TIME, tm);
      pub_txt_(TI_DATE, dt);
    }
    return true;
  }

  bool parse_energy_() {
    if (rlen_ < 35 || rbuf_[6] != 0x05) return false;
    uint8_t cfg = rbuf_[12];
    uint8_t cur_t = (cfg >> 2) & 0x03;
    const char *tn[] = {"T1 Day","T2 Night","T3 Peak","Special"};
    pub_txt_(TI_TARIFF, tn[cur_t]);

    if (rlen_ >= 21) pub_s_(SI_SUM,   u32_(17) / 100.f);
    if (rlen_ >= 29) pub_s_(SI_T1,    u32_(25) / 100.f);
    if (rlen_ >= 33) pub_s_(SI_T2,    u32_(29) / 100.f);
    if (rlen_ >= 37) pub_s_(SI_T3,    u32_(33) / 100.f);
    if (rlen_ >= 53) {
      pub_s_(SI_SUM_R, u32_(37) / 100.f);
      pub_s_(SI_T1_R,  u32_(41) / 100.f);
      pub_s_(SI_T2_R,  u32_(45) / 100.f);
      pub_s_(SI_T3_R,  u32_(49) / 100.f);
    }
    ESP_LOGI(TAG, "kWh SUM=%.2f T1=%.2f T2=%.2f T3=%.2f",
      ss_[SI_SUM] ? ss_[SI_SUM]->state : 0.f,
      ss_[SI_T1]  ? ss_[SI_T1]->state  : 0.f,
      ss_[SI_T2]  ? ss_[SI_T2]->state  : 0.f,
      ss_[SI_T3]  ? ss_[SI_T3]->state  : 0.f);
    return true;
  }

  bool parse_instant_() {
    if (rlen_ < 36 || rbuf_[6] != 0x2B) return false;
    size_t b = 16;
    if (three_phase_) {
      pub_s_(SI_KW,   u24_(b) / 1000.f); b += 3;
      pub_s_(SI_KVAR, s24m_(b, 1000.f)); b += 3;
      pub_s_(SI_FREQ, u16_(b) / 100.f);  b += 2;
      pub_s_(SI_COS,  s16m_(b, 1000.f)); b += 2;
      pub_s_(SI_V1, u16_(b)/100.f); b+=2;
      pub_s_(SI_V2, u16_(b)/100.f); b+=2;
      pub_s_(SI_V3, u16_(b)/100.f); b+=2;
      pub_s_(SI_I1, u24_(b)/1000.f); b+=3;
      pub_s_(SI_I2, u24_(b)/1000.f); b+=3;
      pub_s_(SI_I3, u24_(b)/1000.f); b+=3;
    } else {
      pub_s_(SI_KW,   u16_(b)/1000.f);   b+=2;
      pub_s_(SI_KVAR, s16m_(b,1000.f));  b+=2;
      pub_s_(SI_FREQ, u16_(b)/100.f);    b+=2;
      pub_s_(SI_COS,  s16m_(b,1000.f));  b+=2;
      pub_s_(SI_V1, u16_(b)/100.f); b+=2;
      pub_s_(SI_I1, u24_(b)/1000.f); b+=3;
    }
    return true;
  }

  bool parse_phase_() {
    if (rlen_ < 40 || rbuf_[6] != 0x2B) return false;
    size_t b = 16;
    pub_s_(SI_CA, s16m_(b,1000.f)); b+=2;
    pub_s_(SI_CB, s16m_(b,1000.f)); b+=2;
    pub_s_(SI_CC, s16m_(b,1000.f)); b+=2;
    pub_s_(SI_PA, u16_(b)); b+=2;
    pub_s_(SI_PB, u16_(b)); b+=2;
    pub_s_(SI_PC, u16_(b)); b+=2;
    pub_s_(SI_QA, s16m_(b,1.f)); b+=2;
    pub_s_(SI_QB, s16m_(b,1.f)); b+=2;
    pub_s_(SI_QC, s16m_(b,1.f)); b+=2;
    pub_s_(SI_SA, u16_(b)); b+=2;
    pub_s_(SI_SB, u16_(b)); b+=2;
    pub_s_(SI_SC, u16_(b)); b+=2;
    if (b < rlen_) {
      float t = (rbuf_[b] >= 128) ? (float)(rbuf_[b] - 128) * -1.f : (float)rbuf_[b];
      pub_s_(SI_TEMP, t);
    }
    return true;
  }

  bool parse_ident_() {
    if (rlen_ < 20 || rbuf_[6] != 0x01) return false;
    char fw[12];
    snprintf(fw, 12, "%02d.%02d", rbuf_[13], rbuf_[12]);
    pub_txt_(TI_FW, fw);

    uint8_t sw_ver = rbuf_[13];
    bool par_r2 = (rbuf_[9] >> 3) & 1;
    bool par_r1 = (rbuf_[9] >> 2) & 1;
    bool relay_on = (sw_ver <= 4 && !par_r1) ? par_r2 : !par_r2;
    pub_txt_(TI_RELAY, relay_on ? "ON" : "OFF");
    pub_bin_(BI_RELAY, relay_on);

    bool p1 = (rbuf_[8] >> 1) & 1;
    bool p2 = (rbuf_[8] >> 2) & 1;
    bool p3 = (rbuf_[8] >> 3) & 1;
    bool m1 = (rbuf_[8] >> 4) & 1;
    bool m2 = (rbuf_[8] >> 5) & 1;
    bool seals_ok = !(p1||p2||p3||m1||m2);
    std::string ss = seals_ok ? "OK" : "";
    if (p3) ss += "module;";
    if (p2) ss += "cover;";
    if (p1) ss += "terminals;";
    if (m1) ss += "dc_magnet;";
    if (m2) ss += "ac_magnet;";
    pub_txt_(TI_SEAL, ss);
    pub_bin_(BI_SEAL_OK, seals_ok);
    return true;
  }

  bool parse_battery_() {
    if (rlen_ < 14 || rbuf_[6] != 0x1E) return false;
    pub_s_(SI_BAT, (float)rbuf_[10]);
    return true;
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
