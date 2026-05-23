#pragma once
// =============================================================================
// Mirtek CC1101 ESPHome Component — ESPHome 2026.5.x
// Uses ESPHome spi::SPIDevice (no Arduino SPI.h needed)
// =============================================================================
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/spi/spi.h"
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

// CC1101 command strobes
static const uint8_t CC_SRES=0x30, CC_SCAL=0x33, CC_SRX=0x34;
static const uint8_t CC_STX=0x35,  CC_SIDLE=0x36, CC_SFRX=0x3A, CC_SFTX=0x3B;
static const uint8_t CC_BURST=0x40, CC_READ=0x80;
static const uint8_t CC_TXFIFO=0x3F, CC_RXFIFO=0x3F, CC_RXBYTES=0x3B, CC_VERSION=0x31;

// Mirtek 433 MHz RF settings
static const uint8_t RF_CFG[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,
  0x41,0x00,0x16,0x0F,0x00,0x10,0x8B,0x54,
  0xD9,0x83,0x13,0xD2,0xAA,0x31,0x07,0x0C,
  0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,
  0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,
  0x00,0x59,0x59,0x3F,0x81,0x35,0x09
};

// Sensor index enum — order must match SENSORS list in __init__.py
enum SI {
  SI_SUM=0,SI_T1,SI_T2,SI_T3,
  SI_SR,SI_T1R,SI_T2R,SI_T3R,
  SI_KW,SI_KVAR,SI_FREQ,SI_COS,
  SI_V1,SI_V2,SI_V3,
  SI_I1,SI_I2,SI_I3,
  SI_PA,SI_PB,SI_PC,
  SI_QA,SI_QB,SI_QC,
  SI_SA,SI_SB,SI_SC,
  SI_CA,SI_CB,SI_CC,
  SI_TEMP,SI_BAT,
  SI_COUNT
};
enum TI { TI_TARIFF=0,TI_RELAY,TI_SEAL,TI_TYPE,TI_FW,TI_DATE,TI_TIME,TI_WORK,TI_SYNC,TI_SERIAL,TI_ABON,TI_STATUS,TI_COUNT };
enum BI { BI_3PH=0,BI_RELAY,BI_SEAL,BI_CC,BI_COUNT };

class MirtekCC1101 : public PollingComponent,
                     public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                           spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }
  void set_meter_address(int a) { addr_ = (uint16_t)a; }
  void set_sensor(int i, sensor::Sensor *s)               { if (i < SI_COUNT) ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s)  { if (i < TI_COUNT) ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s){ if (i < BI_COUNT) bs_[i] = s; }

  void setup() override {
    this->spi_setup();
    if (gdo0_) gdo0_->setup();
    bool ok = cc_init_();
    pub_bin_(BI_CC, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");
    ESP_LOGI(TAG, "CC1101 %s, meter addr=%u", ok ? "ready" : "ERROR", addr_);
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101:");
    ESP_LOGCONFIG(TAG, "  addr=%u interval=%ums", addr_, get_update_interval());
    LOG_PIN("  GDO0: ", gdo0_);
  }

  void update() override { poll_all(); }

  void poll_all() {
    poll_count_++;
    bool ok1 = do_cmd_(0x1C,-1,-1,3) && parse_datetime_();
    bool ok2 = do_cmd_(0x05,0x04,-1,4) && parse_energy_();
    bool ok3 = do_cmd_(0x2B,0x00,-1,4) && parse_instant_();
    bool ok4 = !three_phase_ || (do_cmd_(0x2B,0x10,-1,4) && parse_phase_());
    if (poll_count_ % 12 == 1) {
      do_cmd_(0x01,-1,-1,5) && parse_ident_();
      do_cmd_(0x1E,-1,-1,3) && parse_battery_();
    }
    pub_txt_(TI_STATUS, (ok1&&ok2&&ok3&&ok4) ? "OK" : "PARTIAL");
  }

  void relay_on()  { do_cmd_(0x3A,0x08,-1,2); }
  void relay_off() { do_cmd_(0x3A,0x09,-1,2); }

 protected:
  GPIOPin *gdo0_{nullptr};
  uint16_t addr_{1};
  bool three_phase_{true};
  uint32_t poll_count_{0};
  sensor::Sensor          *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};
  uint8_t sbuf_[20]{}, rbuf_[64]{};
  size_t rlen_{0};

  // SPI helpers using ESPHome SPIDevice API
  void cc_strobe_(uint8_t cmd) {
    this->enable(); this->transfer_byte(cmd); this->disable();
  }
  void cc_wreg_(uint8_t addr, uint8_t val) {
    this->enable(); this->transfer_byte(addr & 0x3F); this->transfer_byte(val); this->disable();
  }
  void cc_wburst_(uint8_t addr, const uint8_t *d, size_t n) {
    this->enable(); this->transfer_byte((addr & 0x3F) | CC_BURST);
    for (size_t i = 0; i < n; i++) this->transfer_byte(d[i]);
    this->disable();
  }
  uint8_t cc_rreg_(uint8_t addr) {
    this->enable(); this->transfer_byte(CC_READ | (addr & 0x3F)); uint8_t v = this->transfer_byte(0); this->disable();
    return v;
  }
  uint8_t cc_rstat_(uint8_t addr) {
    this->enable(); this->transfer_byte(CC_READ | CC_BURST | (addr & 0x3F)); uint8_t v = this->transfer_byte(0); this->disable();
    return v;
  }

  bool cc_init_() {
    this->disable(); delayMicroseconds(5);
    this->enable();  delayMicroseconds(10);
    this->disable(); delayMicroseconds(41);
    cc_strobe_(CC_SRES); delay(10);
    cc_wburst_(0x00, RF_CFG, sizeof(RF_CFG));
    cc_strobe_(CC_SCAL); delay(2);
    cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
    uint8_t ver = cc_rstat_(CC_VERSION);
    ESP_LOGD(TAG, "CC1101 ver=0x%02X", ver);
    return (ver == 0x14 || ver == 0x04);
  }

  void stuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    out.push_back(in[0]); out.push_back(in[1]); out.push_back(in[2]);
    for (size_t i = 3; i < n-1; i++) {
      if      (in[i]==0x55){out.push_back(0x73);out.push_back(0x11);}
      else if (in[i]==0x73){out.push_back(0x73);out.push_back(0x22);}
      else                   out.push_back(in[i]);
    }
    out.push_back(in[n-1]);
  }
  void destuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    for (size_t i = 0; i < n; i++) {
      if (in[i]==0x73 && i+1<n) {
        if      (in[i+1]==0x11){out.push_back(0x55);i++;}
        else if (in[i+1]==0x22){out.push_back(0x73);i++;}
        else                     out.push_back(in[i]);
      } else out.push_back(in[i]);
    }
  }

  size_t build_pkt_(uint8_t cmd, int s1, int s2) {
    uint8_t *b = sbuf_;
    uint8_t dl = (s1<0)?0:(s2<0)?1:2;
    b[0]=0x0F+dl; b[1]=0x73; b[2]=0x55; b[3]=0x20+dl; b[4]=0x00;
    b[5]=addr_&0xFF; b[6]=(addr_>>8)&0xFF; b[7]=0xFE; b[8]=0xFF; b[9]=cmd;
    b[10]=b[11]=b[12]=b[13]=0;
    size_t p=14;
    if(s1>=0) b[p++]=(uint8_t)s1;
    if(s2>=0) b[p++]=(uint8_t)s2;
    b[p]=crc8_mirtek(b+3,p-3); b[p+1]=0x55;
    return p+2;
  }

  bool do_cmd_(uint8_t cmd, int s1, int s2, int exp) {
    size_t raw=build_pkt_(cmd,s1,s2);
    std::vector<uint8_t> tx;
    stuff_(sbuf_,raw,tx);
    tx[0]=(uint8_t)(tx.size()-1);

    cc_strobe_(CC_SIDLE); cc_strobe_(CC_SFTX); cc_strobe_(CC_SFRX);
    this->enable(); this->transfer_byte(CC_TXFIFO|CC_BURST);
    for (uint8_t b:tx) this->transfer_byte(b);
    this->disable();
    cc_strobe_(CC_STX);

    if (gdo0_) {
      uint32_t t0=millis();
      while (!gdo0_->digital_read() && millis()-t0<200) delayMicroseconds(100);
      while ( gdo0_->digital_read() && millis()-t0<500) delayMicroseconds(100);
    } else { delay(80); }

    cc_strobe_(CC_SFRX); cc_strobe_(CC_SRX);

    std::vector<uint8_t> raw_all;
    int got=0; uint32_t t0=millis();
    while (millis()-t0<1200 && got<exp) {
      uint8_t rxb=cc_rstat_(CC_RXBYTES);
      if (rxb>0 && rxb<64) {
        got++;
        this->enable(); this->transfer_byte(CC_RXFIFO|CC_READ|CC_BURST);
        uint8_t lb=this->transfer_byte(0);
        if (lb>0&&lb<60) for(uint8_t i=1;i<lb;i++) raw_all.push_back(this->transfer_byte(0));
        this->disable();
        cc_strobe_(CC_SIDLE); cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
      }
      delayMicroseconds(500);
    }
    if (raw_all.empty()) { ESP_LOGW(TAG,"cmd=0x%02X no rx",cmd); return false; }

    std::vector<uint8_t> ds;
    destuff_(raw_all.data(),raw_all.size(),ds);
    if (ds.size()>sizeof(rbuf_)) return false;
    memcpy(rbuf_,ds.data(),ds.size()); rlen_=ds.size();

    if (rlen_<5||rbuf_[0]!=0x73||rbuf_[1]!=0x55) { ESP_LOGW(TAG,"bad hdr"); return false; }
    if (rbuf_[4]!=(addr_&0xFF)||rbuf_[5]!=((addr_>>8)&0xFF)) { ESP_LOGW(TAG,"addr mismatch"); return false; }
    return true;
  }

  float u16_(size_t i){return(float)((uint16_t)rbuf_[i]|((uint16_t)rbuf_[i+1]<<8));}
  float u32_(size_t i){return(float)(((uint32_t)rbuf_[i])|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)rbuf_[i+2]<<16)|((uint32_t)rbuf_[i+3]<<24));}
  float u24_(size_t i){return(float)(((uint32_t)rbuf_[i])|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)rbuf_[i+2]<<16));}
  float s16m_(size_t i,float div){bool n=rbuf_[i+1]>=128;return((float)((uint16_t)rbuf_[i]|((uint16_t)(rbuf_[i+1]&0x7F)<<8))/div)*(n?-1.f:1.f);}
  float s24m_(size_t i,float div){bool n=rbuf_[i+2]>=128;return((float)(((uint32_t)rbuf_[i])|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)(rbuf_[i+2]&0x7F)<<16))/div)*(n?-1.f:1.f);}

  void pub_s_(int i,float v){if(ss_[i])ss_[i]->publish_state(v);}
  void pub_txt_(int i,const std::string &v){if(ts_[i])ts_[i]->publish_state(v);}
  void pub_bin_(int i,bool v){if(bs_[i])bs_[i]->publish_state(v);}

  bool parse_datetime_() {
    if(rlen_<20||rbuf_[6]!=0x1C) return false;
    uint8_t tp=rbuf_[7];
    three_phase_=!((tp==0x98)||(tp==0x99));
    char tb[32];
    switch(tp){
      case 0xA8:case 0xA9:snprintf(tb,32,"3ph transf act-react");break;
      case 0x88:case 0x89:snprintf(tb,32,"3ph transf act bidi");break;
      case 0x80:case 0x81:snprintf(tb,32,"3ph act bidi");break;
      case 0x68:case 0x69:snprintf(tb,32,"3ph transf act");break;
      case 0x98:case 0x99:snprintf(tb,32,"1ph act-react");break;
      default:snprintf(tb,32,"type 0x%02X",tp);
    }
    pub_txt_(TI_TYPE,tb); pub_bin_(BI_3PH,three_phase_);
    if(rlen_>=18){
      char dt[12],tm[10];
      snprintf(tm,10,"%02d:%02d:%02d",rbuf_[13],rbuf_[12],rbuf_[11]);
      snprintf(dt,12,"%02d.%02d.%02d",rbuf_[14],rbuf_[15],rbuf_[16]);
      pub_txt_(TI_TIME,tm); pub_txt_(TI_DATE,dt);
    }
    return true;
  }

  bool parse_energy_() {
    if(rlen_<35||rbuf_[6]!=0x05) return false;
    uint8_t cur_t=(rbuf_[12]>>2)&0x03;
    const char *tn[]={"T1 Day","T2 Night","T3 Peak","Special"};
    pub_txt_(TI_TARIFF,tn[cur_t]);
    if(rlen_>=21) pub_s_(SI_SUM,u32_(17)/100.f);
    if(rlen_>=29) pub_s_(SI_T1, u32_(25)/100.f);
    if(rlen_>=33) pub_s_(SI_T2, u32_(29)/100.f);
    if(rlen_>=37) pub_s_(SI_T3, u32_(33)/100.f);
    if(rlen_>=53){pub_s_(SI_SR,u32_(37)/100.f);pub_s_(SI_T1R,u32_(41)/100.f);pub_s_(SI_T2R,u32_(45)/100.f);pub_s_(SI_T3R,u32_(49)/100.f);}
    return true;
  }

  bool parse_instant_() {
    if(rlen_<36||rbuf_[6]!=0x2B) return false;
    size_t b=16;
    if(three_phase_){
      pub_s_(SI_KW,  u24_(b)/1000.f); b+=3;
      pub_s_(SI_KVAR,s24m_(b,1000.f));b+=3;
      pub_s_(SI_FREQ,u16_(b)/100.f);  b+=2;
      pub_s_(SI_COS, s16m_(b,1000.f));b+=2;
      pub_s_(SI_V1,u16_(b)/100.f);b+=2; pub_s_(SI_V2,u16_(b)/100.f);b+=2; pub_s_(SI_V3,u16_(b)/100.f);b+=2;
      pub_s_(SI_I1,u24_(b)/1000.f);b+=3; pub_s_(SI_I2,u24_(b)/1000.f);b+=3; pub_s_(SI_I3,u24_(b)/1000.f);b+=3;
    } else {
      pub_s_(SI_KW,  u16_(b)/1000.f);   b+=2;
      pub_s_(SI_KVAR,s16m_(b,1000.f));  b+=2;
      pub_s_(SI_FREQ,u16_(b)/100.f);    b+=2;
      pub_s_(SI_COS, s16m_(b,1000.f));  b+=2;
      pub_s_(SI_V1,u16_(b)/100.f);b+=2; pub_s_(SI_I1,u24_(b)/1000.f);b+=3;
    }
    return true;
  }

  bool parse_phase_() {
    if(rlen_<40||rbuf_[6]!=0x2B) return false;
    size_t b=16;
    pub_s_(SI_CA,s16m_(b,1000.f));b+=2; pub_s_(SI_CB,s16m_(b,1000.f));b+=2; pub_s_(SI_CC,s16m_(b,1000.f));b+=2;
    pub_s_(SI_PA,u16_(b));b+=2; pub_s_(SI_PB,u16_(b));b+=2; pub_s_(SI_PC,u16_(b));b+=2;
    pub_s_(SI_QA,s16m_(b,1.f));b+=2; pub_s_(SI_QB,s16m_(b,1.f));b+=2; pub_s_(SI_QC,s16m_(b,1.f));b+=2;
    pub_s_(SI_SA,u16_(b));b+=2; pub_s_(SI_SB,u16_(b));b+=2; pub_s_(SI_SC,u16_(b));b+=2;
    if(b<rlen_) pub_s_(SI_TEMP,(rbuf_[b]>=128)?(float)(rbuf_[b]-128)*-1.f:(float)rbuf_[b]);
    return true;
  }

  bool parse_ident_() {
    if(rlen_<20||rbuf_[6]!=0x01) return false;
    char fw[12]; snprintf(fw,12,"%02d.%02d",rbuf_[13],rbuf_[12]);
    pub_txt_(TI_FW,fw);
    bool r2=(rbuf_[9]>>3)&1, r1=(rbuf_[9]>>2)&1;
    bool relay_on=(rbuf_[13]<=4&&!r1)?r2:!r2;
    pub_txt_(TI_RELAY,relay_on?"ON":"OFF"); pub_bin_(BI_RELAY,relay_on);
    bool p1=(rbuf_[8]>>1)&1,p2=(rbuf_[8]>>2)&1,p3=(rbuf_[8]>>3)&1,m1=(rbuf_[8]>>4)&1,m2=(rbuf_[8]>>5)&1;
    bool ok=!(p1||p2||p3||m1||m2);
    std::string ss=ok?"OK":"";
    if(p3)ss+="module;"; if(p2)ss+="cover;"; if(p1)ss+="terminals;"; if(m1)ss+="dc_mag;"; if(m2)ss+="ac_mag;";
    pub_txt_(TI_SEAL,ss); pub_bin_(BI_SEAL,ok);
    return true;
  }

  bool parse_battery_() {
    if(rlen_<14||rbuf_[6]!=0x1E) return false;
    pub_s_(SI_BAT,(float)rbuf_[10]); return true;
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
