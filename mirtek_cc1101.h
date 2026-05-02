#pragma once
/**
 * mirtek_cc1101.h — Mirtek Star 104/304 for ESPHome
 * Use with: esphome: includes: [mirtek_cc1101.h]
 * https://github.com/Alecseyyy/ESPHome-Mirt-830
 *
 * NOTE: This file relies on esphome.h being included first (via custom_component).
 * Do NOT use as external_components — include it via esphome: includes: instead.
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <CRC8.h>
#include <cmath>
#include <string>

// ── RF-регистры CC1101 ────────────────────────────────────────────────
static const uint8_t MIRTEK_RF[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,
  0x0F,0x00,0x10,0x8B,0x54,0xD9,0x83,0x13,0xD2,0xAA,0x31,
  0x07,0x0C,0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,
  0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,
  0x81,0x35,0x09
};

static const uint8_t CMD_PING=0x01,CMD_ENERGY=0x05,CMD_ABON=0x07;
static const uint8_t CMD_FACTORY=0x0A,CMD_DATETIME=0x1C,CMD_BAT=0x1E;
static const uint8_t CMD_INSTANT=0x2B,CMD_GETINFO=0x30,CMD_RELAY=0x3A;
static const uint8_t SUB_E_ACT=0x04,SUB_E_REACT=0x05;
static const uint8_t SUB_I_UI=0x00,SUB_I_PWR=0x10;
static const uint8_t MT_3P=0xA8,MT_3P2=0xA9,MT_1P=0x98,MT_1P1=0x90;

static inline float mk_s16(uint8_t lo,uint8_t hi,float d){
  return (hi>=128)?float(lo|((hi-128)<<8))/-d:float(lo|(hi<<8))/d;
}
static inline float mk_u32le(const uint8_t*b){
  return float(((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0]);
}
static inline float mk_u16le(const uint8_t*b){
  return float((uint16_t)b[1]<<8|b[0]);
}

// Индексы сенсоров — должны совпадать с __init__.py
enum MirtekSI {
  MSI_SUM,MSI_T1,MSI_T2,MSI_T3,
  MSI_SUM_R,MSI_T1_R,MSI_T2_R,MSI_T3_R,
  MSI_KW,MSI_KVAR,MSI_FREQ,MSI_COS,
  MSI_V1,MSI_V2,MSI_V3,MSI_I1,MSI_I2,MSI_I3,
  MSI_PA,MSI_PB,MSI_PC,MSI_QA,MSI_QB,MSI_QC,
  MSI_SA,MSI_SB,MSI_SC,MSI_CA,MSI_CB,MSI_CC,
  MSI_TEMP,MSI_BAT,MSI_COUNT
};
enum MirtekTI {MTI_TAR,MTI_RELAY,MTI_SEAL,MTI_TYPE,MTI_FW,MTI_DATE,MTI_TIME,MTI_WORK,MTI_SYNC,MTI_SER,MTI_ABON,MTI_ST};
enum MirtekBI {MBI_3P,MBI_REL,MBI_SEAL,MBI_CC};

class MirtekCC1101 : public esphome::Component {
 public:
  esphome::sensor::Sensor        *mss[MSI_COUNT]{};
  esphome::text_sensor::TextSensor *mts[12]{};
  esphome::binary_sensor::BinarySensor *mbs[4]{};

  void set_sensor(int i,esphome::sensor::Sensor*s)               {if(i<MSI_COUNT)mss[i]=s;}
  void set_text_sensor(int i,esphome::text_sensor::TextSensor*s) {if(i<12)mts[i]=s;}
  void set_binary_sensor(int i,esphome::binary_sensor::BinarySensor*s){if(i<4)mbs[i]=s;}
  void set_meter_address(int a){addr_=a;}
  void set_cs_pin(int p){cs_=p;}
  void set_gdo0_pin(int p){gdo0_=p;}

  int addr_{0},cs_{5},gdo0_{22};
  bool cc1101_ok_{false},three_phase_{true};
  int poll_cnt_{0};
  uint8_t meter_type_{MT_3P};

  float msum{NAN},mt1{NAN},mt2{NAN},mt3{NAN};
  float msum_r{NAN},mt1_r{NAN},mt2_r{NAN},mt3_r{NAN};
  float mkw{NAN},mkvar{NAN},mfreq{NAN},mcos{NAN};
  float mv1{NAN},mv2{NAN},mv3{NAN};
  float mi1{NAN},mi2{NAN},mi3{NAN};
  float mpa{NAN},mpb{NAN},mpc{NAN};
  float mqa{NAN},mqb{NAN},mqc{NAN};
  float msa{NAN},msb{NAN},msc{NAN};
  float mca{NAN},mcb{NAN},mcc{NAN};
  float mtemp{NAN},mbat{NAN};
  std::string mtariff{"—"},mrelay{"—"},mseal{"—"},mtype{"—"};
  std::string mfw{"—"},mdate{"—"},mtime{"—"};
  std::string mwork{"—"},msync{"—"},mserial{"—"},mabon{"—"},mstatus{"Init"};

  static const int BUF=64;
  uint8_t tx_[BUF]{},raw_[BUF]{},res_[BUF]{};
  int raw_len_{0},res_len_{0};
  uint8_t my_crc_{0};
  CRC8 crc_;

  float get_setup_priority() const override {return esphome::setup_priority::HARDWARE;}

  void setup() override {
    ESP_LOGI("mirtek","Init CC1101 cs=%d gdo0=%d addr=%d",cs_,gdo0_,addr_);
    ELECHOUSE_cc1101.setGDO0(gdo0_);
    cc1101_ok_=ELECHOUSE_cc1101.getCC1101();
    if(mbs[MBI_CC])mbs[MBI_CC]->publish_state(cc1101_ok_);
    if(!cc1101_ok_){ESP_LOGE("mirtek","CC1101 SPI error!");mstatus="CC1101 ERROR";pub_t_();return;}
    ELECHOUSE_cc1101.SpiStrobe(0x30);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x00,(byte*)MIRTEK_RF,0x2F);
    ELECHOUSE_cc1101.SpiStrobe(0x33);delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);ELECHOUSE_cc1101.SpiStrobe(0x3B);
    ELECHOUSE_cc1101.SpiStrobe(0x34);
    ESP_LOGI("mirtek","CC1101 OK");mstatus="Ready";pub_t_();
  }

  void loop() override {}

  void poll_all() {
    if(!cc1101_ok_)return;
    poll_cnt_++;
    ESP_LOGI("mirtek","=== Poll #%d addr=%d ===",poll_cnt_,addr_);
    if(xact_(CMD_PING,-1))           parse_ping_();
    if(xact_(CMD_DATETIME,-1))       parse_datetime_();
    if(xact_(CMD_ENERGY,1,SUB_E_ACT))  parse_energy_(false);
    if(xact_(CMD_ENERGY,1,SUB_E_REACT))parse_energy_(true);
    if(xact_(CMD_INSTANT,1,SUB_I_UI))  parse_ui_();
    if(three_phase_&&xact_(CMD_INSTANT,1,SUB_I_PWR))parse_pwr_();
    if(xact_(CMD_GETINFO,-1))        parse_getinfo_();
    if(xact_(CMD_BAT,-1))            parse_bat_();
    if(xact_(CMD_FACTORY,1,1))       parse_str_(CMD_FACTORY,1);
    if(xact_(CMD_ABON,1,1))          parse_str_(CMD_ABON,1);
    if(xact_(CMD_ABON,1,6))          parse_str_(CMD_ABON,6);
    pub_all_();
    char st[24];snprintf(st,24,"OK #%d",poll_cnt_);
    mstatus=st;pub_t_();
    ESP_LOGI("mirtek","=== Done ===");
  }

  void relay_on(){ctrl_relay_(0x00);}
  void relay_off(){ctrl_relay_(0x01);}

 private:
  void ctrl_relay_(uint8_t cmd){
    build_(CMD_RELAY,2,0x00,cmd);send_();recv_(600);
    mrelay=(cmd==0)?"On":"Off";
    if(mts[MTI_RELAY])mts[MTI_RELAY]->publish_state(mrelay);
    if(mbs[MBI_REL])mbs[MBI_REL]->publish_state(cmd==0);
  }

  void build_(uint8_t cmd,int sub_cnt=-1,uint8_t s1=0,uint8_t s2=0){
    int dlen=(sub_cnt<0)?0:sub_cnt;
    uint8_t tmp[44]{};int n=0;
    tmp[n++]=0x73;tmp[n++]=0x55;
    tmp[n++]=uint8_t(0x20|(dlen&0x1F));tmp[n++]=0x00;
    tmp[n++]=uint8_t(addr_&0xFF);tmp[n++]=uint8_t((addr_>>8)&0xFF);
    tmp[n++]=0xFE;tmp[n++]=0xFF;tmp[n++]=cmd;
    tmp[n++]=0;tmp[n++]=0;tmp[n++]=0;tmp[n++]=0;
    if(sub_cnt>=1)tmp[n++]=s1;
    if(sub_cnt>=2)tmp[n++]=s2;
    crc_.restart();crc_.setPolynome(0xA9);
    for(int i=2;i<n;i++)crc_.add(tmp[i]);
    tmp[n++]=crc_.calc();tmp[n++]=0x55;
    memset(tx_,0,BUF);int out=0;
    tx_[out++]=tmp[0];tx_[out++]=tmp[1];
    for(int i=2;i<n-1;i++){
      if(tmp[i]==0x55){tx_[out++]=0x73;tx_[out++]=0x11;}
      else if(tmp[i]==0x73){tx_[out++]=0x73;tx_[out++]=0x22;}
      else{tx_[out++]=tmp[i];}
    }
    tx_[out++]=0x55;
    memmove(tx_+1,tx_,out);tx_[0]=(uint8_t)out;
  }

  void send_(){
    ELECHOUSE_cc1101.SpiStrobe(0x33);delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);ELECHOUSE_cc1101.SpiStrobe(0x36);
    ELECHOUSE_cc1101.SpiWriteReg(0x3E,0xC4);
    ELECHOUSE_cc1101.SendData(tx_,tx_[0]+1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);ELECHOUSE_cc1101.SpiStrobe(0x34);
  }

  bool recv_(uint32_t ms){
    memset(raw_,0,BUF);memset(res_,0,BUF);
    raw_len_=0;res_len_=0;
    uint8_t piece[BUF]{};uint32_t t0=millis();
    while(millis()-t0<ms){
      if(ELECHOUSE_cc1101.CheckReceiveFlag()){
        int len=ELECHOUSE_cc1101.ReceiveData(piece);
        for(int i=1;i<len&&raw_len_<BUF-1;i++)raw_[raw_len_++]=piece[i];
        ELECHOUSE_cc1101.SpiStrobe(0x36);ELECHOUSE_cc1101.SpiStrobe(0x3A);
        ELECHOUSE_cc1101.SpiStrobe(0x3B);ELECHOUSE_cc1101.SpiStrobe(0x34);
      }
      yield();
    }
    if(!raw_len_)return false;
    int j=0;
    for(int i=0;i<raw_len_;i++){
      res_[i-j]=raw_[i];res_len_++;
      if(raw_[i]==0x73&&i+1<raw_len_){
        if(raw_[i+1]==0x11){res_[i-j]=0x55;i++;j++;}
        else if(raw_[i+1]==0x22){i++;j++;}
      }
    }
    crc_.reset();crc_.setPolynome(0xA9);
    int ce=res_len_-2;
    if(res_len_>3&&res_[res_len_-3]==0x76)ce=res_len_-3;
    for(int i=2;i<ce;i++)crc_.add(res_[i]);
    my_crc_=crc_.calc();
    return true;
  }

  bool valid_(uint8_t cmd){
    if(res_len_<10)return false;
    if(res_[0]!=0x73||res_[1]!=0x55)return false;
    if(res_[6]!=uint8_t(addr_&0xFF))return false;
    if(res_[8]!=cmd){ESP_LOGW("mirtek","cmd %02X!=%02X",res_[8],cmd);return false;}
    if(res_[res_len_-2]!=my_crc_){ESP_LOGW("mirtek","crc %02X!=%02X",my_crc_,res_[res_len_-2]);return false;}
    if(res_[res_len_-1]!=0x55)return false;
    return true;
  }

  bool xact_(uint8_t cmd,int sub=-1,uint8_t s=0){
    build_(cmd,sub,s,0);send_();
    if(!recv_(800)){ESP_LOGW("mirtek","timeout cmd=%02X",cmd);return false;}
    if(!valid_(cmd)){ESP_LOGW("mirtek","invalid cmd=%02X",cmd);return false;}
    return true;
  }

  void parse_ping_(){
    meter_type_=res_[9];
    three_phase_=(meter_type_==MT_3P||meter_type_==MT_3P2);
    if(meter_type_==MT_3P||meter_type_==MT_3P2)mtype="3ph transformer";
    else if(meter_type_==MT_1P)mtype="1ph 2-element";
    else if(meter_type_==MT_1P1)mtype="1ph 1-element";
    else{char b[8];snprintf(b,8,"0x%02X",meter_type_);mtype=b;}
    char fw[8];snprintf(fw,8,"%02u.%02u",res_[14],res_[13]);mfw=fw;
    bool p3=(res_[10]>>3)&1,p2=(res_[10]>>2)&1,p1=(res_[10]>>1)&1;
    bool m1=(res_[10]>>4)&1,m2=(res_[10]>>5)&1;
    mseal=(!p1&&!p2&&!p3&&!m1&&!m2)?"OK":"ALARM";
    bool r2=(res_[11]>>3)&1;
    mrelay=((res_[14]<5)?r2:!r2)?"On":"Off";
    ESP_LOGI("mirtek","Type=%s FW=%s Relay=%s",mtype.c_str(),fw,mrelay.c_str());
  }

  void parse_datetime_(){
    static const char*dow[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    char dt[24],tm[10];
    snprintf(dt,24,"%02u-%02u-20%02u %s",res_[17],res_[18],res_[19],dow[res_[16]%7]);
    snprintf(tm,10,"%02u:%02u:%02u",res_[15],res_[14],res_[13]);
    mdate=dt;mtime=tm;
    static const char*ts[]={"Day","Night","Halfpeak","Special"};
    mtariff=ts[(res_[10]>>2)&0x03];
  }

  void parse_energy_(bool react){
    float s=mk_u32le(res_+19)/100.0f;
    float a=mk_u32le(res_+27)/100.0f;
    float b=mk_u32le(res_+31)/100.0f;
    float c=mk_u32le(res_+35)/100.0f;
    uint8_t nt=((res_[14]>>6)&0x03)+1;
    if(!react){
      msum=s;mt1=a;mt2=(nt>=2)?b:NAN;mt3=(nt>=3)?c:NAN;
      static const char*ts[]={"Day","Night","Halfpeak","Special"};
      mtariff=ts[(res_[14]>>2)&0x03];
    }else{
      msum_r=s;mt1_r=a;mt2_r=(nt>=2)?b:NAN;mt3_r=(nt>=3)?c:NAN;
    }
  }

  void parse_ui_(){
    const uint8_t*r=res_;
    if(three_phase_){
      mkw=float(r[18]|(r[19]<<8)|(r[20]<<16))/1000.0f;
      mkvar=(r[23]>=128)?float(r[21]|(r[22]<<8)|((r[23]-128)<<16))/-1000.0f
                        :float(r[21]|(r[22]<<8)|(r[23]<<16))/1000.0f;
      mfreq=mk_u16le(r+24)/100.0f;mcos=mk_s16(r[26],r[27],1000.0f);
      mv1=mk_u16le(r+28)/100.0f;mv2=mk_u16le(r+30)/100.0f;mv3=mk_u16le(r+32)/100.0f;
      mi1=float(r[34]|(r[35]<<8)|(r[36]<<16))/1000.0f;
      mi2=float(r[37]|(r[38]<<8)|(r[39]<<16))/1000.0f;
      mi3=float(r[40]|(r[41]<<8)|(r[42]<<16))/1000.0f;
    }else{
      mkw=mk_u16le(r+18)/1000.0f;mkvar=mk_s16(r[20],r[21],1000.0f);
      mfreq=mk_u16le(r+22)/100.0f;mcos=mk_s16(r[24],r[25],1000.0f);
      mv1=mk_u16le(r+26)/100.0f;mv2=NAN;mv3=NAN;
      mi1=float(r[32]|(r[33]<<8)|(r[34]<<16))/1000.0f;mi2=NAN;mi3=NAN;
    }
    ESP_LOGI("mirtek","U=%.1f/%.1f/%.1f P=%.3fkW f=%.2f",mv1,mv2,mv3,mkw,mfreq);
  }

  void parse_pwr_(){
    const uint8_t*r=res_;
    mca=mk_s16(r[18],r[19],1000.0f);mcb=mk_s16(r[20],r[21],1000.0f);mcc=mk_s16(r[22],r[23],1000.0f);
    mpa=mk_u16le(r+24);mpb=mk_u16le(r+26);mpc=mk_u16le(r+28);
    mqa=mk_s16(r[30],r[31],1.0f);mqb=mk_s16(r[32],r[33],1.0f);mqc=mk_s16(r[34],r[35],1.0f);
    msa=mk_u16le(r+36);msb=mk_u16le(r+38);msc=mk_u16le(r+40);
    mtemp=(r[42]>=128)?-(float)(r[42]-128):(float)r[42];
  }

  void parse_getinfo_(){
    uint32_t w=mk4le_(res_+18),sy=mk4le_(res_+32);
    mwork=fmt_sec_(w);msync=fmt_sec_(sy);
  }

  void parse_bat_(){
    if(res_len_<15)return;
    uint8_t tot=res_[13],rem=res_[14];
    mbat=(tot>0)?(100.0f*rem/tot):NAN;
  }

  void parse_str_(uint8_t cmd,uint8_t field){
    if(res_len_<14)return;
    char buf[31]{};memcpy(buf,res_+14,30);
    for(int i=29;i>=0&&(buf[i]==' '||buf[i]==0);i--)buf[i]=0;
    if(cmd==CMD_FACTORY&&field==1)mserial=buf;
    if(cmd==CMD_ABON){
      if(field==1)mabon=buf;
      else if(field==6&&strlen(buf)>0){mabon+=" / ";mabon+=buf;}
    }
  }

  uint32_t mk4le_(const uint8_t*b){
    return ((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0];
  }

  std::string fmt_sec_(uint32_t s){
    uint32_t d=s/86400,h=(s%86400)/3600,m=(s%3600)/60;
    char b[24];
    if(d)snprintf(b,24,"%ud %02u:%02u",(unsigned)d,(unsigned)h,(unsigned)m);
    else snprintf(b,24,"%02u:%02u",(unsigned)h,(unsigned)m);
    return b;
  }

  void pf_(int i,float v){if(mss[i]&&!std::isnan(v))mss[i]->publish_state(v);}

  void pub_all_(){
    pf_(MSI_SUM,msum);pf_(MSI_T1,mt1);pf_(MSI_T2,mt2);pf_(MSI_T3,mt3);
    pf_(MSI_SUM_R,msum_r);pf_(MSI_T1_R,mt1_r);pf_(MSI_T2_R,mt2_r);pf_(MSI_T3_R,mt3_r);
    pf_(MSI_KW,mkw);pf_(MSI_KVAR,mkvar);pf_(MSI_FREQ,mfreq);pf_(MSI_COS,mcos);
    pf_(MSI_V1,mv1);pf_(MSI_V2,mv2);pf_(MSI_V3,mv3);
    pf_(MSI_I1,mi1);pf_(MSI_I2,mi2);pf_(MSI_I3,mi3);
    pf_(MSI_PA,mpa);pf_(MSI_PB,mpb);pf_(MSI_PC,mpc);
    pf_(MSI_QA,mqa);pf_(MSI_QB,mqb);pf_(MSI_QC,mqc);
    pf_(MSI_SA,msa);pf_(MSI_SB,msb);pf_(MSI_SC,msc);
    pf_(MSI_CA,mca);pf_(MSI_CB,mcb);pf_(MSI_CC,mcc);
    pf_(MSI_TEMP,mtemp);pf_(MSI_BAT,mbat);
    pub_t_();
  }

  void pub_t_(){
    if(mts[MTI_TAR])mts[MTI_TAR]->publish_state(mtariff);
    if(mts[MTI_RELAY])mts[MTI_RELAY]->publish_state(mrelay);
    if(mts[MTI_SEAL])mts[MTI_SEAL]->publish_state(mseal);
    if(mts[MTI_TYPE])mts[MTI_TYPE]->publish_state(mtype);
    if(mts[MTI_FW])mts[MTI_FW]->publish_state(mfw);
    if(mts[MTI_DATE])mts[MTI_DATE]->publish_state(mdate);
    if(mts[MTI_TIME])mts[MTI_TIME]->publish_state(mtime);
    if(mts[MTI_WORK])mts[MTI_WORK]->publish_state(mwork);
    if(mts[MTI_SYNC])mts[MTI_SYNC]->publish_state(msync);
    if(mts[MTI_SER])mts[MTI_SER]->publish_state(mserial);
    if(mts[MTI_ABON])mts[MTI_ABON]->publish_state(mabon);
    if(mts[MTI_ST])mts[MTI_ST]->publish_state(mstatus);
    if(mbs[MBI_3P])mbs[MBI_3P]->publish_state(three_phase_);
    if(mbs[MBI_REL])mbs[MBI_REL]->publish_state(mrelay=="On");
    if(mbs[MBI_SEAL])mbs[MBI_SEAL]->publish_state(mseal=="OK");
  }
};
