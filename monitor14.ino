#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <cmath>

#define RX2_PIN 16
#define TX2_PIN 17
#define RS485_CTRL 4
#define SLAVE_ID 0x04
#define LED_PIN 2
#define SKETCH_VERSION "3.4.21"

constexpr uint32_t MODBUS_UPDATE_INTERVAL_MS = 3000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t RESTART_DELAY_MS = 2000;

SemaphoreHandle_t dataMutex = nullptr;
TaskHandle_t modbusTaskHandle = nullptr;
volatile bool webUpdateActive = false;
volatile bool pendingCounterReset = false;

static uint16_t invBuf[80], pvBuf[20], ctrlBuf[50];

struct InverterData {
  uint16_t inv[80]; uint16_t pv[20]; uint16_t ctrl[50]; uint16_t chgCtrl[20];
  float vBatt=0, vBattFiltered=NAN, vOut=0, vGrid=0, vBus=0, vPV=0;
  int pLoad=0, gridPower=0, solarPowerRaw=0, effectiveSolarW=0, loadPercent=0;
  int tempAC=0, tempDC=0; float pvEnergy=0; int workState=0;
  float pNet=0, selfConsumption=0, selfConsumptionFiltered=0;
  float battPowerSigned=0, battPowerMeasured=0, battPowerFiltered=0, battAmpDisplay=0;
  int battWattDisplay=0, gridPowerShow=0, systemBalanceShow=0;
  int offgridEnable=0, energyMode=0, chargerPriority=0;
  const char* stateStr="OFF"; const char* workModeShort="OFF";
  const char* batModeUi="IDLE"; const char* batClassUi="blue"; bool batRegsValid=false;
  String machineType, machinePower;
  uint16_t err1=0,err2=0,err3=0,warn1=0,warn2=0,chgErr=0,chgWarn=0;
};

struct PowerSnapshot {
  bool valid=false; int workState=0; float vGrid=0;
  int gridPowerRaw=0, loadW=0, solarW=0;
};

AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;
ModbusMaster node;
InverterData inv;
PowerSnapshot snap;
String wifiSsid="", wifiPassword="";
bool setupMode=false;
unsigned long lastUpdateMs=0, lastWifiReconnectMs=0;
bool needRestart=false;
unsigned long restartRequestedMs=0;

void preTrans(){digitalWrite(RS485_CTRL,HIGH);delayMicroseconds(20);}
void postTrans(){delayMicroseconds(20);Serial1.flush();digitalWrite(RS485_CTRL,LOW);}

template <typename T>
bool readModbusBlock(uint16_t addr,uint8_t qty,T*buf){
  node.clearResponseBuffer();
  if(node.readHoldingRegisters(addr,qty)==node.ku8MBSuccess){
    for(uint8_t i=0;i<qty;i++)buf[i]=node.getResponseBuffer(i);
    return true;
  }
  return false;
}

uint16_t readSingleRegisterWithRetry(uint16_t addr,uint8_t retries=3){
  for(uint8_t i=0;i<retries;i++){
    node.clearResponseBuffer();
    if(node.readHoldingRegisters(addr,1)==node.ku8MBSuccess)return node.getResponseBuffer(0);
    delay(150);
  }
  return 0xFFFF;
}

String formatVersion(uint16_t raw){
  if(raw==0xFFFF||raw==0)return "--";
  int maj=raw/10000,min=(raw/100)%100,pat=raw%100;char buf[16];
  if(maj>0)snprintf(buf,sizeof(buf),"%d.%02d.%02d",maj,min,pat);
  else snprintf(buf,sizeof(buf),"%d.%02d",min,pat);
  return String(buf);
}

String fUnit(uint16_t raw,float div,int dec,const char*unit){
  if(raw==0xFFFF)return "--";
  float v=(float)(int16_t)raw/div;
  String s=(dec>0)?String(v,dec):String((int)roundf(v));
  return s+" "+unit;
}
String fRelay(uint16_t raw){if(raw==0xFFFF)return "--";return (raw==0)?"Disconnect":"Connect";}
String fAcc(uint16_t hi,uint16_t lo){
  if(hi==0xFFFF&&lo==0xFFFF)return "--";
  return String((float)hi*1000.0f+(float)lo/10.0f,1)+" kWh";
}
const char* workShort(int ws){
  switch(ws){case 0:return "ON";case 1:return "TEST";case 2:return "BATT";
  case 3:return "TIE";case 4:return "BYPASS";case 5:return "STOP";case 6:return "CHRG";default:return "--";}
}
String mpptStr(uint16_t r){if(r==0xFFFF)return "--";return (r==1)?"Run":(r==0?"Stop":String(r));}
String chgStateStr(uint16_t r){if(r==0xFFFF)return "--";return (r==1)?"Charging":(r==0?"Idle":(r==2?"Float":String(r)));}

String decodeErrors(uint16_t e1,uint16_t e2,uint16_t e3){
  if(e1==0xFFFF||e2==0xFFFF||e3==0xFFFF)return "--";
  if(e1==0&&e2==0&&e3==0)return "No errors";
  String errs="";
  const char*b1[]={"Fan locked","Transformer overtemp","Batt V high","Batt V low",
    "Output short","Inverter V high","Overload timeout","Bus V high",
    "Bus soft start fail","Main relay fail","Output V sensor","Grid V sensor",
    "Output I sensor","Grid I sensor","Load I sensor","Grid overcurrent"};
  const char*b2[]={"Radiator overtemp","Charger batt class err","Charger I sensor","Charger I uncontrolled",
    "Grid V low","Grid V high","Grid underfreq","Grid overfreq",
    "Overcurrent protect","Bus V low","Soft start fail","DC in AC output",
    "Batt open","Control I sensor","Output V low"};
  for(int i=0;i<16;i++){
    if(e1&(1<<i)){if(errs.length()>0)errs+=", ";errs+=b1[i];}
    if(e2&(1<<i)){if(errs.length()>0)errs+=", ";errs+=b2[i];}
  }
  return errs.length()>0?errs:"No errors";
}

String decodeWarnings(uint16_t w1,uint16_t w2){
  if(w1==0xFFFF||w2==0xFFFF)return "--";
  if(w1==0&&w2==0)return "No warnings";
  const char*w1b[]={"Fan locked on","Fan2 locked on","Batt overcharged","Low battery",
    "Overload","Power derating","Charger low batt","Charger high PV",
    "Charger overload","Charger overtemp","Charger comm error"};
  String warns="";
  for(int i=0;i<11;i++){if(w1&(1<<i)){if(warns.length()>0)warns+=", ";warns+=w1b[i];}}
  return warns.length()>0?warns:"No warnings";
}

String decodeChgErrors(uint16_t e){
  if(e==0xFFFF)return "--";if(e==0)return "No errors";
  const char*ce[]={"Hardware protect","Overcurrent","I sensor error","Overtemp",
    "PV V high","PV V low","Batt V high","Batt V low","I uncontrolled","Parameter error"};
  String errs="";
  for(int i=0;i<10;i++){if(e&(1<<i)){if(errs.length()>0)errs+=", ";errs+=ce[i];}}
  return errs.length()>0?errs:"No errors";
}

String battTypeStr(uint16_t r){
  if(r==0xFFFF)return "--";
  switch(r){case 0:return "User defined";case 1:return "Use defined";case 2:return "Lithium";
  case 3:return "Sealed lead";case 4:return "AGM";case 5:return "GEL";case 6:return "Flooded";
  default:return "Type "+String(r);}
}

String buildRecommendations(){
  String r="";
  if(inv.err1||inv.err2||inv.err3)r+="⛔ Помилка інвертора: "+decodeErrors(inv.err1,inv.err2,inv.err3)+"\n";
  if(inv.chgErr)r+="⛔ Помилка зарядного пристрою: "+decodeChgErrors(inv.chgErr)+"\n";
  if(inv.warn1||inv.warn2)r+="⚠️ Попередження: "+decodeWarnings(inv.warn1,inv.warn2)+"\n";
  if(inv.tempAC>60)r+="🌡️ Перегрів AC радіатора: "+String(inv.tempAC)+"C — перевірте охолодження\n";
  if(inv.tempDC>60)r+="🌡️ Перегрів DC радіатора: "+String(inv.tempDC)+"C — зменште навантаження\n";
  float lowV=(inv.ctrl[26]!=0xFFFF)?inv.ctrl[26]/10.0f:0.0f;
  float highV=(inv.ctrl[27]!=0xFFFF)?inv.ctrl[27]/10.0f:0.0f;
  if(lowV>0&&inv.vBatt<=lowV+0.5f&&strcmp(inv.batModeUi,"DISCHARGE")==0)
    r+="🔋 Батарея розряджена: "+String(inv.vBatt,2)+"V (поріг "+String(lowV,1)+"V) — зменште навантаження!\n";
  if(highV>0&&inv.vBatt>=highV-0.3f&&inv.effectiveSolarW>200)
    r+="☀️ Батарея повна і є сонце — гарний час увімкнути потужне навантаження\n";
  if(inv.gridPowerShow<-300)
    r+="⚡ Великий імпорт з мережі: "+String(-inv.gridPowerShow)+"W — перевірте навантаження та пріоритет\n";
  if(inv.workState==4)r+="🔌 Інвертор у BYPASS — навантаження живиться безпосередньо, батарея не використовується\n";
  if(inv.workState==6&&inv.effectiveSolarW>300)
    r+="🔆 Зарядка від мережі за наявності сонця — розгляньте пріоритет Solar first\n";
  if(r.length()==0)r="✅ Система працює нормально";
  return r;
}

inline bool isBatRegDead(uint16_t raw){return raw==0xFFFF;}
inline float ampsFrom74(int16_t r74){return (std::abs(r74)>50)?(float)r74/10.0f:(float)r74;}
inline bool battVoltageOk(float v){
  return (v>=9.0f&&v<=17.0f)||(v>=18.0f&&v<=35.0f)||(v>=36.0f&&v<=70.0f);
}
static float ratedPowerW(){
  uint16_t r=inv.inv[76];
  if(r!=0xFFFF&&r>=500&&r<=12000)return (float)r;
  float mp=inv.machinePower.toFloat();
  if(mp>=500.0f)return mp;return 3000.0f;
}
static float maxBatW(){return ratedPowerW()*1.3f+300.0f;}
static float maxBatA(){return fminf(250.0f,maxBatW()/fmaxf(9.0f,inv.inv[4]/10.0f));}
static bool r73Sane(){return !isBatRegDead(inv.inv[72])&&std::abs((int16_t)inv.inv[72])<=(int)maxBatW();}
static bool r74Sane(){return !isBatRegDead(inv.inv[73])&&std::fabs(ampsFrom74((int16_t)inv.inv[73]))<=maxBatA();}

int calculateBatFlowSign(){
  if(inv.workState==6)return +1;
  if(inv.workState==2&&inv.vGrid<80.0f)return -1;
  if(r73Sane()&&(int16_t)inv.inv[72]!=0)return ((int16_t)inv.inv[72]<0)?+1:-1;
  if(r74Sane()&&(int16_t)inv.inv[73]!=0)return ((int16_t)inv.inv[73]<0)?+1:-1;
  return 0;
}

void applyBatFlowSign(float*batPower,float*ampTruth){
  int flow=calculateBatFlowSign();
  if(flow==0||!batPower)return;
  float mag=std::fabs(*batPower);
  *batPower=(flow>0)?mag:-mag;
  if(ampTruth&&std::fabs(*ampTruth)>=0.2f)
    *ampTruth=(flow>0)?std::fabs(*ampTruth):-std::fabs(*ampTruth);
}

void updateBatteryTruth(){
  constexpr float kBatThrW=3.0f;
  const float vBat=inv.inv[4]/10.0f;
  const bool vOk=battVoltageOk(vBat);
  const float pMax=maxBatW();
  float batPower=0.0f,ampTruth=0.0f;bool have=false;
  if(r73Sane()&&std::abs((int16_t)inv.inv[72])>=15){
    have=true;batPower=(float)std::abs((int16_t)inv.inv[72]);
    if(r74Sane())ampTruth=ampsFrom74((int16_t)inv.inv[73]);
    else if(vOk&&batPower>0)ampTruth=batPower/vBat;
    applyBatFlowSign(&batPower,&ampTruth);
  }else if(r74Sane()){
    have=true;ampTruth=ampsFrom74((int16_t)inv.inv[73]);
    if(vOk)batPower=(float)roundf(vBat*std::fabs(ampTruth));
    applyBatFlowSign(&batPower,&ampTruth);
  }
  inv.batRegsValid=have;
  if(!have&&vOk&&(inv.workState==2||inv.workState==3)){
    float gridIn=(inv.vGrid>80.0f&&inv.gridPower<0)?(float)(-inv.gridPower):0.0f;
    float est=(float)inv.effectiveSolarW+gridIn-(float)inv.pLoad-30.0f;
    if(est>pMax)est=pMax;if(est<-pMax)est=-pMax;
    if(std::fabs(est)>=kBatThrW){have=true;batPower=est;ampTruth=est/vBat;}
  }
  if(!have){inv.battPowerFiltered=0.0f;batPower=0.0f;ampTruth=0.0f;}
  else if(std::fabs(batPower)<kBatThrW){
    inv.battPowerFiltered*=0.15f;
    if(std::fabs(inv.battPowerFiltered)<kBatThrW)inv.battPowerFiltered=batPower;
    batPower=inv.battPowerFiltered;
  }else if(inv.battPowerFiltered*batPower<0.0f||std::fabs(inv.battPowerFiltered)<0.01f){
    inv.battPowerFiltered=batPower;
  }else{inv.battPowerFiltered=inv.battPowerFiltered*0.5f+batPower*0.5f;}
  if(have)batPower=inv.battPowerFiltered;
  inv.battPowerMeasured=batPower;
  inv.battWattDisplay=(int)roundf(std::fabs(batPower));
  if(std::fabs(ampTruth)>=0.2f)inv.battAmpDisplay=std::fabs(ampTruth);
  else if(inv.battWattDisplay>0&&vOk)inv.battAmpDisplay=(float)inv.battWattDisplay/vBat;
  else inv.battAmpDisplay=0.0f;
  if(batPower>kBatThrW){inv.batModeUi="CHARGE";inv.batClassUi="green";}
  else if(batPower<-kBatThrW){inv.batModeUi="DISCHARGE";inv.batClassUi="red";}
  else{inv.batModeUi="IDLE";inv.batClassUi="blue";
    inv.battPowerMeasured=0.0f;inv.battWattDisplay=0;inv.battAmpDisplay=0.0f;}
  inv.battPowerSigned=inv.battPowerMeasured;
}

void updateBatteryMetrics(){
  if(!snap.valid)return;
  updateBatteryTruth();
  constexpr float kBatThrW=3.0f;
  const float gridImportW=(snap.gridPowerRaw<0)?(float)(-snap.gridPowerRaw):0.0f;
  const bool gridRelayOn=(inv.inv[37]!=0xFFFF)&&(inv.inv[37]!=0);
  const bool gridOn=((snap.vGrid>80.0f)&&gridRelayOn)||gridImportW>20.0f;
  float chargeW=(inv.battPowerMeasured>kBatThrW)?inv.battPowerMeasured:0.0f;
  float dischargeW=(inv.battPowerMeasured<-kBatThrW)?-inv.battPowerMeasured:0.0f;
  if(!gridOn&&inv.battPowerMeasured<-kBatThrW&&snap.loadW>0){
    float selfForCap=inv.selfConsumptionFiltered;
    if(selfForCap<8.0f)selfForCap=10.0f+(float)snap.loadW*0.38f;
    if(selfForCap>55.0f)selfForCap=55.0f;
    float cap=fmaxf(0.0f,(float)snap.loadW+selfForCap-(float)snap.solarW);
    float mag=-inv.battPowerMeasured;
    if(cap>=kBatThrW&&mag>cap){
      inv.battPowerMeasured=-cap;inv.battPowerSigned=inv.battPowerMeasured;
      inv.battWattDisplay=(int)roundf(cap);
      if(inv.vBatt>10.0f)inv.battAmpDisplay=cap/inv.vBatt;
      dischargeW=cap;chargeW=0.0f;
    }
  }
  float gridTotalIn=gridImportW;
  if(gridOn&&snap.workState==6){
    float realImport=fmaxf(0.0f,(float)snap.loadW+chargeW-dischargeW-(float)snap.solarW);
    gridTotalIn=fmaxf(gridImportW,realImport);
  }
  inv.gridPowerShow=snap.gridPowerRaw;
  if(gridOn&&snap.workState==6&&gridTotalIn>10.0f)inv.gridPowerShow=-(int)roundf(gridTotalIn);
  float gridInForSelf=gridOn?gridTotalIn:0.0f;
  float selfConsumptionRaw=fmaxf(0.0f,gridInForSelf+(float)snap.solarW-(float)snap.loadW-chargeW+dischargeW);
  if(snap.loadW>15){
    if(!gridOn&&dischargeW>0.0f){
      float selfFromBat=dischargeW-(float)snap.loadW+(float)snap.solarW;
      if(selfFromBat>5.0f&&selfFromBat<60.0f)selfConsumptionRaw=selfFromBat;
    }
    if(!inv.batRegsValid)selfConsumptionRaw=fmaxf(selfConsumptionRaw,10.0f+(float)snap.loadW*0.025f);
    else if(selfConsumptionRaw<8.0f)selfConsumptionRaw=fmaxf(selfConsumptionRaw,15.0f+(float)snap.loadW*0.01f);
  }
  if(selfConsumptionRaw>50.0f)selfConsumptionRaw=50.0f;
  inv.selfConsumptionFiltered=(inv.selfConsumptionFiltered<=0.01f)?selfConsumptionRaw:(inv.selfConsumptionFiltered*0.7f+selfConsumptionRaw*0.3f);
  inv.selfConsumption=inv.selfConsumptionFiltered;
  float netFromGrid=gridOn?((float)snap.loadW+chargeW-dischargeW-(float)snap.solarW):((float)snap.loadW-(float)snap.solarW);
  inv.systemBalanceShow=(int)roundf(-netFromGrid);
  inv.pNet=inv.systemBalanceShow;
}

void pollModbusOnce(){
  bool ok=true;
  if(!readModbusBlock(25201,80,invBuf))ok=false;delay(40);
  if(!readModbusBlock(15201,20,pvBuf))ok=false;delay(40);
  if(!readModbusBlock(20101,50,ctrlBuf))ok=false;
  if(!ok){digitalWrite(LED_PIN,HIGH);return;}
  xSemaphoreTake(dataMutex,portMAX_DELAY);
  memcpy(inv.inv,invBuf,sizeof(inv.inv));
  memcpy(inv.pv,pvBuf,sizeof(inv.pv));
  memcpy(inv.ctrl,ctrlBuf,sizeof(inv.ctrl));
  inv.workState=inv.inv[0];
  inv.vBatt=inv.inv[4]/10.0f;
  if(std::isnan(inv.vBattFiltered))inv.vBattFiltered=inv.vBatt;
  else inv.vBattFiltered=inv.vBattFiltered*0.75f+inv.vBatt*0.25f;
  inv.vBatt=inv.vBattFiltered;
  inv.vOut=inv.inv[5]/10.0f;inv.vGrid=inv.inv[6]/10.0f;inv.vBus=inv.inv[7]/10.0f;
  inv.gridPower=(int16_t)inv.inv[13];inv.pLoad=inv.inv[14];
  inv.loadPercent=constrain(inv.inv[15],0,100);
  inv.tempAC=(int16_t)inv.inv[32];inv.tempDC=(int16_t)inv.inv[34];
  inv.vPV=inv.pv[4]/10.0f;inv.solarPowerRaw=inv.pv[7];
  inv.effectiveSolarW=(inv.pv[1]==1||inv.pv[1]==2)?inv.solarPowerRaw:0;
  inv.pvEnergy=(float)inv.pv[16]*1000.0f+(float)inv.pv[17]/10.0f;
  inv.offgridEnable=inv.ctrl[0];inv.energyMode=inv.ctrl[8];inv.chargerPriority=inv.ctrl[42];
  inv.err1=inv.inv[60];inv.err2=inv.inv[61];inv.err3=inv.inv[62];
  inv.warn1=inv.inv[64];inv.warn2=inv.inv[65];
  inv.chgErr=inv.pv[12];inv.chgWarn=inv.pv[13];
  snap.valid=true;snap.workState=inv.workState;snap.vGrid=inv.vGrid;
  snap.gridPowerRaw=inv.gridPower;snap.loadW=inv.pLoad;snap.solarW=inv.effectiveSolarW;
  updateBatteryMetrics();
  switch(inv.workState){
    case 0:inv.stateStr="POWER ON";inv.workModeShort="ON";break;
    case 1:inv.stateStr="SELFTEST";inv.workModeShort="TEST";break;
    case 2:inv.stateStr="OFF GRID";inv.workModeShort="BATT";break;
    case 3:inv.stateStr="GRID TIE";inv.workModeShort="TIE";break;
    case 4:inv.stateStr="BYPASS";inv.workModeShort="BYPASS";break;
    case 5:inv.stateStr="STOP";inv.workModeShort="STOP";break;
    case 6:inv.stateStr="GRID CHRG";inv.workModeShort="CHRG";break;
    default:inv.stateStr="UNKNOWN";inv.workModeShort="UNK";break;
  }
  xSemaphoreGive(dataMutex);digitalWrite(LED_PIN,LOW);
}

void modbusTask(void*pvParameters){
  (void)pvParameters;
  for(;;){
    if(pendingCounterReset){
      pendingCounterReset=false;
      bool ok1=(node.writeSingleRegister(20213,1)==node.ku8MBSuccess);delay(100);
      bool ok2=(node.writeSingleRegister(10112,1)==node.ku8MBSuccess);
      Serial.printf("Counter reset: inverter=%d charger=%d\n",ok1?1:0,ok2?1:0);delay(100);
    }
    if(!webUpdateActive&&(millis()-lastUpdateMs>=MODBUS_UPDATE_INTERVAL_MS)){
      lastUpdateMs=millis();pollModbusOnce();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inverter Dashboard</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:'Consolas','Menlo',monospace;padding:14px;margin:0}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:12px}
.box{background:#1c1c1c;border-radius:10px;padding:12px;text-align:center;align-items:center}
.label{color:#888;font-size:13px}
.value{font-size:14px;margin-top:8px;font-weight:bold;white-space:pre-line;line-height:1.6;text-align:center}
.green{color:#00ff88}.red{color:#ff4d4d}.blue{color:#4da3ff}.yellow{color:#ffd166}
.balance.pos{color:#00ff88}.balance.neg{color:#ff4d4d}.balance.zero{color:#4da3ff}
.rec-tile{font-size:12px;line-height:1.5;text-align:left;white-space:pre-line;margin-top:6px}
.upd{display:flex;flex-direction:column;gap:6px;align-items:center;margin-top:8px;width:100%}
.upd input[type=file]{color:#888;font-size:11px;max-width:100%}
.btn-upd{background:#00ff88;color:#000;border:none;padding:8px 16px;border-radius:8px;cursor:pointer;font-size:12px;font-weight:bold;width:100%}
.btn-warn{background:#ffd166}.btn-gray{background:#555;color:#fff}
.prog{height:8px;background:#333;border-radius:4px;margin-top:8px;overflow:hidden;display:none;width:100%}
.prog div{height:100%;width:0%;background:#00ff88}
.updstatus{margin-top:8px;font-size:11px;color:#888}
.cols{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
.panel{background:#1c1c1c;border-radius:10px;padding:12px;font-size:12px}
.panel h4{margin:0 0 10px 0;color:#aaa;font-size:12px;font-weight:normal;letter-spacing:1px}
.row{display:flex;justify-content:space-between;gap:8px;padding:5px 0;border-bottom:1px solid #242424}
.row:last-child{border-bottom:none}
.row .k{color:#888;white-space:nowrap}
.row .v{color:#e6e6e6;font-weight:bold;text-align:right}
@media(max-width:900px){.cols{grid-template-columns:1fr 1fr}.grid{grid-template-columns:1fr 1fr}}
</style></head><body>
<div class="grid">
  <div class="box"><div class="label">SOLAR</div><div class="value green" id="solar">0V
0W</div></div>
  <div class="box"><div class="label">GRID</div><div class="value blue" id="grid">0V
0W</div></div>
  <div class="box"><div class="label">SYSTEM BALANCE</div><div class="value balance zero" id="balance">---
0W | 0%</div></div>
  <div class="box"><div class="label">LOAD</div><div class="value red" id="load">0V
0W</div></div>
  <div class="box"><div class="label">BAT</div><div class="value yellow" id="bat">0V
0W</div></div>
  <div class="box"><div class="label">ENERGY STATS</div><div class="value green" id="pvEnergy">PV ENERGY - 0 kWh
ACCUM load - 0 kWh
ACCUM DISCHARGE - 0 kWh</div></div>
  <div class="box"><div class="label">INV LOSS / AC TEMP</div><div class="value yellow" id="lossTemp">INV LOSS - 0 W
AC RAD TEMP - 0C</div></div>
  <div class="box"><div class="label">SYSTEM SETTINGS</div><div class="value" id="settings">Offgrid work enable - ---
Current mode - ---
Charger priority - ---</div></div>
  <div class="box"><div class="label">RECOMMENDATIONS</div><div class="value rec-tile" id="recs">--</div></div>
</div>
<div class="cols">
  <div class="panel"><h4>BATTERY SETTINGS</h4>
    <div class="row"><span class="k">Battery type</span><span class="v" id="bs_type">--</span></div>
    <div class="row"><span class="k">Battery AH</span><span class="v" id="bs_ah">--</span></div>
    <div class="row"><span class="k">Battery low V</span><span class="v" id="bs_lowv">--</span></div>
    <div class="row"><span class="k">Battery high V</span><span class="v" id="bs_highv">--</span></div>
    <div class="row"><span class="k">Battery low return V</span><span class="v" id="bs_lowret">--</span></div>
    <div class="row"><span class="k">Max charger I</span><span class="v" id="bs_maxchg">--</span></div>
    <div class="row"><span class="k">Max discharger I</span><span class="v" id="bs_maxdis">--</span></div>
    <div class="row"><span class="k">Grid max input I</span><span class="v" id="bs_gridin">--</span></div>
    <div class="row"><span class="k">Grid max output I</span><span class="v" id="bs_gridout">--</span></div>
    <div class="row"><span class="k">Absorb charger I</span><span class="v" id="bs_absorb">--</span></div>
    <div class="row"><span class="k">Max combine I</span><span class="v" id="bs_combine">--</span></div>
    <div class="row"><span class="k">Charger time</span><span class="v" id="bs_chgtime">--</span></div>
    <div class="row"><span class="k">Discharger time</span><span class="v" id="bs_distime">--</span></div>
  </div>
  <div class="panel"><h4>CHARGER MESSAGE (LIVE)</h4>
    <div class="row"><span class="k">Work state</span><span class="v" id="ch_work">--</span></div>
    <div class="row"><span class="k">Mppt state</span><span class="v" id="ch_mppt">--</span></div>
    <div class="row"><span class="k">Charging state</span><span class="v" id="ch_state">--</span></div>
    <div class="row"><span class="k">PV voltage</span><span class="v" id="ch_pvv">--</span></div>
    <div class="row"><span class="k">Battery voltage</span><span class="v" id="ch_battv">--</span></div>
    <div class="row"><span class="k">Current</span><span class="v" id="ch_cur">--</span></div>
    <div class="row"><span class="k">Power</span><span class="v" id="ch_pow">--</span></div>
    <div class="row"><span class="k">Radiator temp</span><span class="v" id="ch_rad">--</span></div>
    <div class="row"><span class="k">External temp</span><span class="v" id="ch_ext">--</span></div>
    <div class="row"><span class="k">Battery Relay</span><span class="v" id="ch_brel">--</span></div>
    <div class="row"><span class="k">PV Relay</span><span class="v" id="ch_pvrel">--</span></div>
    <div class="row"><span class="k">BattVol Grade</span><span class="v" id="ch_grade">--</span></div>
    <div class="row"><span class="k">Rated Current</span><span class="v" id="ch_rcur">--</span></div>
    <div class="row"><span class="k">ACCUM power</span><span class="v" id="ch_acc">--</span></div>
    <div class="row"><span class="k">Error</span><span class="v red" id="ch_err">--</span></div>
    <div class="row"><span class="k">Warning</span><span class="v yellow" id="ch_warn">--</span></div>
  </div>
  <div class="panel"><h4>INVERTER MESSAGE A</h4>
    <div class="row"><span class="k">Work state</span><span class="v" id="ia_work">--</span></div>
    <div class="row"><span class="k">AC voltage grade</span><span class="v" id="ia_acgrade">--</span></div>
    <div class="row"><span class="k">Rated power</span><span class="v" id="ia_rated">--</span></div>
    <div class="row"><span class="k">Battery voltage</span><span class="v" id="ia_battv">--</span></div>
    <div class="row"><span class="k">Inverter voltage</span><span class="v" id="ia_invv">--</span></div>
    <div class="row"><span class="k">Grid voltage</span><span class="v" id="ia_gridv">--</span></div>
    <div class="row"><span class="k">BUS voltage</span><span class="v" id="ia_busv">--</span></div>
    <div class="row"><span class="k">Control current</span><span class="v" id="ia_ictrl">--</span></div>
    <div class="row"><span class="k">Inverter current</span><span class="v" id="ia_iinv">--</span></div>
    <div class="row"><span class="k">Grid current</span><span class="v" id="ia_igrid">--</span></div>
    <div class="row"><span class="k">Load current</span><span class="v" id="ia_iload">--</span></div>
    <div class="row"><span class="k">PInverter</span><span class="v" id="ia_pinv">--</span></div>
    <div class="row"><span class="k">PGrid</span><span class="v blue" id="ia_pgrid">--</span></div>
    <div class="row"><span class="k">PLoad</span><span class="v" id="ia_pload">--</span></div>
    <div class="row"><span class="k">Load percent</span><span class="v" id="ia_lpct">--</span></div>
    <div class="row"><span class="k">SInverter</span><span class="v" id="ia_sinv">--</span></div>
    <div class="row"><span class="k">SGrid</span><span class="v" id="ia_sgrid">--</span></div>
    <div class="row"><span class="k">SLoad</span><span class="v" id="ia_sload">--</span></div>
  </div>
  <div class="panel"><h4>INVERTER MESSAGE B</h4>
    <div class="row"><span class="k">AC radiator temp</span><span class="v" id="ib_tac">--</span></div>
    <div class="row"><span class="k">Transformer temp</span><span class="v" id="ib_ttr">--</span></div>
    <div class="row"><span class="k">DC radiator temp</span><span class="v" id="ib_tdc">--</span></div>
    <div class="row"><span class="k">Inverter relay</span><span class="v" id="ib_rinv">--</span></div>
    <div class="row"><span class="k">Grid relay</span><span class="v" id="ib_rgrid">--</span></div>
    <div class="row"><span class="k">Load relay</span><span class="v" id="ib_rload">--</span></div>
    <div class="row"><span class="k">N-Line relay</span><span class="v" id="ib_rnline">--</span></div>
    <div class="row"><span class="k">DC relay</span><span class="v" id="ib_rdc">--</span></div>
    <div class="row"><span class="k">Earth relay</span><span class="v" id="ib_rearth">--</span></div>
    <div class="row"><span class="k">QInverter</span><span class="v" id="ib_qinv">--</span></div>
    <div class="row"><span class="k">QGrid</span><span class="v" id="ib_qgrid">--</span></div>
    <div class="row"><span class="k">QLoad</span><span class="v" id="ib_qload">--</span></div>
    <div class="row"><span class="k">ACCUM charge</span><span class="v" id="ib_accchg">--</span></div>
    <div class="row"><span class="k">ACCUM discharge</span><span class="v" id="ib_accdis">--</span></div>
    <div class="row"><span class="k">ACCUM buy</span><span class="v" id="ib_accbuy">--</span></div>
    <div class="row"><span class="k">ACCUM sell</span><span class="v" id="ib_accsell">--</span></div>
    <div class="row"><span class="k">ACCUM load</span><span class="v" id="ib_accload">--</span></div>
    <div class="row"><span class="k">ACCUM self_use</span><span class="v" id="ib_accself">--</span></div>
    <div class="row"><span class="k">ACCUM PV_sell</span><span class="v" id="ib_accpvsell">--</span></div>
    <div class="row"><span class="k">ACCUM grid_charge</span><span class="v" id="ib_accgridchg">--</span></div>
    <div class="row"><span class="k">Batt power</span><span class="v green" id="ib_bpow">--</span></div>
    <div class="row"><span class="k">Batt current</span><span class="v" id="ib_bcur">--</span></div>
    <div class="row"><span class="k">Inverter Hz</span><span class="v" id="ib_ihz">--</span></div>
    <div class="row"><span class="k">Grid Hz</span><span class="v" id="ib_ghz">--</span></div>
    <div class="row"><span class="k">Error</span><span class="v red" id="ib_err">--</span></div>
    <div class="row"><span class="k">Warning</span><span class="v yellow" id="ib_warn">--</span></div>
  </div>
</div>
<div class="grid">
  <div class="box"><div class="label">FIRMWARE UPDATE</div>
    <div class="upd">
      <input type="file" id="fwfile" accept=".bin">
      <button class="btn-upd" onclick="doUpdate()">Оновити</button>
    </div>
    <div class="prog" id="progbox"><div id="prog"></div></div>
    <div class="updstatus" id="updStatus">Оберіть файл .bin</div>
  </div>
  <div class="box"><div class="label">COUNTERS RESET</div>
    <div class="upd">
      <button class="btn-upd btn-warn" onclick="resetCounters()">Скинути лічильники</button>
    </div>
    <div class="updstatus" id="cntStatus">Обнулення ACCUM та PV ENERGY (незворотно!)</div>
  </div>
  <div class="box"><div class="label">WI-FI RESET</div>
    <div class="upd">
      <button class="btn-upd btn-gray" onclick="resetWifi()">Скинути Wi-Fi</button>
    </div>
    <div class="updstatus">Налаштування мережі буде видалено</div>
  </div>
</div>
<script>
function s(id,v){document.getElementById(id).innerText=v;}
function setBalance(w,m,l){
  let e=document.getElementById("balance");
  e.innerText=m+"\n"+w+"W | "+l+"%";
  e.className="value balance "+(w>5?"pos":(w<-5?"neg":"zero"));
}
async function resetWifi(){
  if(confirm("Скинути Wi-Fi?")){await fetch("/reset_wifi");alert("Перезавантаження в режим точки доступу...");location.reload();}
}
async function resetCounters(){
  if(!confirm("Скинути всі накопичувальні лічильники інвертора?\nACCUM та PV ENERGY обнуляться.\nДія НЕЗВОРОТНА!"))return;
  await fetch("/reset_counters");
  s("cntStatus","✅ Команду надіслано — лічильники обнуляться протягом кількох секунд");
}
function doUpdate(){
  let f=document.getElementById("fwfile").files[0];
  if(!f){alert("Оберіть файл прошивки (.bin)");return;}
  if(!confirm("Почати оновлення прошивки?\nПристрій перезавантажиться після завершення."))return;
  let fd=new FormData();fd.append("update",f,f.name);
  let xhr=new XMLHttpRequest();
  document.getElementById("progbox").style.display="block";
  document.getElementById("prog").style.width="0%";
  s("updStatus","Завантаження прошивки...");
  xhr.open("POST","/update");
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable){let p=Math.round(e.loaded/e.total*100);document.getElementById("prog").style.width=p+"%";s("updStatus","Завантаження: "+p+"%");}
  };
  xhr.onload=function(){
    if(xhr.status==200&&xhr.responseText=="OK"){document.getElementById("prog").style.width="100%";s("updStatus","✅ Прошивку записано! Перезавантаження...");setTimeout(function(){location.reload();},10000);}
    else{s("updStatus","❌ Помилка оновлення: "+xhr.responseText);}
  };
  xhr.onerror=function(){s("updStatus","❌ Помилка з'єднання");};
  xhr.send(fd);
}
async function update(){
  try{
    let r=await fetch("/api");let d=await r.json();
    s("solar",d.pvvolt+"V\n"+d.solar+"W");
    s("grid",d.grid+"V\n"+d.gridPowerShow+"W");
    s("load",d.loadVolt+"V\n"+d.loadPower+"W");
    s("lossTemp","INV LOSS - "+d.invLoss+" W\nAC RAD TEMP - "+d.tempAC+"C");
    s("pvEnergy","PV ENERGY - "+d.pvEnergy+" kWh\nACCUM load - "+d.accumLoad+"\nACCUM DISCHARGE - "+d.accumDischarge);
    let b=document.getElementById("bat");b.className="value "+d.batClass;
    b.innerText=d.batMode+"\n"+d.batteryVolt+" V / "+d.battAmp+" A"+d.batSuffix+"\n"+d.batPowerDisplay+" W";
    setBalance(d.net,d.workMode||d.state,d.loadPercent);
    s("settings","Offgrid work enable - "+d.offgridEnable+"\nCurrent mode - "+d.energyMode+"\nCharger priority - "+d.chargerPriority);
    s("recs",d.recs);
    s("bs_type",d.bs_type);s("bs_ah",d.bs_ah);s("bs_lowv",d.bs_lowv);s("bs_highv",d.bs_highv);
    s("bs_lowret",d.bs_lowret);s("bs_maxchg",d.bs_maxchg);s("bs_maxdis",d.bs_maxdis);
    s("bs_gridin",d.bs_gridin);s("bs_gridout",d.bs_gridout);s("bs_absorb",d.bs_absorb);
    s("bs_combine",d.bs_combine);s("bs_chgtime",d.bs_chgtime);s("bs_distime",d.bs_distime);
    s("ch_work",d.ch_work);s("ch_mppt",d.ch_mppt);s("ch_state",d.ch_state);s("ch_pvv",d.ch_pvv);
    s("ch_battv",d.ch_battv);s("ch_cur",d.ch_cur);s("ch_pow",d.ch_pow);s("ch_rad",d.ch_rad);
    s("ch_ext",d.ch_ext);s("ch_brel",d.ch_brel);s("ch_pvrel",d.ch_pvrel);s("ch_grade",d.ch_grade);
    s("ch_rcur",d.ch_rcur);s("ch_acc",d.ch_acc);s("ch_err",d.ch_err);s("ch_warn",d.ch_warn);
    s("ia_work",d.ia_work);s("ia_acgrade",d.ia_acgrade);s("ia_rated",d.ia_rated);s("ia_battv",d.ia_battv);
    s("ia_invv",d.ia_invv);s("ia_gridv",d.ia_gridv);s("ia_busv",d.ia_busv);s("ia_ictrl",d.ia_ictrl);
    s("ia_iinv",d.ia_iinv);s("ia_igrid",d.ia_igrid);s("ia_iload",d.ia_iload);s("ia_pinv",d.ia_pinv);
    s("ia_pgrid",d.ia_pgrid);s("ia_pload",d.ia_pload);s("ia_lpct",d.ia_lpct);s("ia_sinv",d.ia_sinv);
    s("ia_sgrid",d.ia_sgrid);s("ia_sload",d.ia_sload);
    s("ib_tac",d.ib_tac);s("ib_ttr",d.ib_ttr);s("ib_tdc",d.ib_tdc);s("ib_rinv",d.ib_rinv);
    s("ib_rgrid",d.ib_rgrid);s("ib_rload",d.ib_rload);s("ib_rnline",d.ib_rnline);
    s("ib_rdc",d.ib_rdc);s("ib_rearth",d.ib_rearth);s("ib_qinv",d.ib_qinv);s("ib_qgrid",d.ib_qgrid);
    s("ib_qload",d.ib_qload);s("ib_accchg",d.ib_accchg);s("ib_accdis",d.ib_accdis);
    s("ib_accbuy",d.ib_accbuy);s("ib_accsell",d.ib_accsell);s("ib_accload",d.ib_accload);
    s("ib_accself",d.ib_accself);s("ib_accpvsell",d.ib_accpvsell);s("ib_accgridchg",d.ib_accgridchg);
    s("ib_bpow",d.ib_bpow);s("ib_bcur",d.ib_bcur);s("ib_ihz",d.ib_ihz);s("ib_ghz",d.ib_ghz);
    s("ib_err",d.ib_err);s("ib_warn",d.ib_warn);
  }catch(e){}
}
setInterval(update,3000);update();
</script></body></html>
)rawliteral";

const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Налаштування Wi-Fi</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:20px;text-align:center}
.card{background:#1c1c1c;border-radius:12px;padding:20px;max-width:360px;margin:0 auto}
select,input,button{width:100%;box-sizing:border-box;padding:12px;margin:8px 0;background:#333;color:#fff;border:1px solid #555;border-radius:8px;font-size:14px}
button{background:#00ff88;color:#000;font-weight:bold;cursor:pointer;border:none;margin-top:15px}
</style></head><body>
<div class="card">
  <h2>Налаштування Wi-Fi</h2>
  <form action="/save_wifi" method="POST">
    <label>Оберіть мережу:</label>
    <select name="ssid" id="ssid_select" required><option value="">Пошук мереж...</option></select>
    <label>Пароль:</label>
    <input type="password" name="password" placeholder="Введіть пароль">
    <button type="submit">Зберегти</button>
  </form>
</div>
<script>
async function scan(){
  let r=await fetch("/scan_wifi");let n=await r.json();
  let s=document.getElementById("ssid_select");s.innerHTML="";
  if(!n.length){s.innerHTML="<option value=''>Мережі не знайдено</option>";return;}
  n.forEach(x=>{let o=document.createElement("option");o.value=x.ssid;o.innerText=x.ssid+" ("+x.rssi+" dBm)";s.appendChild(o);});
}
scan();
</script></body></html>
)rawliteral";

void triggerReboot(){needRestart=true;restartRequestedMs=millis();}

void startSetupAP(){
  setupMode=true;WiFi.mode(WIFI_AP);WiFi.softAP("Inverter-Setup");
  dnsServer.start(53,"*",WiFi.softAPIP());
  server.on("/",HTTP_GET,[](AsyncWebServerRequest*req){req->send_P(200,"text/html",SETUP_HTML);});
  server.on("/scan_wifi",HTTP_GET,[](AsyncWebServerRequest*req){
    int n=WiFi.scanNetworks();
    AsyncResponseStream*res=req->beginResponseStream("application/json");
    res->print("[");
    for(int i=0;i<n;++i){if(i>0)res->print(",");res->printf("{\"ssid\":\"%s\",\"rssi\":%d}",WiFi.SSID(i).c_str(),WiFi.RSSI(i));}
    res->print("]");WiFi.scanDelete();req->send(res);
  });
  server.on("/save_wifi",HTTP_POST,[](AsyncWebServerRequest*req){
    if(req->hasParam("ssid",true)&&req->hasParam("password",true)){
      preferences.begin("wifi_config",false);
      preferences.putString("ssid",req->getParam("ssid",true)->value());
      preferences.putString("password",req->getParam("password",true)->value());
      preferences.end();
      req->send(200,"text/html","<h3 style='color:white;text-align:center;'>Налаштування збережено! Перезавантаження...</h3>");
      triggerReboot();
    }else req->send(400,"text/plain","Bad Request");
  });
  server.onNotFound([](AsyncWebServerRequest*req){req->send_P(200,"text/html",SETUP_HTML);});
  server.begin();
}

void onUpdateUpload(AsyncWebServerRequest*request,String filename,size_t index,uint8_t*data,size_t len,bool final){
  if(index==0){
    webUpdateActive=true;Serial.printf("Update Start: %s\n",filename.c_str());
    if(!Update.begin((ESP.getFreeSketchSpace()-0x1000)&0xFFFFF000))Update.printError(Serial);
  }
  if(!Update.hasError()){if(Update.write(data,len)!=len)Update.printError(Serial);}
  if(final){
    if(Update.end(true))Serial.printf("Update Success: %u bytes\n",(unsigned)(index+len));
    else{Update.printError(Serial);webUpdateActive=false;}
  }
}

void setupWebServer(){
  server.on("/",HTTP_GET,[](AsyncWebServerRequest*req){req->send_P(200,"text/html",INDEX_HTML);});
  server.on("/reset_wifi",HTTP_GET,[](AsyncWebServerRequest*req){
    preferences.begin("wifi_config",false);preferences.clear();preferences.end();
    req->send(200,"text/plain","OK");triggerReboot();
  });
  server.on("/reset_counters",HTTP_GET,[](AsyncWebServerRequest*req){
    pendingCounterReset=true;req->send(200,"text/plain","OK");
  });
  server.on("/update",HTTP_POST,[](AsyncWebServerRequest*req){
    if(Update.hasError()){webUpdateActive=false;req->send(500,"text/plain","FAIL");}
    else{req->send(200,"text/plain","OK");triggerReboot();}
  },onUpdateUpload);

  server.on("/api",HTTP_GET,[](AsyncWebServerRequest*req){
    bool locked=(dataMutex!=nullptr)&&(xSemaphoreTake(dataMutex,pdMS_TO_TICKS(200))==pdTRUE);
    float loadVoltage=(inv.workState==2||inv.workState==3)?inv.vOut:((inv.workState==4||inv.workState==6)?inv.vGrid:0.0f);
    int batPowerInt=(strcmp(inv.batModeUi,"IDLE")==0)?0:inv.battWattDisplay;
    float battAmpShown=0.0f;
    if(strcmp(inv.batModeUi,"CHARGE")==0)battAmpShown=inv.battAmpDisplay;
    else if(strcmp(inv.batModeUi,"DISCHARGE")==0)battAmpShown=-inv.battAmpDisplay;
    String recs=buildRecommendations();recs.replace("\n","\\n");
    String bs_type=battTypeStr(inv.ctrl[28]);
    String bs_ah=fUnit(inv.ctrl[29],1,0,"AH");
    String bs_lowv=fUnit(inv.ctrl[26],10,1,"V");
    String bs_highv=fUnit(inv.ctrl[27],10,1,"V");
    String bs_lowret=fUnit(inv.ctrl[30],10,1,"V");
    String bs_maxchg=fUnit(inv.ctrl[13],10,1,"A");
    String bs_maxdis=fUnit(inv.ctrl[12],10,1,"A");
    String bs_gridin=fUnit(inv.ctrl[15],10,1,"A");
    String bs_gridout=fUnit(inv.ctrl[14],10,1,"A");
    String bs_absorb=fUnit(inv.ctrl[25],10,1,"A");
    String bs_combine=fUnit(inv.ctrl[31],10,1,"A");
    String bs_chgtime="--";
    if(inv.ctrl[9]==1&&inv.ctrl[36]!=0xFFFF){char buf[32];snprintf(buf,sizeof(buf),"%02d:%02d - %02d:%02d",(int)inv.ctrl[36],(int)inv.ctrl[37],(int)inv.ctrl[38],(int)inv.ctrl[39]);bs_chgtime=String(buf);}
    String bs_distime="--";
    if(inv.ctrl[40]!=0xFFFF){char buf[32];snprintf(buf,sizeof(buf),"%02d:%02d - %02d:%02d",(int)inv.ctrl[40],(int)inv.ctrl[41],(int)inv.ctrl[42],(int)inv.ctrl[43]);bs_distime=String(buf);}
    String ch_work=(inv.pv[0]==0xFFFF)?"--":workShort(inv.pv[0]);
    String ch_mppt=mpptStr(inv.pv[1]);
    String ch_state=chgStateStr(inv.pv[2]);
    String ch_pvv=fUnit(inv.pv[4],10,1,"V");
    String ch_battv=fUnit(inv.pv[5],10,1,"V");
    String ch_cur=(inv.pv[6]==0xFFFF)?"--":String((int16_t)inv.pv[6]/10.0f,1)+" A";
    String ch_pow=fUnit(inv.pv[7],1,0,"W");
    String ch_rad=fUnit(inv.pv[8],1,0,"C");
    String ch_ext=fUnit(inv.pv[9],1,0,"C");
    String ch_brel=fRelay(inv.pv[12]);
    String ch_pvrel=fRelay(inv.pv[13]);
    String ch_grade=(inv.pv[14]==0xFFFF)?"--":String(inv.pv[14])+" V";
    String ch_rcur=(inv.pv[15]==0xFFFF||inv.pv[15]==0)?"--":String(inv.pv[15])+" A";
    String ch_acc=fAcc(inv.pv[16],inv.pv[17]);
    String ch_err=decodeChgErrors(inv.chgErr);
    String ch_warn=(inv.chgWarn==0xFFFF||inv.chgWarn==0)?"No warnings":"Warning";
    String ia_work=inv.workModeShort;
    String ia_acgrade=fUnit(inv.inv[1],1,0,"V");
    String ia_rated=(inv.inv[76]!=0xFFFF&&inv.inv[76]!=0)?String(inv.inv[76])+" VA":(inv.machinePower+" VA");
    String ia_battv=fUnit(inv.inv[4],10,1,"V");
    String ia_invv=fUnit(inv.inv[5],10,1,"V");
    String ia_gridv=fUnit(inv.inv[6],10,1,"V");
    String ia_busv=fUnit(inv.inv[7],10,1,"V");
    String ia_ictrl=fUnit(inv.inv[8],10,1,"A");
    String ia_iinv=fUnit(inv.inv[9],10,1,"A");
    String ia_igrid=fUnit(inv.inv[10],10,1,"A");
    String ia_iload=fUnit(inv.inv[11],10,1,"A");
    String ia_pinv=fUnit(inv.inv[12],1,0,"W");
    String ia_pgrid=fUnit(inv.inv[13],1,0,"W");
    String ia_pload=fUnit(inv.inv[14],1,0,"W");
    String ia_lpct=fUnit(inv.inv[15],1,0,"%");
    String ia_sinv=fUnit(inv.inv[16],1,0,"VA");
    String ia_sgrid=fUnit(inv.inv[17],1,0,"VA");
    String ia_sload=fUnit(inv.inv[18],1,0,"VA");
    String ib_tac=fUnit(inv.inv[32],1,0,"C");
    String ib_ttr=fUnit(inv.inv[33],1,0,"C");
    String ib_tdc=fUnit(inv.inv[34],1,0,"C");
    String ib_rinv=fRelay(inv.inv[36]);
    String ib_rgrid=fRelay(inv.inv[37]);
    String ib_rload=fRelay(inv.inv[38]);
    String ib_rnline=fRelay(inv.inv[39]);
    String ib_rdc=fRelay(inv.inv[40]);
    String ib_rearth=fRelay(inv.inv[41]);
    String ib_qinv=fUnit(inv.inv[20],1,0,"var");
    String ib_qgrid=fUnit(inv.inv[21],1,0,"var");
    String ib_qload=fUnit(inv.inv[22],1,0,"var");
    String ib_accchg=fAcc(inv.inv[44],inv.inv[45]);
    String ib_accdis=fAcc(inv.inv[46],inv.inv[47]);
    String ib_accbuy=fAcc(inv.inv[48],inv.inv[49]);
    String ib_accsell=fAcc(inv.inv[50],inv.inv[51]);
    String ib_accload=fAcc(inv.inv[52],inv.inv[53]);
    String ib_accself=fAcc(inv.inv[54],inv.inv[55]);
    String ib_accpvsell=fAcc(inv.inv[56],inv.inv[57]);
    String ib_accgridchg=fAcc(inv.inv[58],inv.inv[59]);
    String ib_bpow=r73Sane()?fUnit(inv.inv[72],1,0,"W"):(String((int)roundf(inv.battPowerSigned))+" W");
    String ib_bcur=r74Sane()?fUnit(inv.inv[73],1,0,"A"):(String(battAmpShown,1)+" A");
    String ib_ihz=fUnit(inv.inv[24],100,2,"Hz");
    String ib_ghz=fUnit(inv.inv[25],100,2,"Hz");
    String ib_err=decodeErrors(inv.err1,inv.err2,inv.err3);
    String ib_warn=decodeWarnings(inv.warn1,inv.warn2);

    AsyncResponseStream*res=req->beginResponseStream("application/json");
    res->printf("{\"state\":\"%s\",",inv.stateStr);
    res->printf("\"workMode\":\"%s\",",inv.workModeShort);
    res->printf("\"batteryVolt\":%.2f,",inv.vBatt);
    res->printf("\"batPowerDisplay\":%d,",batPowerInt);
    res->printf("\"battAmp\":%.1f,",battAmpShown);
    res->printf("\"batMode\":\"%s\",",inv.batModeUi);
    res->printf("\"batClass\":\"%s\",",inv.batClassUi);
    res->printf("\"batSuffix\":\"%s\",",inv.batRegsValid?"":" ~");
    res->printf("\"solar\":%d,",inv.effectiveSolarW);
    res->printf("\"pvvolt\":%.1f,",inv.vPV);
    res->printf("\"loadPower\":%d,",inv.pLoad);
    res->printf("\"loadVolt\":%.1f,",loadVoltage);
    res->printf("\"grid\":%.1f,",inv.vGrid);
    res->printf("\"gridPowerShow\":%d,",inv.gridPowerShow);
    res->printf("\"loadPercent\":%d,",inv.loadPercent);
    res->printf("\"tempAC\":%d,",inv.tempAC);
    res->printf("\"invLoss\":%d,",(int)roundf(inv.selfConsumption));
    res->printf("\"pvEnergy\":%.1f,",inv.pvEnergy);
    res->printf("\"accumLoad\":\"%s\",",ib_accload.c_str());
    res->printf("\"accumDischarge\":\"%s\",",ib_accdis.c_str());
    res->printf("\"net\":%.0f,",inv.pNet);
    res->printf("\"sketchVersion\":\"" SKETCH_VERSION "\",");
    res->printf("\"offgridEnable\":\"%s\",",inv.offgridEnable==1?"ON":"OFF");
    switch(inv.energyMode){
      case 1:res->print("\"energyMode\":\"SBU\",");break;
      case 2:res->print("\"energyMode\":\"SUB\",");break;
      case 3:res->print("\"energyMode\":\"UTI\",");break;
      case 4:res->print("\"energyMode\":\"SOL\",");break;
      default:res->printf("\"energyMode\":\"MODE%d\",",inv.energyMode);break;
    }
    switch(inv.chargerPriority){
      case 0:res->print("\"chargerPriority\":\"Solar first\",");break;
      case 2:res->print("\"chargerPriority\":\"Solar+Utility\",");break;
      case 3:res->print("\"chargerPriority\":\"Only Solar\",");break;
      default:res->printf("\"chargerPriority\":\"PRIO%d\",",inv.chargerPriority);break;
    }
    res->printf("\"recs\":\"%s\",",recs.c_str());
    res->printf("\"bs_type\":\"%s\",",bs_type.c_str());
    res->printf("\"bs_ah\":\"%s\",",bs_ah.c_str());
    res->printf("\"bs_lowv\":\"%s\",",bs_lowv.c_str());
    res->printf("\"bs_highv\":\"%s\",",bs_highv.c_str());
    res->printf("\"bs_lowret\":\"%s\",",bs_lowret.c_str());
    res->printf("\"bs_maxchg\":\"%s\",",bs_maxchg.c_str());
    res->printf("\"bs_maxdis\":\"%s\",",bs_maxdis.c_str());
    res->printf("\"bs_gridin\":\"%s\",",bs_gridin.c_str());
    res->printf("\"bs_gridout\":\"%s\",",bs_gridout.c_str());
    res->printf("\"bs_absorb\":\"%s\",",bs_absorb.c_str());
    res->printf("\"bs_combine\":\"%s\",",bs_combine.c_str());
    res->printf("\"bs_chgtime\":\"%s\",",bs_chgtime.c_str());
    res->printf("\"bs_distime\":\"%s\",",bs_distime.c_str());
    res->printf("\"ch_work\":\"%s\",",ch_work.c_str());
    res->printf("\"ch_mppt\":\"%s\",",ch_mppt.c_str());
    res->printf("\"ch_state\":\"%s\",",ch_state.c_str());
    res->printf("\"ch_pvv\":\"%s\",",ch_pvv.c_str());
    res->printf("\"ch_battv\":\"%s\",",ch_battv.c_str());
    res->printf("\"ch_cur\":\"%s\",",ch_cur.c_str());
    res->printf("\"ch_pow\":\"%s\",",ch_pow.c_str());
    res->printf("\"ch_rad\":\"%s\",",ch_rad.c_str());
    res->printf("\"ch_ext\":\"%s\",",ch_ext.c_str());
    res->printf("\"ch_brel\":\"%s\",",ch_brel.c_str());
    res->printf("\"ch_pvrel\":\"%s\",",ch_pvrel.c_str());
    res->printf("\"ch_grade\":\"%s\",",ch_grade.c_str());
    res->printf("\"ch_rcur\":\"%s\",",ch_rcur.c_str());
    res->printf("\"ch_acc\":\"%s\",",ch_acc.c_str());
    res->printf("\"ch_err\":\"%s\",",ch_err.c_str());
    res->printf("\"ch_warn\":\"%s\",",ch_warn.c_str());
    res->printf("\"ia_work\":\"%s\",",ia_work.c_str());
    res->printf("\"ia_acgrade\":\"%s\",",ia_acgrade.c_str());
    res->printf("\"ia_rated\":\"%s\",",ia_rated.c_str());
    res->printf("\"ia_battv\":\"%s\",",ia_battv.c_str());
    res->printf("\"ia_invv\":\"%s\",",ia_invv.c_str());
    res->printf("\"ia_gridv\":\"%s\",",ia_gridv.c_str());
    res->printf("\"ia_busv\":\"%s\",",ia_busv.c_str());
    res->printf("\"ia_ictrl\":\"%s\",",ia_ictrl.c_str());
    res->printf("\"ia_iinv\":\"%s\",",ia_iinv.c_str());
    res->printf("\"ia_igrid\":\"%s\",",ia_igrid.c_str());
    res->printf("\"ia_iload\":\"%s\",",ia_iload.c_str());
    res->printf("\"ia_pinv\":\"%s\",",ia_pinv.c_str());
    res->printf("\"ia_pgrid\":\"%s\",",ia_pgrid.c_str());
    res->printf("\"ia_pload\":\"%s\",",ia_pload.c_str());
    res->printf("\"ia_lpct\":\"%s\",",ia_lpct.c_str());
    res->printf("\"ia_sinv\":\"%s\",",ia_sinv.c_str());
    res->printf("\"ia_sgrid\":\"%s\",",ia_sgrid.c_str());
    res->printf("\"ia_sload\":\"%s\",",ia_sload.c_str());
    res->printf("\"ib_tac\":\"%s\",",ib_tac.c_str());
    res->printf("\"ib_ttr\":\"%s\",",ib_ttr.c_str());
    res->printf("\"ib_tdc\":\"%s\",",ib_tdc.c_str());
    res->printf("\"ib_rinv\":\"%s\",",ib_rinv.c_str());
    res->printf("\"ib_rgrid\":\"%s\",",ib_rgrid.c_str());
    res->printf("\"ib_rload\":\"%s\",",ib_rload.c_str());
    res->printf("\"ib_rnline\":\"%s\",",ib_rnline.c_str());
    res->printf("\"ib_rdc\":\"%s\",",ib_rdc.c_str());
    res->printf("\"ib_rearth\":\"%s\",",ib_rearth.c_str());
    res->printf("\"ib_qinv\":\"%s\",",ib_qinv.c_str());
    res->printf("\"ib_qgrid\":\"%s\",",ib_qgrid.c_str());
    res->printf("\"ib_qload\":\"%s\",",ib_qload.c_str());
    res->printf("\"ib_accchg\":\"%s\",",ib_accchg.c_str());
    res->printf("\"ib_accdis\":\"%s\",",ib_accdis.c_str());
    res->printf("\"ib_accbuy\":\"%s\",",ib_accbuy.c_str());
    res->printf("\"ib_accsell\":\"%s\",",ib_accsell.c_str());
    res->printf("\"ib_accload\":\"%s\",",ib_accload.c_str());
    res->printf("\"ib_accself\":\"%s\",",ib_accself.c_str());
    res->printf("\"ib_accpvsell\":\"%s\",",ib_accpvsell.c_str());
    res->printf("\"ib_accgridchg\":\"%s\",",ib_accgridchg.c_str());
    res->printf("\"ib_bpow\":\"%s\",",ib_bpow.c_str());
    res->printf("\"ib_bcur\":\"%s\",",ib_bcur.c_str());
    res->printf("\"ib_ihz\":\"%s\",",ib_ihz.c_str());
    res->printf("\"ib_ghz\":\"%s\",",ib_ghz.c_str());
    res->printf("\"ib_err\":\"%s\",",ib_err.c_str());
    res->printf("\"ib_warn\":\"%s\"",ib_warn.c_str());
    res->print("}");
    req->send(res);
    if(locked)xSemaphoreGive(dataMutex);
  });
  server.onNotFound([](AsyncWebServerRequest*req){req->redirect("/");});
  server.begin();
}

void setupOTA(){
  ArduinoOTA.setHostname("inverter");
  ArduinoOTA.onStart([](){webUpdateActive=true;Serial.println("Start updating sketch");digitalWrite(LED_PIN,HIGH);});
  ArduinoOTA.onEnd([](){Serial.println("\nEnd");digitalWrite(LED_PIN,LOW);});
  ArduinoOTA.onProgress([](unsigned int progress,unsigned int total){Serial.printf("Progress: %u%%\r",(progress/(total/100)));});
  ArduinoOTA.onError([](ota_error_t error){
    Serial.printf("Error[%u]: ",error);
    if(error==OTA_AUTH_ERROR)Serial.println("Auth Failed");
    else if(error==OTA_BEGIN_ERROR)Serial.println("Begin Failed");
    else if(error==OTA_CONNECT_ERROR)Serial.println("Connect Failed");
    else if(error==OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
    else if(error==OTA_END_ERROR)Serial.println("End Failed");
    webUpdateActive=false;digitalWrite(LED_PIN,LOW);
  });
  ArduinoOTA.begin();Serial.println("OTA Ready");
}

void setup(){
  Serial.begin(115200);
  pinMode(RS485_CTRL,OUTPUT);digitalWrite(RS485_CTRL,LOW);
  pinMode(LED_PIN,OUTPUT);digitalWrite(LED_PIN,HIGH);
  WiFi.persistent(false);
  preferences.begin("wifi_config",true);
  wifiSsid=preferences.getString("ssid","");
  wifiPassword=preferences.getString("password","");
  preferences.end();
  if(wifiSsid.length()>0){
    WiFi.mode(WIFI_STA);WiFi.begin(wifiSsid.c_str(),wifiPassword.c_str());
    unsigned long startWifi=millis();
    while(WiFi.status()!=WL_CONNECTED&&(millis()-startWifi<WIFI_CONNECT_TIMEOUT_MS))delay(300);
  }
  if(WiFi.status()!=WL_CONNECTED){startSetupAP();return;}
  MDNS.begin("inverter");
  Serial1.begin(19200,SERIAL_8N1,RX2_PIN,TX2_PIN);
  node.begin(SLAVE_ID,Serial1);
  node.preTransmission(preTrans);node.postTransmission(postTrans);
  delay(1500);
  uint16_t raw20000=readSingleRegisterWithRetry(20000);
  uint16_t raw20001=readSingleRegisterWithRetry(20001);
  char ch1=(raw20000>>8)&0xFF;char ch2=raw20000&0xFF;
  inv.machineType=String(ch1)+String(ch2);
  inv.machinePower=(raw20001==1800||raw20001==3000)?String(raw20001):"—";
  dataMutex=xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(modbusTask,"modbus_poll",8192,nullptr,5,&modbusTaskHandle,0);
  setupWebServer();setupOTA();
}

void loop(){
  unsigned long currentMs=millis();
  if(needRestart&&(currentMs-restartRequestedMs>=RESTART_DELAY_MS))ESP.restart();
  if(setupMode){dnsServer.processNextRequest();return;}
  if(WiFi.status()!=WL_CONNECTED){
    if(currentMs-lastWifiReconnectMs>=WIFI_RECONNECT_INTERVAL_MS){lastWifiReconnectMs=currentMs;WiFi.reconnect();}
  }
  ArduinoOTA.handle();
}
