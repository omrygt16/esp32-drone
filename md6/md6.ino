/*
 * ESP32 Web Kontrollü Mini Drone (X tipi quad, coreless motor + MOSFET)
 *
 * Bağlantılar:
 *   MPU6050 : SDA=21, SCL=22
 *   Motorlar: SOL ÖN=26, SAĞ ÖN=25, SOL ARKA=27, SAĞ ARKA=14   (MOSFET gate'lerine)
 *
 * Pervane yönleri (yukarıdan bakış):  FL=CW  FR=CCW  RL=CCW  RR=CW
 * MPU6050 yönü: çip yukarı bakacak, kart üzerindeki X oku drone'un İLERİSİNE baksın.
 *
 * WiFi: "MiniDrone" / "drone1234"  ->  http://192.168.4.1
 *
 * Arduino-ESP32 core 2.x ve 3.x ile derlenir. Ek kütüphane gerekmez.
 *
 * !!! İlk testleri MUTLAKA pervanesiz yap. !!!
 */






  /*bağlantılar 
  *mpu6050
  *scl 22
  *sda 21
  *vcc 3.3v
  *gnd gnd
  */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <math.h>

// ======================= AYARLAR =======================
#define PIN_SDA 21
#define PIN_SCL 22
enum { M_FL = 0, M_FR, M_RL, M_RR };
const uint8_t MOTOR_PIN[4] = {26, 25, 27, 14};
const float   MOTOR_TRIM[4] = {1.0f, 1.0f, 1.0f, 1.0f};   // motor güç eşitleme (0.9-1.0)

#define WIFI_SSID "MiniDrone"
#define WIFI_PASS "drone1234"          // en az 8 karakter

#define PWM_FREQ 20000                 // 20 kHz: sessiz ve düzgün akım
#define PWM_BITS 10
#define LOOP_HZ  500

// IMU filtre: 2=94Hz  3=42Hz (titreşim için varsayılan)  4=21Hz (çok titreşimli)
#define IMU_DLPF 3
const float GYRO_LPF_HZ = 80.0f;       // yazılımsal ek gyro filtresi
const float ACC_LPF_HZ  = 10.0f;       // accel çok agresif filtrelenir (titreşim)
const float D_LPF_HZ    = 40.0f;       // D terimi filtresi
const float ACC_K       = 0.004f;      // complementary filtre accel ağırlığı (~0.5 sn)

// Pilot limitleri
const float MAX_ANGLE    = 25.0f;      // derece
const float MAX_YAW_RATE = 150.0f;     // derece/sn
const float MAX_RATE     = 250.0f;     // dış çevrim çıkış limiti

// Gaz / motor
const float THR_CUTOFF = 0.04f;        // bunun altında motorlar tamamen durur
const float MIN_SPIN   = 0.05f;        // uçuşta motorun altına inmeyeceği duty
const float MAX_OUT    = 0.95f;        // gaz tavanı (miksaj için pay bırakır)
const float THR_UP_RATE   = 2.5f;      // gaz artış hız limiti (1/sn)
const float THR_DOWN_RATE = 8.0f;

// PID (başlangıç değerleri, 7 cm / ~25-30 g sınıfı için). Birim: duty / (deg/s)
const float KP_ANGLE = 5.0f;           // açı -> hız (deg/s per deg)
const float RP_KP = 0.0020f, RP_KI = 0.0035f, RP_KD = 0.00003f;
const float Y_KP  = 0.0015f, Y_KI  = 0.0004f;
#define YAW_SIGN 1.0f                  // yaw ters dönerse -1 yap

// Güvenlik
const uint32_t LINK_TIMEOUT_MS = 300;   // bu kadar paket gelmezse FAILSAFE
const uint32_t FS_MAX_MS       = 2500;  // failsafe'te en geç bu sürede motorlar kapanır
const float    FS_DECAY        = 0.35f; // failsafe gaz azalma hızı (1/sn)
const float    TILT_CUT_DEG    = 65.0f; // bu açıyı geçerse motorlar kesilir
const uint32_t IDLE_DISARM_MS  = 8000;  // armed + gaz sıfır ise otomatik disarm
const int      IMU_FAIL_LIMIT  = 5;     // art arda hata -> disarm
const int      CAL_SAMPLES     = 500;   // 1 sn gyro kalibrasyonu
const float    CAL_MOTION_DPS  = 6.0f;  // kalibrasyon sırasında hareket toleransı

// ======================= GLOBAL =======================
enum State { ST_CALIB = 0, ST_DISARMED, ST_ARMED, ST_FAILSAFE };

WebServer server(80);

volatile State state = ST_CALIB;
const char* volatile reason = "BOOT";

volatile float inThr = 0, inYaw = 0, inPitch = 0, inRoll = 0;
volatile uint32_t lastPktMs = 0;
uint32_t lastSeq = 0;
volatile bool armReq = false, disarmReq = false, calReq = false;

volatile float telRoll = 0, telPitch = 0;
volatile float telM[4] = {0, 0, 0, 0};

const int MPU_ADDR = 0x68;
bool imuOK = false;
float gyroBias[3] = {0, 0, 0};

// ======================= PWM =======================
static inline void pwmWrite(int i, uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PIN[i], d);
#else
  ledcWrite(i, d);
#endif
}

void pwmInit() {
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PIN[i], PWM_FREQ, PWM_BITS);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(MOTOR_PIN[i], i);
#endif
    pwmWrite(i, 0);
  }
}

void motorsOff() {
  for (int i = 0; i < 4; i++) { pwmWrite(i, 0); telM[i] = 0; }
}

// ======================= MPU6050 =======================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool imuInit() {
  if (!mpuWrite(0x6B, 0x80)) return false;   // reset
  delay(100);
  if (!mpuWrite(0x6B, 0x01)) return false;   // uyandır, PLL (gyro X) saat
  delay(10);
  bool ok = true;
  ok &= mpuWrite(0x19, 1);                   // 1kHz/(1+1) = 500 Hz
  ok &= mpuWrite(0x1A, IMU_DLPF);            // donanımsal düşük geçiren filtre
  ok &= mpuWrite(0x1B, 0x08);                // gyro  +-500 dps  (65.5 LSB/dps)
  ok &= mpuWrite(0x1C, 0x08);                // accel +-4 g      (8192 LSB/g)
  return ok;
}

// a[] -> g, g[] -> dps (sensör ekseni)
bool imuRead(float* a, float* g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  // hepsi 0 ise sensör takılmış demektir
  if (r[0] == 0 && r[1] == 0 && r[2] == 0 && r[4] == 0 && r[5] == 0 && r[6] == 0) return false;
  a[0] = r[0] / 8192.0f; a[1] = r[1] / 8192.0f; a[2] = r[2] / 8192.0f;
  g[0] = r[4] / 65.5f;   g[1] = r[5] / 65.5f;   g[2] = r[6] / 65.5f;
  return true;
}

// ======================= PID =======================
struct PID {
  float kp, ki, kd, iLim, oLim, i, prev, d;
  PID(float a, float b, float c, float l, float o)
    : kp(a), ki(b), kd(c), iLim(l), oLim(o), i(0), prev(0), d(0) {}
  void reset() { i = 0; d = 0; }
  float run(float sp, float meas, float dt, bool en, float aD) {
    if (!en) { i = 0; d = 0; prev = meas; return 0; }
    float e = sp - meas;
    i += ki * e * dt;
    i = constrain(i, -iLim, iLim);
    float dd = -(meas - prev) / dt;      // ölçüm üzerinden türev (setpoint kick yok)
    prev = meas;
    d += aD * (dd - d);
    return constrain(kp * e + i + kd * d, -oLim, oLim);
  }
};

PID pidR(RP_KP, RP_KI, RP_KD, 0.10f, 0.30f);
PID pidP(RP_KP, RP_KI, RP_KD, 0.10f, 0.30f);
PID pidY(Y_KP,  Y_KI,  0.0f,  0.08f, 0.15f);

static inline float pt1a(float fc, float dt) {
  float rc = 1.0f / (2.0f * PI * fc);
  return dt / (dt + rc);
}

// ======================= KONTROL GÖREVİ =======================
void setDisarmed(const char* why) {
  state = ST_DISARMED;
  reason = why;
  pidR.reset(); pidP.reset(); pidY.reset();
  motorsOff();
}

void controlTask(void*) {
  esp_task_wdt_add(NULL);                          // bu görev kilitlenirse reset -> motorlar durur
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastUs = micros();

  float roll = 0, pitch = 0;
  float af[3] = {0, 0, 1};
  float gfr = 0, gfp = 0, gfy = 0;
  bool afInit = false;

  int imuFail = 0;
  uint32_t lastReinit = 0;

  // kalibrasyon
  int calCount = 0;
  float calSum[3] = {0, 0, 0}, calMin[3], calMax[3];
  auto calRestart = [&]() {
    calCount = 0;
    for (int i = 0; i < 3; i++) { calSum[i] = 0; calMin[i] = 1e9f; calMax[i] = -1e9f; }
  };
  if (state == ST_CALIB) calRestart();

  float thrCur = 0;
  uint32_t fsStart = 0, idleSince = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / LOOP_HZ));
    esp_task_wdt_reset();

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    dt = constrain(dt, 0.0005f, 0.01f);
    uint32_t nowMs = millis();

    // ---------- 1) IMU oku ----------
    float a[3], g[3];
    if (imuOK && imuRead(a, g)) {
      imuFail = 0;
    } else {
      if (imuOK && ++imuFail < IMU_FAIL_LIMIT) continue;   // tek tük hatayı tolere et
      imuOK = false;
      if (state != ST_DISARMED) setDisarmed("IMU_FAIL"); else motorsOff();
      if (nowMs - lastReinit > 500) {                      // toparlanmayı dene
        lastReinit = nowMs;
        Wire.end(); Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000); Wire.setTimeOut(5);
        if (imuInit()) { imuOK = true; imuFail = 0; state = ST_CALIB; reason = "CALIB"; calRestart(); afInit = false; }
      }
      continue;
    }

    // ---------- 2) Filtreler ----------
    float aA = pt1a(ACC_LPF_HZ, dt);
    if (!afInit) { af[0] = a[0]; af[1] = a[1]; af[2] = a[2]; afInit = true; }
    for (int i = 0; i < 3; i++) af[i] += aA * (a[i] - af[i]);

    // gövde eksenine çevir (X ileri, Y sol, Z yukarı):
    //  roll(+ sağ yatış)=gx   pitch(+ burun yukarı)=-gy   yaw(+ saat yönü)=-gz
    float gr = g[0] - gyroBias[0];
    float gp = -(g[1] - gyroBias[1]);
    float gy = -(g[2] - gyroBias[2]);
    float aG = pt1a(GYRO_LPF_HZ, dt);
    gfr += aG * (gr - gfr);
    gfp += aG * (gp - gfp);
    gfy += aG * (gy - gfy);

    // ---------- 3) Açı kestirimi (complementary) ----------
    float accRoll  = atan2f(af[1], af[2]) * RAD_TO_DEG;
    float accPitch = atan2f(af[0], sqrtf(af[1] * af[1] + af[2] * af[2])) * RAD_TO_DEG;

    if (state == ST_CALIB) {
      roll = accRoll; pitch = accPitch;
      motorsOff();
      for (int i = 0; i < 3; i++) {
        calSum[i] += g[i];
        if (g[i] < calMin[i]) calMin[i] = g[i];
        if (g[i] > calMax[i]) calMax[i] = g[i];
      }
      bool moved = false;
      for (int i = 0; i < 3; i++) if (calMax[i] - calMin[i] > CAL_MOTION_DPS) moved = true;
      if (moved) { calRestart(); reason = "CALIB_HOLD_STILL"; }
      else if (++calCount >= CAL_SAMPLES) {
        for (int i = 0; i < 3; i++) gyroBias[i] = calSum[i] / calCount;
        pidR.reset(); pidP.reset(); pidY.reset();
        state = ST_DISARMED; reason = "READY";
      }
      telRoll = roll; telPitch = pitch;
      continue;
    }

    roll  += gr * dt;
    pitch += gp * dt;
    float n = sqrtf(af[0] * af[0] + af[1] * af[1] + af[2] * af[2]);
    if (n > 0.8f && n < 1.2f) {              // ivmelenme/titreşim varsa accel'e güvenme
      roll  += ACC_K * (accRoll  - roll);
      pitch += ACC_K * (accPitch - pitch);
    }
    telRoll = roll; telPitch = pitch;

    // ---------- 4) Komutlar ve güvenlik ----------
    float thrIn   = inThr;
    float rollIn  = inRoll;
    float pitchIn = inPitch;
    float yawIn   = inYaw;
    uint32_t lp   = lastPktMs;
    bool linkFresh = (lp != 0) && ((nowMs - lp) < LINK_TIMEOUT_MS);

    if (disarmReq) {
      disarmReq = false;
      if (state == ST_ARMED || state == ST_FAILSAFE) setDisarmed("USER_DISARM");
    }
    if (calReq) {
      calReq = false;
      if (state == ST_DISARMED) { state = ST_CALIB; reason = "CALIB"; calRestart(); continue; }
    }
    if (armReq) {
      armReq = false;
      if (state == ST_DISARMED) {
        const char* deny = nullptr;
        if (!linkFresh) deny = "ARM_NO_LINK";
        else if (thrIn > 0.03f) deny = "ARM_THR_HIGH";
        else if (fabsf(roll) > 12 || fabsf(pitch) > 12) deny = "ARM_NOT_LEVEL";
        else if (fabsf(gfr) > 20 || fabsf(gfp) > 20 || fabsf(gfy) > 20) deny = "ARM_MOVING";
        if (deny) reason = deny;
        else {
          pidR.reset(); pidP.reset(); pidY.reset();
          thrCur = 0; idleSince = 0;
          state = ST_ARMED; reason = "ARMED";
        }
      }
    }

    if (state == ST_ARMED || state == ST_FAILSAFE) {
      if (fabsf(roll) > TILT_CUT_DEG || fabsf(pitch) > TILT_CUT_DEG) setDisarmed("TILT_CUT");
    }
    if (state == ST_ARMED && !linkFresh) {
      state = ST_FAILSAFE; fsStart = nowMs; reason = "LINK_LOST";
    }
    if (state == ST_FAILSAFE && ((nowMs - fsStart) > FS_MAX_MS || thrCur <= THR_CUTOFF)) {
      setDisarmed("FAILSAFE_LAND");
    }

    if (state != ST_ARMED && state != ST_FAILSAFE) {
      thrCur = 0;
      motorsOff();
      continue;
    }

    // ---------- 5) Hedefler ----------
    float rollT, pitchT, yawRateT, thrTarget, downRate;
    if (state == ST_ARMED) {
      thrTarget = thrIn;
      rollT     =  rollIn  * MAX_ANGLE;
      pitchT    = -pitchIn * MAX_ANGLE;     // +pitchIn = ileri = burun aşağı
      yawRateT  =  yawIn   * MAX_YAW_RATE;
      downRate  = THR_DOWN_RATE;
    } else {                                 // FAILSAFE: düz tut, yavaşça alçal
      thrTarget = 0; rollT = 0; pitchT = 0; yawRateT = 0;
      downRate  = FS_DECAY;
    }
    if (thrTarget > thrCur) thrCur = min(thrTarget, thrCur + THR_UP_RATE * dt);
    else                    thrCur = max(thrTarget, thrCur - downRate * dt);

    // gaz sıfırda uzun süre kalırsa otomatik disarm
    if (state == ST_ARMED) {
      if (thrCur <= THR_CUTOFF) {
        if (idleSince == 0) idleSince = nowMs;
        else if (nowMs - idleSince > IDLE_DISARM_MS) { setDisarmed("IDLE_DISARM"); continue; }
      } else idleSince = 0;
    }

    // ---------- 6) PID (kademeli: açı -> hız) ----------
    bool flying = thrCur > THR_CUTOFF;
    float aD = pt1a(D_LPF_HZ, dt);
    float rateSPr = constrain(KP_ANGLE * (rollT  - roll),  -MAX_RATE, MAX_RATE);
    float rateSPp = constrain(KP_ANGLE * (pitchT - pitch), -MAX_RATE, MAX_RATE);
    float outR = pidR.run(rateSPr,  gfr, dt, flying, aD);
    float outP = pidP.run(rateSPp,  gfp, dt, flying, aD);
    float outY = pidY.run(yawRateT, gfy, dt, flying, aD) * YAW_SIGN;

    if (!flying) { motorsOff(); continue; }

    // ---------- 7) Miksaj ----------
    float thrNorm = (thrCur - THR_CUTOFF) / (1.0f - THR_CUTOFF);
    float base = MIN_SPIN + thrNorm * (MAX_OUT - MIN_SPIN);
    float m[4];
    m[M_FL] = base + outP + outR - outY;
    m[M_FR] = base + outP - outR + outY;
    m[M_RL] = base - outP + outR + outY;
    m[M_RR] = base - outP - outR - outY;

    float hi = max(max(m[0], m[1]), max(m[2], m[3]));
    if (hi > 1.0f) { float sh = hi - 1.0f; for (int i = 0; i < 4; i++) m[i] -= sh; }

    for (int i = 0; i < 4; i++) {
      m[i] = constrain(m[i], MIN_SPIN, 1.0f) * MOTOR_TRIM[i];
      m[i] = constrain(m[i], 0.0f, 1.0f);
      pwmWrite(i, (uint32_t)(m[i] * ((1 << PWM_BITS) - 1)));
      telM[i] = m[i];
    }
  }
}

// ======================= WEB =======================
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>MiniDrone</title>
<style>
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif;height:100vh;display:flex;flex-direction:column;overflow:hidden}
#top{padding:8px;display:flex;gap:8px;align-items:center}
button{font-size:16px;padding:14px 18px;border:0;border-radius:10px;color:#fff}
#arm{background:#2a7}#dis{background:#555}#cal{background:#46a}#kill{background:#c22;font-weight:700;flex:1}
#st{font:13px/1.5 monospace;padding:0 10px;min-height:60px}
#pads{flex:1;display:flex;gap:16px;padding:12px}
.pad{flex:1;position:relative;background:#1c1c1c;border:2px solid #333;border-radius:16px;touch-action:none}
.k{position:absolute;width:64px;height:64px;margin:-32px;border-radius:50%;background:#4af;opacity:.85;left:50%;top:50%;pointer-events:none}
</style></head><body>
<div id=top><button id=arm>ARM</button><button id=dis>DISARM</button><button id=cal>CAL</button><button id=kill>KILL</button></div>
<div id=st>...</div>
<div id=pads><div class=pad id=L><div class=k></div></div><div class=pad id=R><div class=k></div></div></div>
<script>
const $=id=>document.getElementById(id),cl=(v,a,b)=>Math.max(a,Math.min(b,v));
const S={t:0,y:0,p:0,r:0};let seq=0,busy=false;
function setK(k,x,y){k.style.left=(50+x*50)+'%';k.style.top=(50-y*50)+'%'}
function pad(el,down,move,up){const k=el.querySelector('.k');let id=null;
 const pos=e=>{const b=el.getBoundingClientRect();return[cl((e.clientX-b.left)/b.width*2-1,-1,1),cl(1-(e.clientY-b.top)/b.height*2,-1,1)]};
 el.onpointerdown=e=>{if(id!==null)return;id=e.pointerId;el.setPointerCapture(id);const[x,y]=pos(e);down(x,y,k)};
 el.onpointermove=e=>{if(e.pointerId!==id)return;const[x,y]=pos(e);move(x,y,k)};
 const end=e=>{if(e.pointerId!==id)return;id=null;up(k)};
 el.onpointerup=end;el.onpointercancel=end;return k}
let y0=0,t0=0;
// SOL: gaz (göreceli, yerinde kalır) + yaw (bırakınca ortaya döner)
const kL=pad($('L'),(x,y,k)=>{y0=y;t0=S.t;S.y=x;setK(k,x,S.t*2-1)},
 (x,y,k)=>{S.y=x;S.t=cl(t0+(y-y0)/2,0,1);setK(k,x,S.t*2-1)},k=>{S.y=0;setK(k,0,S.t*2-1)});
// SAĞ: pitch (yukarı=ileri) + roll
const kR=pad($('R'),(x,y,k)=>{S.r=x;S.p=y;setK(k,x,y)},(x,y,k)=>{S.r=x;S.p=y;setK(k,x,y)},k=>{S.r=S.p=0;setK(k,0,0)});
setK(kL,0,-1);setK(kR,0,0);
const ST=['CALIB','DISARMED','ARMED','FAILSAFE'];
function show(j){$('st').innerHTML=ST[j.st]+' | '+j.rs+'<br>roll '+j.r+'  pitch '+j.p+'  | link '+j.la+' ms<br>thr '+(S.t*100|0)+'%  M '+j.m.join(' ')}
async function tick(){if(busy)return;busy=true;const c=new AbortController(),to=setTimeout(()=>c.abort(),250);
 try{const r=await fetch(`/c?t=${S.t.toFixed(3)}&y=${S.y.toFixed(3)}&p=${S.p.toFixed(3)}&r=${S.r.toFixed(3)}&s=${++seq}`,{signal:c.signal,cache:'no-store'});show(await r.json())}
 catch(e){$('st').textContent='BAĞLANTI YOK'}finally{clearTimeout(to);busy=false}}
setInterval(tick,40);
const cmd=u=>fetch(u,{cache:'no-store',keepalive:true}).catch(()=>{});
const zero=()=>{S.t=0;setK(kL,0,-1)};
$('arm').onclick=()=>{if(S.t>0.03){alert('Önce gazı sıfırla');return}cmd('/a')};
$('dis').onclick=()=>{zero();cmd('/d')};
$('cal').onclick=()=>cmd('/cal');
$('kill').onclick=()=>{zero();cmd('/k');cmd('/k')};
addEventListener('keydown',e=>{if(e.code==='Space'||e.key==='Escape')$('kill').click()});
document.addEventListener('visibilitychange',()=>{if(document.hidden){zero();cmd('/k')}});
addEventListener('pagehide',()=>cmd('/k'));
</script></body></html>
)rawliteral";

void sendTel() {
  char b[220];
  uint32_t lp = lastPktMs;
  uint32_t la = lp ? (millis() - lp) : 9999;
  snprintf(b, sizeof(b),
           "{\"st\":%d,\"rs\":\"%s\",\"r\":%.1f,\"p\":%.1f,\"la\":%u,\"m\":[%d,%d,%d,%d]}",
           (int)state, (const char*)reason, (float)telRoll, (float)telPitch, (unsigned)la,
           (int)(telM[0] * 100), (int)(telM[1] * 100), (int)(telM[2] * 100), (int)(telM[3] * 100));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

static float argF(const char* name, float lo, float hi) {
  float v = server.arg(name).toFloat();
  if (isnan(v)) v = 0;
  return constrain(v, lo, hi);
}

void hCtl() {
  uint32_t s = strtoul(server.arg("s").c_str(), nullptr, 10);
  int32_t d = (int32_t)(s - lastSeq);
  if (d <= 0 && d > -50) { sendTel(); return; }    // eski/sırasız paket
  lastSeq = s;
  inThr   = argF("t", 0.0f, 1.0f);
  inYaw   = argF("y", -1.0f, 1.0f);
  inPitch = argF("p", -1.0f, 1.0f);
  inRoll  = argF("r", -1.0f, 1.0f);
  lastPktMs = millis();
  sendTel();
}

void setup() {
  // Önce motor pinleri LOW
  for (int i = 0; i < 4; i++) { pinMode(MOTOR_PIN[i], OUTPUT); digitalWrite(MOTOR_PIN[i], LOW); }
  Serial.begin(115200);
  pwmInit();
  motorsOff();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  imuOK = imuInit();
  if (imuOK) { state = ST_CALIB; reason = "CALIB"; }
  else       { state = ST_DISARMED; reason = "IMU_FAIL"; }
  Serial.println(imuOK ? "IMU OK" : "IMU FAIL");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 2);      // kanal 1, gizli değil, max 2 istemci
  WiFi.setSleep(false);

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/c", hCtl);
  server.on("/t", sendTel);
  server.on("/a", []() { armReq = true;    sendTel(); });
  server.on("/d", []() { disarmReq = true; sendTel(); });
  server.on("/k", []() { disarmReq = true; inThr = 0; sendTel(); });
  server.on("/cal", []() { calReq = true;  sendTel(); });
  server.begin();

  // Watchdog: kontrol görevi donarsa reset -> motorlar durur
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wc;
  wc.timeout_ms = 500; wc.idle_core_mask = 0; wc.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wc) != ESP_OK) esp_task_wdt_init(&wc);
#else
  esp_task_wdt_init(1, true);
#endif

  xTaskCreatePinnedToCore(controlTask, "ctl", 6144, NULL, 5, NULL, 1);
}

void loop() {
  server.handleClient();
  delay(2);
}
