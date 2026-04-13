/*
 * ARES-X — Complete ESP32 Firmware
 * Sensors: MLX90614 (thermal) + MPU6500 (IMU) + BMP280 (pressure/alt) + MQ135 (air quality)
 * Motors: L298N with safety gate + sonar
 * LEDs: Front + Back NeoPixel strips
 * Comms: WiFi AP web control + Pi UART bridge
 *
 * Pin map:
 *   GPIO 5   — Front LED strip
 *   GPIO 12  — Sonar ECHO
 *   GPIO 13  — Sonar TRIG
 *   GPIO 14  — L298N IN1
 *   GPIO 16  — Serial2 RX (Pi UART)
 *   GPIO 17  — Serial2 TX (Pi UART)
 *   GPIO 18  — Back LED strip
 *   GPIO 21  — I2C SDA (MLX90614 + MPU6500 + BMP280)
 *   GPIO 22  — I2C SCL
 *   GPIO 25  — L298N IN4
 *   GPIO 26  — L298N IN3
 *   GPIO 27  — L298N IN2
 *   GPIO 32  — ENB PWM
 *   GPIO 33  — ENA PWM
 *   GPIO 34  — MQ135 AOUT
 *   GPIO 35  — MQ135 DOUT
 *   GY-91:   SDO→GND, NCS→3.3V, CSB→3.3V
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_BMP280.h>

// ================= LED =================
#define FRONT_LED_PIN 5
#define BACK_LED_PIN  18
#define NUM_FRONT 8
#define NUM_BACK  8
Adafruit_NeoPixel frontStrip(NUM_FRONT, FRONT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel backStrip (NUM_BACK,  BACK_LED_PIN,  NEO_GRB + NEO_KHZ800);
unsigned long lastBlink = 0;
bool blinkState = false;

// ================= WiFi =================
const char* apSsid     = "ARES-X";
const char* apPassword = "12345678";
WebServer server(80);

// ================= L298N =================
#define IN1     14
#define IN2     27
#define IN3     26
#define IN4     25
#define ENA_PIN 33
#define ENB_PIN 32
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
uint8_t manualSpeed = 230;
uint8_t autoSpeed   = 200;

// ================= Ultrasonic =================
#define TRIG_PIN 13
#define ECHO_PIN 12
bool  safetyEnabled   = true;
const float SAFETY_DIST_CM = 25.0;
const int   CONFIRM_COUNT  = 3;
int   obstacleCounter = 0;
bool  safetyTriggered = false;
float lastDistance    = -1;

// ================= Pi UART =================
unsigned long lastPiCmd = 0;
const unsigned long PI_TIMEOUT = 2000;
bool piConnected = false;

// ================= Command flags =================
bool forwardCmd = false;
bool backCmd    = false;
bool leftCmd    = false;
bool rightCmd   = false;
bool brakeCmd   = false;

// ================= Modes =================
bool obstacleMode = false;
bool followMode   = false;

// ================= Auto Obstacle =================
enum AutoState { AUTO_IDLE, AUTO_FORWARD, AUTO_BACKWARD, AUTO_TURN };
AutoState autoState = AUTO_IDLE;
unsigned long autoStateStart = 0;
bool turnLeftNext = true;
const unsigned long BACK_TIME       = 600;
const unsigned long TURN_TIME       = 700;
const float         OBSTACLE_DIST_CM = 20.0;

// ================= Sensors =================
unsigned long lastMPU = 0;
// MLX90614
Adafruit_MLX90614 mlx;
bool mlxOk = false;
float mlx_obj = -999, mlx_amb = -999;
unsigned long lastMLX = 0;

// MPU6500 (via Adafruit MPU6050 library)
// MPU6500 direct register access
#define MPU_ADDR     0x68
#define MPU_PWR_MGMT 0x6B
#define MPU_ACCEL_CFG 0x1C
#define MPU_GYRO_CFG  0x1B
#define MPU_ACCEL_OUT 0x3B
#define MPU_GYRO_OUT  0x43
#define MPU_TEMP_OUT  0x41
// Scale factors
#define ACCEL_SCALE  8192.0  // ±4G → 8192 LSB/g → m/s² multiply by 9.80665/8192
#define GYRO_SCALE   65.5    // ±500°/s

bool mpuOk = false;
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyr_x = 0, gyr_y = 0, gyr_z = 0;
float imu_temp = 0;

// BMP280
Adafruit_BMP280 bmp;
bool bmpOk = false;
float bmp_temp = 0, bmp_pres = 0, bmp_alt = 0;
unsigned long lastBMP = 0;

// MQ135
#define MQ135_AOUT 34
#define MQ135_DOUT 35
int  mq_raw   = 0;
bool mq_alert = false;
unsigned long lastMQ = 0;

// Sensor broadcast to Pi
unsigned long lastSensorTx = 0;
const unsigned long SENSOR_TX_INTERVAL = 2000;

// ================= HTML =================
const char index_html[] PROGMEM = R"=====(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ARES-X Control</title>
<style>
  body{background:#0d0d0d;color:#e0e0e0;font-family:Arial;text-align:center;padding:10px}
  h2{color:#00e5ff;margin-bottom:4px}
  .sub{color:#555;font-size:12px;margin-bottom:16px}
  .btn{padding:14px 22px;margin:5px;border:none;border-radius:8px;background:#1a1a1a;
       color:#fff;font-size:15px;cursor:pointer;border:1px solid #2a2a2a}
  .btn:active{background:#333}
  .btn-red{border-color:#8b0000;color:#ff6666}
  .btn-green{border-color:#1a5c1a;color:#66ff66}
  .section{margin-top:18px;border-top:1px solid #222;padding-top:14px}
  .card{background:#111;border:1px solid #1e1e1e;border-radius:8px;
        padding:10px;margin:8px auto;max-width:320px}
  .card-title{font-size:10px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
  .val{font-size:22px;font-weight:bold;color:#00e5ff;font-family:monospace}
  .val-warn{color:#ff9800}
  .val-danger{color:#f44336}
  .val-good{color:#00e676}
  .row{display:flex;justify-content:space-around;flex-wrap:wrap}
  .mini{font-size:13px;color:#aaa;margin:2px 0}
  .badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:11px;margin:2px}
  .badge-good{background:#00e67622;color:#00e676;border:1px solid #00e67644}
  .badge-warn{background:#ff980022;color:#ff9800;border:1px solid #ff980044}
  .badge-bad{background:#f4433622;color:#f44336;border:1px solid #f4433644}
</style>
</head>
<body>
<h2>ARES&#8209;X</h2>
<div class="sub">Rover Command Center</div>

<button class="btn" onmousedown="cmd('f',1)" onmouseup="cmd('f',0)"
        ontouchstart="cmd('f',1)" ontouchend="cmd('f',0)">&#9650; Forward</button><br>
<button class="btn" onmousedown="cmd('l',1)" onmouseup="cmd('l',0)"
        ontouchstart="cmd('l',1)" ontouchend="cmd('l',0)">&#9664; Left</button>
<button class="btn btn-red" onclick="cmd('s',1)">&#9632; Stop</button>
<button class="btn" onmousedown="cmd('r',1)" onmouseup="cmd('r',0)"
        ontouchstart="cmd('r',1)" ontouchend="cmd('r',0)">Right &#9654;</button><br>
<button class="btn" onmousedown="cmd('b',1)" onmouseup="cmd('b',0)"
        ontouchstart="cmd('b',1)" ontouchend="cmd('b',0)">&#9660; Backward</button>

<!-- Thermal -->
<div class="section">
  <div class="card">
    <div class="card-title">&#128293; Thermal (MLX90614)</div>
    <div class="row">
      <div><div class="val" id="objT">--</div><div class="mini">Object °C</div></div>
      <div><div class="val" id="ambT">--</div><div class="mini">Ambient °C</div></div>
    </div>
  </div>
</div>

<!-- IMU -->
<div class="section">
  <div class="card">
    <div class="card-title">&#9654; IMU (MPU6500)</div>
    <div class="mini">Accel: <span id="ax">-</span> <span id="ay">-</span> <span id="az">-</span> m/s²</div>
    <div class="mini">Gyro:  <span id="gx">-</span> <span id="gy">-</span> <span id="gz">-</span> °/s</div>
    <div class="mini">Chip temp: <span id="it">-</span>°C</div>
  </div>
</div>

<!-- BMP280 -->
<div class="section">
  <div class="card">
    <div class="card-title">&#127760; Pressure (BMP280)</div>
    <div class="row">
      <div><div class="val" id="bpres">--</div><div class="mini">hPa</div></div>
      <div><div class="val" id="balt">--</div><div class="mini">Alt m</div></div>
    </div>
  </div>
</div>

<!-- Air Quality -->
<div class="section">
  <div class="card">
    <div class="card-title">&#127811; Air Quality (MQ135)</div>
    <div class="val" id="aqval">--</div>
    <div id="aqbadge"></div>
    <div class="mini" id="aqraw"></div>
  </div>
</div>

<!-- Safety -->
<div class="section">
  <div class="card">
    <div class="card-title">&#128737; Safety Gate</div>
    <div class="mini" id="safetyStatus">Loading...</div>
    <div class="mini" id="distDisplay">Distance: --</div>
    <br>
    <button class="btn btn-green" onclick="setSafety(1)">Enable</button>
    <button class="btn btn-red"   onclick="setSafety(0)">Disable</button>
  </div>
</div>

<!-- Speed -->
<div class="section">
  <div class="card">
    <div class="card-title">Speed</div>
    <input type="range" min="100" max="255" value="230" id="spd"
           oninput="setSpeed(this.value)" style="width:200px">
    <div class="mini" id="spdLabel">230</div>
  </div>
</div>

<!-- Modes -->
<div class="section">
  <b>Obstacle Avoid</b><br>
  <button class="btn btn-green" onclick="mode(1)">ON</button>
  <button class="btn btn-red"   onclick="mode(0)">OFF</button>
</div>
<div class="section">
  <b>Follow Mode</b><br>
  <button class="btn btn-green" onclick="follow(1)">ON</button>
  <button class="btn btn-red"   onclick="follow(0)">OFF</button>
</div>

<script>
function cmd(d,s)     { fetch(`/cmd?dir=${d}&state=${s}`); }
function mode(v)      { fetch(`/mode?auto=${v}`); }
function follow(v)    { fetch(`/follow?f=${v}`); }
function setSafety(v) { fetch(`/safety?en=${v}`).then(pollStatus); }
function setSpeed(v)  { document.getElementById('spdLabel').textContent=v; fetch(`/speed?v=${v}`); }

function setVal(id, v, decimals=1){ const e=document.getElementById(id); if(e) e.textContent=typeof v==='number'?v.toFixed(decimals):v; }

function pollStatus(){
  fetch('/status').then(r=>r.json()).then(d=>{
    // Safety
    document.getElementById('safetyStatus').textContent =
      d.safety ? 'Safety ON — blocks forward < '+d.dist_threshold+'cm' : 'Safety OFF';
    document.getElementById('distDisplay').textContent =
      d.distance > 0 ? 'Distance: '+d.distance.toFixed(1)+' cm' : 'Distance: no echo';

    // Thermal
    if(d.mlx_ok){
      const col = d.obj_temp > 37 ? '#f44336' : d.obj_temp > 33 ? '#ff9800' : '#00e5ff';
      const el = document.getElementById('objT');
      if(el){ el.textContent = d.obj_temp.toFixed(1); el.style.color = col; }
      setVal('ambT', d.amb_temp);
    }

    // IMU
    setVal('ax', d.acc_x, 2); setVal('ay', d.acc_y, 2); setVal('az', d.acc_z, 2);
    setVal('gx', d.gyr_x, 1); setVal('gy', d.gyr_y, 1); setVal('gz', d.gyr_z, 1);
    setVal('it', d.imu_temp, 1);

    // BMP280
    setVal('bpres', d.bmp_pres, 1);
    setVal('balt',  d.bmp_alt,  1);

    // Air quality
    const aq = d.mq_raw;
    setVal('aqraw', 'ADC: '+aq+'/4095');
    let label, cls;
    if      (aq < 800)  { label='GOOD';     cls='badge-good'; }
    else if (aq < 1500) { label='MODERATE'; cls='badge-warn'; }
    else if (aq < 2500) { label='POOR';     cls='badge-warn'; }
    else                { label='VERY POOR';cls='badge-bad';  }
    document.getElementById('aqval').textContent = label;
    document.getElementById('aqbadge').innerHTML =
      d.mq_alert ? '<span class="badge badge-bad">THRESHOLD ALERT</span>' : '';
  });
}
setInterval(pollStatus, 500);
pollStatus();
</script>
</body>
</html>)=====";

// ================= Sensor read functions =================
void readMLX(){
  if(!mlxOk || millis()-lastMLX < 1000) return;
  lastMLX = millis();
  mlx_obj = mlx.readObjectTempC();
  mlx_amb = mlx.readAmbientTempC();
}

void readMPU(){
  if(!mpuOk || millis()-lastMPU < 50) return;
  lastMPU = millis();

  // Read accel — 6 bytes from 0x3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  int16_t ax = (Wire.read()<<8)|Wire.read();
  int16_t ay = (Wire.read()<<8)|Wire.read();
  int16_t az = (Wire.read()<<8)|Wire.read();

  // Read temp — 2 bytes from 0x41
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_TEMP_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  int16_t rt = (Wire.read()<<8)|Wire.read();

  // Read gyro — 6 bytes from 0x43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  int16_t gx = (Wire.read()<<8)|Wire.read();
  int16_t gy = (Wire.read()<<8)|Wire.read();
  int16_t gz = (Wire.read()<<8)|Wire.read();

  // Convert to real units
  acc_x    = ax / ACCEL_SCALE * 9.80665;
  acc_y    = ay / ACCEL_SCALE * 9.80665;
  acc_z    = az / ACCEL_SCALE * 9.80665;
  gyr_x    = gx / GYRO_SCALE;
  gyr_y    = gy / GYRO_SCALE;
  gyr_z    = gz / GYRO_SCALE;
  imu_temp = (rt / 333.87) + 21.0;  // MPU6500 temp formula
}

void readBMP(){
  if(!bmpOk || millis()-lastBMP < 500) return;
  lastBMP  = millis();
  bmp_temp = bmp.readTemperature();
  bmp_pres = bmp.readPressure() / 100.0F;
  bmp_alt  = bmp.readAltitude(1013.25);
}

void readMQ135(){
  if(millis()-lastMQ < 200) return;
  lastMQ   = millis();
  mq_raw   = analogRead(MQ135_AOUT);
  mq_alert = (digitalRead(MQ135_DOUT) == LOW);
}

// Send all sensor data to Pi over Serial2 every 2s
void sendSensorsToPi(){
  if(millis()-lastSensorTx < SENSOR_TX_INTERVAL) return;
  lastSensorTx = millis();

  // Thermal — send then yield
  Serial2.print("{\"type\":\"thermal\",\"object\":");
  Serial2.print(mlx_obj,1);
  Serial2.print(",\"ambient\":");
  Serial2.print(mlx_amb,1);
  Serial2.println("}");
  delay(10);  // yield between packets

  // IMU
  Serial2.print("{\"type\":\"imu\",\"ax\":");
  Serial2.print(acc_x,2); Serial2.print(",\"ay\":"); Serial2.print(acc_y,2);
  Serial2.print(",\"az\":"); Serial2.print(acc_z,2);
  Serial2.print(",\"gx\":"); Serial2.print(gyr_x,2);
  Serial2.print(",\"gy\":"); Serial2.print(gyr_y,2);
  Serial2.print(",\"gz\":"); Serial2.print(gyr_z,2);
  Serial2.println("}");
  delay(10);

  // Environment
  Serial2.print("{\"type\":\"env\",\"pressure\":");
  Serial2.print(bmp_pres,1); Serial2.print(",\"altitude\":");
  Serial2.print(bmp_alt,1);  Serial2.print(",\"mq_raw\":");
  Serial2.print(mq_raw);     Serial2.print(",\"mq_alert\":");
  Serial2.print(mq_alert ? "true" : "false");
  Serial2.println("}");
}

// ================= Sonar =================
float getDistanceCm(){
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long d = pulseIn(ECHO_PIN, HIGH, 30000);
  if(d==0) return -1;
  return d * 0.0343 / 2.0;
}

void updateSafety(){
  if(!safetyEnabled){ safetyTriggered=false; obstacleCounter=0; return; }
  lastDistance = getDistanceCm();
  if(lastDistance>0 && lastDistance<SAFETY_DIST_CM){
    obstacleCounter++;
    if(obstacleCounter>=CONFIRM_COUNT) safetyTriggered=true;
  } else { obstacleCounter=0; safetyTriggered=false; }
}

// ================= LED =================
void updateLEDs(){
  if(millis()-lastBlink>400){ blinkState=!blinkState; lastBlink=millis(); }

  uint32_t c_W  = frontStrip.Color(255,229,122);
  uint32_t c_G  = frontStrip.Color(102,  0,204);
  uint32_t c_O  = frontStrip.Color(255,100,  0);
  uint32_t c_DR = frontStrip.Color(102,  0,  0);
  uint32_t c_BR = frontStrip.Color(255,  0,  0);
  uint32_t c_SA = frontStrip.Color(255,140,  0);
  uint32_t blinkColor = blinkState ? c_O : c_W;

  // Front
  if(safetyTriggered && forwardCmd){
    uint32_t warn = blinkState ? c_SA : frontStrip.Color(80,40,0);
    for(int i=0;i<NUM_FRONT;i++) frontStrip.setPixelColor(i,warn);
  } else {
    for(int i=0;i<NUM_FRONT;i++) frontStrip.setPixelColor(i,c_W);
    frontStrip.setPixelColor(2,c_G); frontStrip.setPixelColor(3,c_G);
    frontStrip.setPixelColor(4,c_G); frontStrip.setPixelColor(5,c_G);
    if(leftCmd)  frontStrip.setPixelColor(0,blinkColor);
    if(rightCmd) frontStrip.setPixelColor(7,blinkColor);
  }

  // Back
  bool movingBack = backCmd && !obstacleMode && !followMode;
  if(brakeCmd){
    for(int i=0;i<NUM_BACK;i++) backStrip.setPixelColor(i,c_BR);
  } else if(movingBack){
    backStrip.setPixelColor(0,c_W); backStrip.setPixelColor(1,c_W);
    for(int i=2;i<=5;i++) backStrip.setPixelColor(i,c_BR);
    backStrip.setPixelColor(6,c_W); backStrip.setPixelColor(7,c_W);
  } else {
    backStrip.setPixelColor(0,c_O);
    for(int i=1;i<=6;i++) backStrip.setPixelColor(i,c_DR);
    backStrip.setPixelColor(7,c_O);
    if(leftCmd){
      backStrip.setPixelColor(0,c_O);
      for(int i=1;i<=5;i++) backStrip.setPixelColor(i,c_DR);
      backStrip.setPixelColor(6,blinkColor); backStrip.setPixelColor(7,blinkColor);
    } else if(rightCmd){
      backStrip.setPixelColor(0,blinkColor); backStrip.setPixelColor(1,blinkColor);
      for(int i=2;i<=6;i++) backStrip.setPixelColor(i,c_DR);
      backStrip.setPixelColor(7,c_O);
    }
  }
  frontStrip.show(); backStrip.show();
}

// ================= Motors =================
void setMotorSpeed(uint8_t l, uint8_t r){ ledcWrite(ENA_PIN,l); ledcWrite(ENB_PIN,r); }
void driveStop(){ digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);setMotorSpeed(0,0); }
void driveForward(uint8_t s){ digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);setMotorSpeed(s,s); }
void driveBackward(uint8_t s){ digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);setMotorSpeed(s,s); }
void driveLeft(uint8_t s){ digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);setMotorSpeed(s,s); }
void driveRight(uint8_t s){ digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);setMotorSpeed(s,s); }

void handleManual(){
  if(forwardCmd){ if(safetyTriggered) driveStop(); else driveForward(manualSpeed); }
  else if(backCmd)  driveBackward(manualSpeed);
  else if(leftCmd)  driveLeft(manualSpeed);
  else if(rightCmd) driveRight(manualSpeed);
  else              driveStop();
}

void handleObstacle(){
  unsigned long now=millis();
  switch(autoState){
    case AUTO_IDLE: autoState=AUTO_FORWARD; break;
    case AUTO_FORWARD:{
      float d=getDistanceCm();
      if(d>0&&d<OBSTACLE_DIST_CM){ driveStop();autoState=AUTO_BACKWARD;autoStateStart=now; }
      else driveForward(autoSpeed);
    }break;
    case AUTO_BACKWARD:
      if(now-autoStateStart>=BACK_TIME){ autoState=AUTO_TURN;autoStateStart=now; }
      else driveBackward(autoSpeed); break;
    case AUTO_TURN:
      turnLeftNext?driveLeft(autoSpeed):driveRight(autoSpeed);
      if(now-autoStateStart>=TURN_TIME){ driveStop();turnLeftNext=!turnLeftNext;autoState=AUTO_FORWARD; }
      break;
  }
}

void handleFollow(){
  float d=getDistanceCm();
  if(d<0){driveStop();return;}
  if(d<12) driveBackward(autoSpeed);
  else if(d<=20) driveStop();
  else if(d<=45) driveForward(autoSpeed);
  else driveStop();
}

// ================= Pi UART =================
void handlePiSerial(){
  while(Serial2.available()){
    char c=Serial2.read();
    lastPiCmd=millis(); piConnected=true;
    forwardCmd=backCmd=leftCmd=rightCmd=false; brakeCmd=false;
    switch(c){
      case 'F': forwardCmd=true; break;
      case 'B': backCmd=true;    break;
      case 'L': leftCmd=true;    break;
      case 'R': rightCmd=true;   break;
      case 'S': brakeCmd=true; driveStop(); break;
      case 'E': safetyEnabled=true;  break;
      case 'D': safetyEnabled=false; break;
      case 'O': obstacleMode=true;  followMode=false; autoState=AUTO_FORWARD; break;
      case 'P': followMode=true;    obstacleMode=false; break;
      case 'M': obstacleMode=false; followMode=false; driveStop(); break;
    }
    Serial2.print("OK:"); Serial2.println(c);
  }
  if(piConnected && millis()-lastPiCmd>PI_TIMEOUT){
    piConnected=false; driveStop();
  }
}

// ================= HTTP =================
void handleRoot(){ server.send_P(200,"text/html",index_html); }

void handleCmd(){
  if(obstacleMode||followMode){ server.send(200,"text/plain","IGNORED:MODE"); return; }
  if(piConnected){ server.send(200,"text/plain","IGNORED:PI"); return; }
  String d=server.arg("dir"); bool p=server.arg("state").toInt();
  if(d=="f")      { forwardCmd=p; if(p) brakeCmd=false; }
  else if(d=="b") { backCmd=p;    if(p) brakeCmd=false; }
  else if(d=="l") { leftCmd=p;    if(p) brakeCmd=false; }
  else if(d=="r") { rightCmd=p;   if(p) brakeCmd=false; }
  else if(d=="s") { forwardCmd=backCmd=leftCmd=rightCmd=false; brakeCmd=true; driveStop(); }
  server.send(200,"text/plain","OK");
}

void handleMode(){
  obstacleMode=server.arg("auto").toInt(); followMode=false;
  autoState=AUTO_FORWARD; forwardCmd=backCmd=leftCmd=rightCmd=false; driveStop();
  server.send(200,"text/plain","OK");
}

void handleFollowHttp(){
  followMode=server.arg("f").toInt(); obstacleMode=false;
  forwardCmd=backCmd=leftCmd=rightCmd=false; driveStop();
  server.send(200,"text/plain","OK");
}

void handleSafety(){
  safetyEnabled=server.arg("en").toInt(); obstacleCounter=0; safetyTriggered=false;
  server.send(200,"text/plain",safetyEnabled?"SAFETY:ON":"SAFETY:OFF");
}

void handleSpeed(){
  manualSpeed=constrain(server.arg("v").toInt(),100,255);
  server.send(200,"text/plain","OK");
}

void handleStatus(){
  String j="{";
  j+="\"safety\":"         +String(safetyEnabled?"true":"false")+",";
  j+="\"triggered\":"      +String(safetyTriggered?"true":"false")+",";
  j+="\"distance\":"       +String(lastDistance,1)+",";
  j+="\"dist_threshold\":" +String(SAFETY_DIST_CM,0)+",";
  j+="\"pi_connected\":"   +String(piConnected?"true":"false")+",";
  j+="\"speed\":"          +String(manualSpeed)+",";
  // Thermal
  j+="\"mlx_ok\":"         +String(mlxOk?"true":"false")+",";
  j+="\"obj_temp\":"       +String(mlx_obj,1)+",";
  j+="\"amb_temp\":"       +String(mlx_amb,1)+",";
  // IMU
  j+="\"mpu_ok\":"         +String(mpuOk?"true":"false")+",";
  j+="\"acc_x\":"          +String(acc_x,2)+",";
  j+="\"acc_y\":"          +String(acc_y,2)+",";
  j+="\"acc_z\":"          +String(acc_z,2)+",";
  j+="\"gyr_x\":"          +String(gyr_x,2)+",";
  j+="\"gyr_y\":"          +String(gyr_y,2)+",";
  j+="\"gyr_z\":"          +String(gyr_z,2)+",";
  j+="\"imu_temp\":"       +String(imu_temp,1)+",";
  // BMP280
  j+="\"bmp_ok\":"         +String(bmpOk?"true":"false")+",";
  j+="\"bmp_pres\":"       +String(bmp_pres,1)+",";
  j+="\"bmp_alt\":"        +String(bmp_alt,1)+",";
  // MQ135
  j+="\"mq_raw\":"         +String(mq_raw)+",";
  j+="\"mq_alert\":"       +String(mq_alert?"true":"false");
  j+="}";
  server.send(200,"application/json",j);
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,16,17);

  brakeCmd=false; safetyTriggered=false; obstacleCounter=0;

  // I2C — shared bus for all three sensors
  Wire.begin(21,22);

  // MLX90614
  if(mlx.begin()){ mlxOk=true; Serial.println("MLX90614 OK"); }
  else Serial.println("MLX90614 FAILED");

  // MPU6500 via Adafruit MPU6050 library
  // Wake MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT); Wire.write(0x00);
  Wire.endTransmission(); delay(100);
  // Set accel ±4G
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CFG); Wire.write(0x08);
  Wire.endTransmission();
  // Set gyro ±500°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CFG); Wire.write(0x08);
  Wire.endTransmission();
  // Verify WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  byte whoami = Wire.read();
  if(whoami == 0x70 || whoami == 0x71 || whoami == 0x73){
      mpuOk = true;
      Serial.print("MPU6500 OK — WHO_AM_I=0x");
      Serial.println(whoami, HEX);
  } else {
      Serial.print("MPU6500 FAILED — WHO_AM_I=0x");
      Serial.println(whoami, HEX);
  }

  // BMP280
  if(bmp.begin(0x76)){
    bmpOk=true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 OK");
  } else Serial.println("BMP280 FAILED");

  // MQ135
  pinMode(MQ135_AOUT,INPUT);
  pinMode(MQ135_DOUT,INPUT);
  Serial.println("MQ135 OK — warm up 3-5 min for accuracy");

  // LEDs
  frontStrip.begin(); frontStrip.setBrightness(180); frontStrip.show();
  backStrip.begin();  backStrip.setBrightness(180);  backStrip.show();

  // Motors
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcAttach(ENA_PIN,PWM_FREQ,PWM_RES);
  ledcAttach(ENB_PIN,PWM_FREQ,PWM_RES);

  // Sonar
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  // WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid,apPassword);

  server.on("/",       handleRoot);
  server.on("/cmd",    handleCmd);
  server.on("/mode",   handleMode);
  server.on("/follow", handleFollowHttp);
  server.on("/safety", handleSafety);
  server.on("/speed",  handleSpeed);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println("ARES-X online");
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());
}

// ================= LOOP =================
void loop(){
  server.handleClient();
  handlePiSerial();
  updateSafety();
  readMPU();         // every loop — no gate
  readMLX();
  readBMP();
  readMQ135();
  sendSensorsToPi(); // broadcast all sensor JSON to Pi every 2s

  if(followMode)        handleFollow();
  else if(obstacleMode) handleObstacle();
  else                  handleManual();

  updateLEDs();
  delay(5);
}
