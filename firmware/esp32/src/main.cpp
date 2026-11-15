/*
 * Puente ESP32: HMI (PC)  <-->  ESP32 (maestro Modbus RTU)  <-->  FPGA (esclavo)
 *
 *  - Cada POLL_MS lee 9 holding registers de la FPGA (función 0x03, una sola trama)
 *    y envía la telemetría a la HMI por USB y, si hay cliente, por Wi-Fi (TCP).
 *  - Cuando la HMI manda un comando, escribe en la FPGA (0x06 para SP / EN, 0x10 para Kp-Ki-Kd).
 *
 * MAPA DE REGISTROS DE LA FPGA (esclavo ID 1, 16 bits; ver lev_pkg.vhd / mb_regmap.vhd):
 *   Dir  Nombre   Acceso  Descripción   (PV = 655.35 -> sensor sin eco)
 *    0   SP       R/W     Referencia: DISTANCIA sensor-pelota  (cm x100)  p.ej. 2000 = 20.00 cm
 *    1   KP       R/W     Ganancia P  [PWM/cm]          x100   p.ej.  80 = 0.80
 *    2   KI       R/W     Ganancia I  [PWM/(cm*s)]      x100   p.ej.  78 = 0.78
 *    3   KD       R/W     Ganancia D  [PWM*s/cm]        x100   p.ej.  25 = 0.25
 *    4   EN       R/W     Habilitar lazo (0/1)
 *    5   PV       R       DISTANCIA medida sensor-pelota (cm x100)
 *    6   ERR      R       SP-PV en distancias, int16 con signo (cm x100)
 *    7   PWM      R       Duty calculado por el PID     (0-255)
 *    8   STATUS   R       bit0 lazo on, bit1 sensor sin eco, bit2 Arduino sin comunicar,
 *                         bit3 PWM saturado arriba, bit4 PWM en el minimo, bit5 trama Arduino mala
 *    9   BIAS     R/W     PWM base de sustentacion (no lo usa este puente)
 * Limites de la FPGA (si se pasan responde excepcion 03): SP <= 100.00, KP <= 159.99, KD <= 7.99.
 *
 * ALTURA vs DISTANCIA: el sensor esta ARRIBA del tubo y la FPGA trabaja con distancias.
 * La HMI trabaja con ALTURAS (0 = abajo). Este puente convierte con
 *     altura = H0_CM - distancia
 * donde H0_CM es la distancia del sensor a la pelota cuando descansa en el fondo (calibrar).
 * Por eso el error tambien cambia de signo: (SP-PV) en alturas = -(SP-PV) en distancias.
 *
 * Protocolo con la HMI (ASCII, una trama por línea; todo en ALTURAS, cm):
 *   ESP32 -> PC : *PV,SP,ERR,PWM,EN,KP,KI,KD,MB,ST#   (MB = código ModbusMaster, 0 = OK;
 *                                                      ST = registro STATUS de la FPGA)
 *                 *ACK,<SP|PID|EN>,<código Modbus>#   (respuesta a un comando)
 *   PC -> ESP32 : *SP,20.00#   *PID,1.60,0.20,1.50#   *EN,1#
 *
 * Conexiones (módulo RS485 de flujo automático, sin RE/DE):
 *   ESP32 GPIO17 (TX2) -> RXD del transceptor      ESP32 GPIO16 (RX2) <- TXD del transceptor
 */

#include <Arduino.h>
#include <ModbusMaster.h>

// ------------------------- Configuración -------------------------
#define USE_WIFI        1                 // 0 = solo USB
const char *WIFI_SSID = "TU_RED_WIFI";       // nombre de la red Wi-Fi
const char *WIFI_PASS = "TU_CLAVE_WIFI";     // clave de la red Wi-Fi
#define TCP_PORT        8888

#define MODBUS_RX_PIN   16
#define MODBUS_TX_PIN   17
// #define MODBUS_DE_RE_PIN 4             // solo si el módulo NO es de flujo automático
#define SLAVE_ID        1
#define MB_BAUD         9600              // igual que la UART Modbus de la FPGA
#define MB_CONFIG       SERIAL_8N1
#define POLL_MS         100               // periodo de muestreo / telemetría
#define H0_CM           46.0f             // distancia sensor -> pelota en el fondo (PV con ventilador apagado)
#define ALT_MAX_CM      37.0f             // altura maxima permitida para la referencia (igual a SP_MAX de la HMI)

#if USE_WIFI
#include <WiFi.h>
WiFiServer server(TCP_PORT);
WiFiClient client;
#endif

// ------------------------- Registros -------------------------
enum : uint16_t {
  REG_SP = 0, REG_KP, REG_KI, REG_KD, REG_EN, REG_PV, REG_ERR, REG_PWM, REG_STATUS, NUM_REGS
};
// Limites de las ganancias en la FPGA (en unidades de la HMI)
const float KP_MAX = 159.99f, KI_MAX = 655.35f, KD_MAX = 7.99f;
uint16_t regs[NUM_REGS] = {0};   // última copia leída de la FPGA
const uint16_t PV_SIN_ECO = 0xFFFF;               // la FPGA satura PV a 65535 si el sensor no da eco
float pvAltOk = 0.0f, errAltOk = 0.0f;            // ultimo PV / error validos (en alturas)

ModbusMaster node;
String rxUsb, rxTcp;
uint32_t lastPoll = 0;

// ------------------------- Utilidades -------------------------
static uint16_t toFix(float v) {                 // float -> entero x100 (0..655.35)
  v = constrain(v, 0.0f, 655.35f);
  return (uint16_t)lroundf(v * 100.0f);
}

static float distToAlt(uint16_t reg) {           // registro de distancia (cm x100) -> altura cm
  return H0_CM - reg / 100.0f;
}

void sendOut(const char *msg) {                  // a USB y al cliente Wi-Fi
  Serial.print(msg);
#if USE_WIFI
  if (client && client.connected()) client.print(msg);
#endif
}

void preTransmission() {
#ifdef MODBUS_DE_RE_PIN
  digitalWrite(MODBUS_DE_RE_PIN, HIGH);
  delayMicroseconds(50);
#endif
}
void postTransmission() {
#ifdef MODBUS_DE_RE_PIN
  delayMicroseconds(50);
  digitalWrite(MODBUS_DE_RE_PIN, LOW);
#endif
}

// ------------------------- Lectura de la FPGA + telemetría -------------------------
void pollFpga() {
  uint8_t r = node.readHoldingRegisters(0, NUM_REGS);       // 0x03, 9 registros
  if (r == node.ku8MBSuccess) {
    for (uint8_t i = 0; i < NUM_REGS; i++) regs[i] = node.getResponseBuffer(i);
  }
  // Si falla, se reenvía la última copia con el código de error en MB.
  // Distancias -> alturas; el error SP-PV cambia de signo al pasar a alturas.
  // Sin eco (PV = 65535): se repite el ultimo PV valido; el bit 1 de STATUS avisa a la HMI.
  if (regs[REG_PV] != PV_SIN_ECO) {
    pvAltOk  = distToAlt(regs[REG_PV]);
    errAltOk = -(int16_t)regs[REG_ERR] / 100.0f;
  }
  char buf[128];
  snprintf(buf, sizeof(buf), "*%.2f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.2f,%u,%u#\n",
           pvAltOk, distToAlt(regs[REG_SP]), errAltOk,
           (unsigned)regs[REG_PWM], (unsigned)(regs[REG_EN] & 1),
           regs[REG_KP] / 100.0f, regs[REG_KI] / 100.0f, regs[REG_KD] / 100.0f, (unsigned)r,
           (unsigned)regs[REG_STATUS]);
  sendOut(buf);
}

// ------------------------- Comandos de la HMI -------------------------
void handleCommand(const String &s) {
  if (s.length() < 4 || s[0] != '*' || s[s.length() - 1] != '#') return;
  String body = s.substring(1, s.length() - 1);
  const char *name = nullptr;
  uint8_t r = 0xFF;

  if (body.startsWith("SP,")) {
    float h = constrain(body.substring(3).toFloat(), 0.0f, fminf(ALT_MAX_CM, H0_CM));
    r = node.writeSingleRegister(REG_SP, toFix(H0_CM - h));  // 0x06, altura -> distancia
    name = "SP";
  } else if (body.startsWith("PID,")) {
    float kp, ki, kd;
    name = "PID";
    if (sscanf(body.c_str() + 4, "%f,%f,%f", &kp, &ki, &kd) == 3) {
      node.setTransmitBuffer(0, toFix(constrain(kp, 0.0f, KP_MAX)));
      node.setTransmitBuffer(1, toFix(constrain(ki, 0.0f, KI_MAX)));
      node.setTransmitBuffer(2, toFix(constrain(kd, 0.0f, KD_MAX)));
      r = node.writeMultipleRegisters(REG_KP, 3);            // 0x10, Kp-Ki-Kd en una trama
    }
  } else if (body.startsWith("EN,")) {
    r = node.writeSingleRegister(REG_EN, body.substring(3).toInt() ? 1 : 0);  // 0x06
    name = "EN";
  } else {
    return;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "*ACK,%s,%u#\n", name, (unsigned)r);
  sendOut(buf);
}

// Acumula caracteres de un Stream y despacha cada línea completa
void pump(Stream &s, String &buf) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) handleCommand(buf);
      buf = "";
    } else if (c != '\r' && buf.length() < 80) {
      buf += c;
    }
  }
}

// ------------------------- Setup / loop -------------------------
void setup() {
  Serial.begin(115200);                                      // USB hacia la HMI

#ifdef MODBUS_DE_RE_PIN
  pinMode(MODBUS_DE_RE_PIN, OUTPUT);
  digitalWrite(MODBUS_DE_RE_PIN, LOW);
#endif
  Serial2.begin(MB_BAUD, MB_CONFIG, MODBUS_RX_PIN, MODBUS_TX_PIN);
  node.begin(SLAVE_ID, Serial2);
#ifdef MODBUS_DE_RE_PIN
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
#endif

#if USE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    Serial.print("IP ESP32: ");                              // línea informativa (no es trama)
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi no disponible: solo USB");
  }
#endif
  Serial.println("Puente Modbus RTU listo");
}

void loop() {
  pump(Serial, rxUsb);
#if USE_WIFI
  if (server.hasClient()) {                                  // un único cliente a la vez
    if (client && client.connected()) client.stop();
    client = server.available();
    client.setNoDelay(true);
    rxTcp = "";
  }
  if (client && client.connected()) pump(client, rxTcp);
#endif

  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();
    pollFpga();
  }
}
