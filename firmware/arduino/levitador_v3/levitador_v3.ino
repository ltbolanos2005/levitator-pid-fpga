// =====================================================================
// LEVITADOR NEUMATICO - ARDUINO UNO - version 3
//
// El Arduino NO calcula el PID: mide, envia PV a la FPGA y aplica el PWM
// que la FPGA le devuelve.  Tambien sirve de "consola" por el Monitor
// Serie para cambiar parametros mientras no exista la interfaz grafica.
//
// CONEXIONES
//   HC-SR04  TRIG -> D2     ECHO -> D3     (sensor ARRIBA del tubo)
//   MOSFET   SIG  -> D9                    (PWM del ventilador)
//   FPGA     D10 (RX) <- TX de la FPGA     (3.3 V es suficiente)
//            D11 (TX) -> RX de la FPGA     (con DIVISOR 5V -> 3.3V: 1k + 2k)
//   GND comun entre Arduino, FPGA y fuente del ventilador.
//
// PROTOCOLO (9600 8N1, CHK = XOR de todos los bytes anteriores)
//   Arduino -> FPGA : AA PV_H PV_L CHK            (PV en mm, FFFF = sin eco)
//   FPGA -> Arduino : 55 MV STATUS CHK
//   Escribir: A5 REG VAL_H VAL_L CHK   Leer: A6 REG 00 00 CHK
//   Respuesta: 5A REG VAL_H VAL_L CHK  (REG con bit 7 = error)
//
// PV y SP son DISTANCIAS medidas desde el sensor (arriba) hasta la pelota.
//
// COMANDOS (Monitor Serie 115200, "Nueva linea")
//   sp 150        referencia: distancia sensor-pelota en mm
//   kp 0.08       ganancia P en PWM por mm
//   ki 0.004      ganancia I en PWM por (mm*muestra)
//   kd 0.5        ganancia D en PWM por (mm/muestra)
//   bias 120      PWM base de sustentacion (0..255)
//   cal 120       MODO MANUAL: KP=KI=KD=0 y PWM = 120 (para calibrar)
//   on / off      arrancar / detener el lazo
//   leer          muestra todos los registros (uno por ciclo, sin parar el lazo)
//   plot          alterna salida compatible con el Serial Plotter
//   ayuda         lista de comandos
// =====================================================================
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

const uint8_t TRIG_PIN = 2;
const uint8_t ECHO_PIN = 3;
const uint8_t PWM_PIN  = 9;

SoftwareSerial fpgaSerial(10, 11);   // RX = 10, TX = 11

// ---- Temporizacion --------------------------------------------------
const unsigned long PERIODO_MS = 50;    // 20 muestras/s  (el PID de la FPGA asume 50 ms)
const unsigned long TIMEOUT_MS = 25;    // espera maxima de la respuesta de la FPGA
const int           MAX_FALLOS = 5;     // fallos seguidos -> ventilador apagado

// ---- Sensor ---------------------------------------------------------
const uint16_t PV_SIN_ECO  = 0xFFFF;
const uint16_t DIST_MIN_MM = 20;        // el HC-SR04 no es fiable a menos de ~2 cm
const uint16_t DIST_MAX_MM = 1000;      // ajustar al largo de tu tubo
const uint16_t SALTO_MAX_MM = 60;       // salto imposible entre dos muestras (50 ms)
const uint8_t  MAX_RECHAZOS = 2;        // muestras malas seguidas toleradas

// ---- Registros de la FPGA (mismo mapa que MODBUS) ---------------------
const uint8_t REG_PV = 0, REG_SP = 1, REG_MV = 2, REG_ERROR = 3, REG_KP = 4,
              REG_KI = 5, REG_KD = 6, REG_ENABLE = 7, REG_STATUS = 8, REG_BIAS = 9;
const char* NOMBRES[10] = {"PV", "SP", "MV", "ERROR", "KP", "KI", "KD", "ENABLE", "STATUS", "BIAS"};
const float GAIN_SCALE = 4096.0;        // las ganancias son valor/4096 (GAIN_SHIFT = 12)

int           fallos = 0;
uint16_t      spCache = 150;
uint8_t       statusAnterior = 0xFF;
bool          modoPlot = false;
unsigned long tUltima = 0;
int8_t        leerIdx = -1;             // >= 0: lectura de registros en curso

uint16_t      pvValido = PV_SIN_ECO;    // ultimo PV aceptado por el filtro
uint8_t       rechazos = 0;

char          lineBuf[32];
uint8_t       lineLen = 0;

// ---------------------------------------------------------------------
// Enlace con la FPGA
// ---------------------------------------------------------------------
void vaciarEntrada() {
  while (fpgaSerial.available()) fpgaSerial.read();
}

bool leerBytes(uint8_t* buf, int n, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  int i = 0;
  while (i < n) {
    if (fpgaSerial.available()) {
      buf[i++] = fpgaSerial.read();
    } else if (millis() - t0 > timeoutMs) {
      return false;
    }
  }
  return true;
}

// Escribir (0xA5) o leer (0xA6) un registro. true si la FPGA confirma.
bool comandoUnaVez(uint8_t cmd, uint8_t reg, uint16_t valor, uint16_t &respuesta) {
  uint8_t t[5] = {cmd, reg, (uint8_t)(valor >> 8), (uint8_t)(valor & 0xFF), 0};
  t[4] = t[0] ^ t[1] ^ t[2] ^ t[3];
  vaciarEntrada();
  fpgaSerial.write(t, 5);
  uint8_t r[5];
  if (!leerBytes(r, 5, TIMEOUT_MS + 10)) return false;
  if (r[0] != 0x5A || (r[0] ^ r[1] ^ r[2] ^ r[3] ^ r[4]) != 0) return false;
  if (r[1] & 0x80) return false;            // la FPGA rechazo el registro
  respuesta = ((uint16_t)r[2] << 8) | r[3];
  return true;
}

bool comandoRegistro(uint8_t cmd, uint8_t reg, uint16_t valor, uint16_t &respuesta) {
  for (int i = 0; i < 3; i++) {             // hasta 3 intentos
    if (comandoUnaVez(cmd, reg, valor, respuesta)) return true;
    delay(10);
  }
  return false;
}

// ---------------------------------------------------------------------
// Sensor HC-SR04 (mm) y filtro anti-picos
// ---------------------------------------------------------------------
uint16_t medirMM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duracion == 0) return PV_SIN_ECO;
  unsigned long mm = duracion * 343UL / 2000UL;       // 0.343 mm/us, ida y vuelta
  if (mm < DIST_MIN_MM || mm > DIST_MAX_MM) return PV_SIN_ECO;
  return (uint16_t)mm;
}

// Rechaza hasta MAX_RECHAZOS muestras seguidas que sean "sin eco" o que
// salten mas de SALTO_MAX_MM (repite la ultima buena). No agrega retardo
// a los movimientos reales, y evita que un eco perdido apague el ventilador
// (la FPGA lo hace con PV = FFFF).
uint16_t filtrar(uint16_t raw) {
  bool malo = (raw == PV_SIN_ECO) ||
              (pvValido != PV_SIN_ECO && abs((int)raw - (int)pvValido) > (int)SALTO_MAX_MM);
  if (!malo) {
    pvValido = raw;
    rechazos = 0;
    return raw;
  }
  if (pvValido != PV_SIN_ECO && rechazos < MAX_RECHAZOS) {
    rechazos++;
    return pvValido;
  }
  pvValido = raw;                           // falla persistente o salto real
  rechazos = 0;
  return raw;
}

// ---------------------------------------------------------------------
// Consola
// ---------------------------------------------------------------------
void imprimirAyuda() {
  Serial.println(F("Comandos: sp <mm> | kp <x> | ki <x> | kd <x> | bias <0-255> | cal <0-255> | on | off | leer | plot | ayuda"));
  Serial.println(F("Ganancias en unidades fisicas: kp en PWM/mm, ki en PWM/(mm*muestra), kd en PWM/(mm/muestra)"));
  Serial.println(F("PV y SP son distancias desde el sensor (arriba) hasta la pelota."));
}

void imprimirStatus(uint8_t st) {
  Serial.print(F("STATUS 0x")); Serial.print(st, HEX); Serial.print(F(" ->"));
  if (st & 0x01) Serial.print(F(" LAZO_ON")); else Serial.print(F(" LAZO_OFF"));
  if (st & 0x02) Serial.print(F(" SIN_ECO"));
  if (st & 0x04) Serial.print(F(" TIMEOUT"));
  if (st & 0x08) Serial.print(F(" SAT_ALTA"));
  if (st & 0x10) Serial.print(F(" PWM_MINIMO"));
  if (st & 0x20) Serial.print(F(" TRAMA_MALA"));
  Serial.println();
}

void imprimirValor(uint8_t reg, uint16_t v) {
  if (reg == REG_ERROR) {
    Serial.println((int16_t)v);
  } else if (reg == REG_KP || reg == REG_KI || reg == REG_KD) {
    Serial.print(v / GAIN_SCALE, 5); Serial.print(F("  (raw ")); Serial.print(v); Serial.println(F(")"));
  } else {
    Serial.println(v);
  }
}

void leerSiguiente() {
  uint16_t v;
  Serial.print(NOMBRES[leerIdx]); Serial.print(F(" = "));
  if (comandoRegistro(0xA6, leerIdx, 0, v)) {
    imprimirValor(leerIdx, v);
    if (leerIdx == REG_SP) spCache = v;
  } else {
    Serial.println(F("(sin respuesta)"));
  }
  if (++leerIdx >= 10) leerIdx = -1;
}

bool escribir(uint8_t reg, long valor) {
  if (valor < 0 || valor > 65535) { Serial.println(F("Valor fuera de rango")); return false; }
  uint16_t r;
  if (comandoRegistro(0xA5, reg, (uint16_t)valor, r)) {
    Serial.print(F("OK ")); Serial.print(NOMBRES[reg]); Serial.print(F(" = ")); imprimirValor(reg, r);
    if (reg == REG_SP) spCache = r;
    return true;
  }
  Serial.println(F("ERROR: la FPGA no confirmo el comando"));
  return false;
}

void escribirGanancia(uint8_t reg, const char* arg) {
  if (!arg) { Serial.println(F("Falta el valor")); return; }
  float g = atof(arg);
  if (g < 0.0 || g >= 16.0) { Serial.println(F("Ganancia fuera de rango (0 a 15.99)")); return; }
  escribir(reg, (long)(g * GAIN_SCALE + 0.5));
}

void escribirEntero(uint8_t reg, const char* arg) {
  if (!arg) { Serial.println(F("Falta el valor")); return; }
  escribir(reg, atol(arg));
}

// Modo manual para calibrar: sin PID, PWM fijo (la FPGA aplica ademas su PWM minimo)
void calibrar(const char* arg) {
  if (!arg) { Serial.println(F("Uso: cal <0-255>")); return; }
  long pwm = atol(arg);
  if (pwm < 0 || pwm > 255) { Serial.println(F("PWM fuera de rango (0-255)")); return; }
  if (escribir(REG_KP, 0) && escribir(REG_KI, 0) && escribir(REG_KD, 0) &&
      escribir(REG_BIAS, pwm) && escribir(REG_ENABLE, 1)) {
    Serial.println(F("MODO MANUAL: MV = BIAS. Cambia el PWM con 'bias <n>'. Restaura el PID con kp/ki/kd."));
  }
}

void procesarLinea(char* s) {
  char* cmd = strtok(s, " ");
  char* arg = strtok(NULL, " ");
  if (!cmd) return;

  if      (!strcmp(cmd, "sp"))   escribirEntero(REG_SP, arg);
  else if (!strcmp(cmd, "kp"))   escribirGanancia(REG_KP, arg);
  else if (!strcmp(cmd, "ki"))   escribirGanancia(REG_KI, arg);
  else if (!strcmp(cmd, "kd"))   escribirGanancia(REG_KD, arg);
  else if (!strcmp(cmd, "bias")) escribirEntero(REG_BIAS, arg);
  else if (!strcmp(cmd, "cal"))  calibrar(arg);
  else if (!strcmp(cmd, "on"))   escribir(REG_ENABLE, 1);
  else if (!strcmp(cmd, "off"))  escribir(REG_ENABLE, 0);
  else if (!strcmp(cmd, "leer") || !strcmp(cmd, "?")) leerIdx = 0;
  else if (!strcmp(cmd, "plot")) modoPlot = !modoPlot;
  else imprimirAyuda();
}

void revisarSerialPC() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        lineLen = 0;
        procesarLinea(lineBuf);
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = tolower(c);
    }
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);    // Monitor Serie del PC
  fpgaSerial.begin(9600);  // Enlace con la FPGA

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  // Opcional: PWM de ~31 kHz en D9 (silencia el zumbido del ventilador).
  // Probar solo si tu modulo MOSFET conmuta bien a esa frecuencia.
  // TCCR1B = (TCCR1B & 0xF8) | 0x01;

  Serial.println(F("Levitador v3 - PID en FPGA. El lazo arranca APAGADO: escribe 'on'. 'ayuda' para ver comandos."));
  delay(100);
  uint16_t v;
  if (comandoRegistro(0xA6, REG_SP, 0, v)) spCache = v;
}

void loop() {
  revisarSerialPC();

  if (millis() - tUltima < PERIODO_MS) return;
  tUltima = millis();

  // 1. Medicion (con filtro anti-picos)
  uint16_t pv = filtrar(medirMM());

  // 2. Enviar PV a la FPGA
  uint8_t t[4] = {0xAA, (uint8_t)(pv >> 8), (uint8_t)(pv & 0xFF), 0};
  t[3] = t[0] ^ t[1] ^ t[2];
  vaciarEntrada();
  fpgaSerial.write(t, 4);

  // 3. Recibir MV y STATUS
  uint8_t r[4];
  bool ok = leerBytes(r, 4, TIMEOUT_MS) && r[0] == 0x55 && ((r[0] ^ r[1] ^ r[2] ^ r[3]) == 0);

  if (ok) {
    fallos = 0;
    uint8_t mv = r[1];
    uint8_t st = r[2];
    analogWrite(PWM_PIN, mv);                 // 4. Aplicar control al ventilador

    if (modoPlot) {
      Serial.print(F("PV_mm:")); Serial.print(pv == PV_SIN_ECO ? 0 : pv);
      Serial.print(F(",SP_mm:")); Serial.print(spCache);
      Serial.print(F(",MV:"));    Serial.println(mv);
    } else {
      Serial.print(F("PV: "));
      if (pv == PV_SIN_ECO) Serial.print(F("sin eco")); else { Serial.print(pv); Serial.print(F(" mm")); }
      Serial.print(F(" | SP: ")); Serial.print(spCache);
      Serial.print(F(" mm | MV: ")); Serial.println(mv);
      if (st != statusAnterior) imprimirStatus(st);
    }
    statusAnterior = st;
  } else {
    fallos++;
    if (!modoPlot) Serial.println(F("Sin respuesta valida de la FPGA"));
    if (fallos >= MAX_FALLOS) analogWrite(PWM_PIN, 0);   // seguridad
  }

  // 5. Si hay una lectura de registros en curso, leer UNO por ciclo
  //    (no interrumpe el control)
  if (leerIdx >= 0) leerSiguiente();
}
