/*
  ESP32 → CoAP (POST JSON array a /echo) — Minimal, cohesivo y con bajo acoplamiento
  - Serial: 9600
  - Envío cada 15 s
  - JSON: [
      {"data":{"temperatura":x,"humedad":y,"voltaje":z,"cantidad_producida":N},"timestamp":ms_epoch},
      ...
    ]
  - CoAP: CON + POST /echo + Content-Format: application/json (50) + Token (TKL=2)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <string.h>

// ======= CONFIG =======
#define WIFI_SSID     "Cuama"  // cambiar a red local
#define WIFI_PASS     "boxman12" //contrasena local

#define SERVER_IP     "3.93.196.162"
#define SERVER_PORT   5683

#define SEND_EVERY_MS 15000UL  //tiempo de espera antes de volver a mandar mensaje
#define DEVICE_ID     "esp32-A"     // si quieres incluirlo en el JSON, puedes
#define BATCH_SIZE    3             // cantidad de lecturas por mensaje (>=1)
// ======================

// Estado mínimo
WiFiUDP g_udp;
static uint16_t g_nextMID  = 1;      // Message ID (CoAP)
static uint16_t g_nextTok  = 1;      // Token simple (2 bytes)
static uint32_t g_prodCnt  = 0;      // contador "cantidad_producida"

// ---------- Utilidades ----------
static float frand(float a, float b) {
  return a + (float)random(0, 10001) / 10000.0f * (b - a);
}

static bool wifi_connect(const char* ssid, const char* pass, uint32_t timeoutMs = 20000) {
  Serial.printf("Conectando a %s...\r\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    if (millis() - t0 > timeoutMs) {
      Serial.println("\nError WiFi");
      return false;
    }
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
  // Intento de hora real; si no hay NTP, habrá fallback a millis().
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  return true;
}

// epoch en milisegundos (con fallback si no hay NTP aún)
static uint64_t now_ms() {
  time_t s = time(nullptr);
  if (s < 100000) return (uint64_t)millis();     // aproximado desde arranque
  return ((uint64_t)s) * 1000ULL;
}

// ---------- JSON (arreglo con N lecturas) ----------
/* Construye:
   [
     {"data":{"temperatura":T,"humedad":H,"voltaje":V,"cantidad_producida":C},"timestamp":MS},
     ...
   ]
   Retorna longitud escrita. */
static int build_json_batch(char* out, size_t max, int count) {
  int n = 0;
  n += snprintf(out + n, (max > (size_t)n ? max - n : 0), "[");
  for (int i = 0; i < count; ++i) {
    // Simulación de sensores
    float temperatura = frand(22.5f, 30.0f);
    float humedad     = frand(35.0f, 75.0f);
    float voltaje     = frand(3.5f, 4.2f);
    g_prodCnt += (uint32_t)frand(5.0f, 20.0f);   // incremento de producción
    uint64_t ts = now_ms();

    n += snprintf(out + n, (max > (size_t)n ? max - n : 0),
      "%s{\"data\":{\"temperatura\":%.1f,\"humedad\":%.1f,\"voltaje\":%.2f,"
      "\"cantidad_producida\":%lu},\"timestamp\":%llu}",
      (i==0 ? "" : ","),
      temperatura, humedad, voltaje,
      (unsigned long)g_prodCnt,
      (unsigned long long)ts
    );
    if ((size_t)n >= max) { n = (int)max - 1; break; }
  }
  n += snprintf(out + n, (max > (size_t)n ? max - n : 0), "]");
  if (n < 0) n = 0;
  if ((size_t)n >= max) n = (int)max - 1;
  return n;
}

// ---------- CoAP (construcción de paquete) ----------
static int coap_write_opt(uint8_t* p, uint8_t delta, uint8_t length) {
  p[0] = (uint8_t)((delta << 4) | (length & 0x0F));
  return 1;
}

static int coap_add_uri_path(uint8_t* p, uint16_t prevOptNum, const char* seg) {
  const uint16_t opt = 11;       // Uri-Path
  const uint8_t  L   = (uint8_t)strlen(seg);
  const uint8_t  D   = (uint8_t)(opt - prevOptNum);
  int n = 0;
  n += coap_write_opt(p + n, D, L);
  memcpy(p + n, seg, L);
  return n + L;
}

/* POST(CON) /echo
   - TKL=2 (token de 2 bytes)
   - Content-Format = application/json (50)
   - payload = json[] (sin '\0') */
static int coap_build_post_echo_json(uint8_t* pkt, uint16_t mid, uint16_t token,
                                     const char* json, size_t jsonLen) {
  int n = 0;
  pkt[n++] = 0x42;            // ver=1, type=CON, TKL=2
  pkt[n++] = 0x02;            // POST
  pkt[n++] = (uint8_t)(mid >> 8);
  pkt[n++] = (uint8_t)(mid & 0xFF);

  // Token (2 bytes)
  pkt[n++] = (uint8_t)(token >> 8);
  pkt[n++] = (uint8_t)(token & 0xFF);

  // Uri-Path: "echo"
  uint16_t prev = 0;
  n += coap_add_uri_path(pkt + n, prev, "echo"); prev = 11;

  // Content-Format (12) = 50 (application/json). delta=1, len=1
  pkt[n++] = (uint8_t)((1 << 4) | 1);
  pkt[n++] = 0x32;

  // Payload
  pkt[n++] = 0xFF;
  memcpy(pkt + n, json, jsonLen);
  return n + jsonLen;
}

// ---------- Transporte (UDP + espera ACK/RST) ----------
static bool udp_send(const uint8_t* buf, int len, const char* ipStr, uint16_t port) {
  IPAddress ip; ip.fromString(String(ipStr));
  if (!g_udp.beginPacket(ip, port)) return false;
  int w = g_udp.write(buf, len);
  bool ok = g_udp.endPacket();
  Serial.printf("TX %d bytes -> %s:%u\n", w, ipStr, port);
  return ok && (w == len);
}

/* Espera respuesta con MID esperado. Retorna:
   2 (ACK), 3 (RST) o -1 (timeout). outCode recibe el code. */
static int coap_wait_ack(uint16_t expectMID, uint32_t timeoutMs, uint8_t* outCode) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    int p = g_udp.parsePacket();
    if (p > 0) {
      uint8_t rb[256]; int n = g_udp.read(rb, sizeof(rb));
      if (n >= 4) {
        uint8_t ver  = (rb[0] >> 6) & 0x03;
        uint8_t type = (rb[0] >> 4) & 0x03;
        uint8_t code =  rb[1];
        uint16_t mid  = (uint16_t(rb[2]) << 8) | rb[3];
        if (ver == 1 && mid == expectMID) {
          if (outCode) *outCode = code;
          return type; // 2=ACK, 3=RST
        }
      }
    }
    delay(5);
  }
  return -1;
}

// ---------- Programa ----------
void setup() {
  Serial.begin(9600);
  delay(200);

  randomSeed(micros());
  wifi_connect(WIFI_SSID, WIFI_PASS);

  // Puerto local efímero
  g_udp.begin(0);
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < SEND_EVERY_MS) return;
  last = now;

  // 1) JSON array
  char json[512];
  int jlen = build_json_batch(json, sizeof(json), BATCH_SIZE);
  Serial.print("JSON: "); Serial.println(json);

  // 2) CoAP POST /echo
  uint8_t pkt[768];
  uint16_t mid = g_nextMID++;
  uint16_t tok = g_nextTok++;          // TKL=2
  int len = coap_build_post_echo_json(pkt, mid, tok, json, (size_t)jlen);

  if (!udp_send(pkt, len, SERVER_IP, SERVER_PORT)) {
    Serial.println("Error de envío UDP");
    return;
  }

  // 3) Espera ACK/RST (1 s)
  uint8_t code = 0;
  int t = coap_wait_ack(mid, 1000, &code);
  if (t == 2) {
    Serial.printf("ACK. code=0x%02X MID=%u\n", code, mid);
  } else if (t == 3) {
    Serial.printf("RST. code=0x%02X MID=%u\n", code, mid);
  } else if (t >= 0) {
    Serial.printf("Resp no esperada. type=%d code=0x%02X MID=%u\n", t, code, mid);
  } else {
    Serial.println("Sin respuesta (timeout)");
  }
}
