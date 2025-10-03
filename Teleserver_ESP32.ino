/*
  ESP32 + CoAP simple library (cliente) + POST manual
  - Serial: 9600
  - Cada 5 s:
      1) Sonda UDP "HELLO_FROM_ESP32"
      2) CoAP PUT "echo" (librería)
      3) CoAP POST "echo" (paquete construido a mano, con Content-Format text/plain)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>   // Hirotaka Niisato

// ====== CONFIG ======
#define WIFI_SSID   "Cuama"
#define WIFI_PASS   "boxman12"

#define SERVER_IP   "13.222.158.168"
#define SERVER_PORT 5683

#define LOCAL_UDP_PORT 45863
const char* MSG_TEXT = "MENSAJE_DE_PRUEBA_ESP32";
// =====================

WiFiUDP udp;
Coap coap(udp);
unsigned long lastSend = 0;
static uint16_t nextMessageId = 1;

// ---------- Callbacks ----------
void onCoapResponse(CoapPacket &packet, IPAddress ip, int port) {
  Serial.print("Respuesta CoAP desde ");
  Serial.print(ip); Serial.print(":"); Serial.println(port);
  Serial.printf("Code: %d.%02d\n", packet.code >> 5, packet.code & 0x1F);
  Serial.print("Payload: ");
  for (int i = 0; i < packet.payloadlen; i++) Serial.print((char)packet.payload[i]);
  Serial.println();
}

// ---------- Utilidades ----------
void hexdump(const uint8_t* p, int n) {
  for (int i = 0; i < n; ++i) {
    if (i && (i % 16) == 0) Serial.println();
    Serial.printf("%02X ", p[i]);
  }
  Serial.println();
}

bool sendUDP(const uint8_t* buf, int len) {
  IPAddress dst; dst.fromString(String(SERVER_IP));
  if (!udp.beginPacket(dst, SERVER_PORT)) return false;
  int w = udp.write(buf, len);
  bool ok = udp.endPacket();
  Serial.printf("TX UDP -> %s:%d bytes=%d\n", SERVER_IP, SERVER_PORT, w);
  return ok && (w == len);
}

void sendProbeUDP() {
  const char* probe = "HELLO_FROM_ESP32";
  Serial.println("[SONDA] UDP plano...");
  sendUDP((const uint8_t*)probe, strlen(probe));
}

// ---- CoAP helpers (POST manual /echo) ----
int writeOptionHeader(uint8_t* p, uint8_t delta, uint8_t length) {
  p[0] = (uint8_t)((delta << 4) | (length & 0x0F));
  return 1;
}
int addUriPath(uint8_t* p, uint16_t prevOptNum, const char* segment) {
  const uint16_t optNum = 11; // Uri-Path
  uint8_t segLen = (uint8_t)strlen(segment);
  uint8_t delta = (uint8_t)(optNum - prevOptNum);
  int n = 0;
  n += writeOptionHeader(p + n, delta, segLen);
  memcpy(p + n, segment, segLen);
  n += segLen;
  return n;
}
int buildCoapPostEcho(uint8_t* pkt, uint16_t mid, const char* payload) {
  int n = 0;
  pkt[n++] = 0x40;   // ver=1, type=CON, TKL=0
  pkt[n++] = 0x02;   // code=0.02 (POST)
  pkt[n++] = (uint8_t)((mid >> 8) & 0xFF);
  pkt[n++] = (uint8_t)( mid       & 0xFF);
  uint16_t prev = 0;
  n += addUriPath(pkt + n, prev, "echo"); prev = 11;
  // Content-Format (12): text/plain (0) => delta=1, len=1, value=0x00
  pkt[n++] = (uint8_t)((1 << 4) | 1);
  pkt[n++] = 0x00;
  // payload
  pkt[n++] = 0xFF;
  size_t plen = strlen(payload);
  memcpy(pkt + n, payload, plen);
  n += plen;
  return n;
}

void sendCoapPostEcho() {
  uint8_t pkt[256];
  uint16_t mid = nextMessageId++;
  int len = buildCoapPostEcho(pkt, mid, MSG_TEXT);

  Serial.printf("[COAP-MANUAL] POST /echo (MID=%u) payload=\"%s\"\n", mid, MSG_TEXT);
  Serial.println("[COAP-MANUAL] TX hexdump:");
  hexdump(pkt, len);

  if (sendUDP(pkt, len)) {
    // Espera 1 s por respuesta (opcional)
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      int p = udp.parsePacket();
      if (p > 0) {
        uint8_t rb[256]; int n = udp.read(rb, sizeof(rb));
        if (n >= 4) {
          uint8_t ver  = (rb[0] >> 6) & 0x03;
          uint8_t type = (rb[0] >> 4) & 0x03; // 2=ACK,3=RST
          uint8_t code = rb[1];
          uint16_t rmid = (uint16_t(rb[2])<<8) | rb[3];
          Serial.printf("[COAP-MANUAL RX] ver=%u type=%u code=0x%02X mid=%u len=%d\n",
                        ver, type, code, rmid, n);
          Serial.println("[COAP-MANUAL RX] hexdump:");
          hexdump(rb, n);
        }
        break;
      }
      delay(5);
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.println();
  Serial.print("OK WiFi. IP: "); Serial.println(WiFi.localIP());

  udp.begin(LOCAL_UDP_PORT);
  Serial.printf("UDP local en puerto %u\n", LOCAL_UDP_PORT);

  // CoAP (librería) para PUT
  coap.start();
  coap.response(onCoapResponse);
}

void loop() {
  coap.loop(); // procesa respuestas de la librería

  if (millis() - lastSend > 5000) {
    lastSend = millis();

    // 1) Sonda UDP
    sendProbeUDP();

    // 2) CoAP PUT (librería) -> por si tu server acepta PUT
    Serial.println("[COAP-LIB] PUT \"echo\" ...");
    coap.put(SERVER_IP, SERVER_PORT, "echo", MSG_TEXT);

    // 3) CoAP POST manual (lo que tu server probablemente espera)
    sendCoapPostEcho();
  }
}
