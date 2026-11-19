/*
  ESP32-S3 CAM + Gemini AI + TFT ST7789 240x240 + Keyboard USB
  ------------------------------------------------------------
  WIRING (board ESP32-S3 CAM / Freenove N16R8, kamera OV2640 bawaan)

  TFT ST7789 240x240 (tanpa pin CS):
    GND -> GND
    VCC -> 3V3
    SCK -> GPIO 21
    SDA -> GPIO 47
    RES -> GPIO 42
    DC  -> GPIO 41
    BLK -> 3V3 (selalu nyala)

  Keyboard USB (potong kabel / pakai socket USB-A female):
    Merah  (VBUS 5V) -> 5V
    Hitam  (GND)     -> GND
    Putih  (D-)      -> GPIO 19
    Hijau  (D+)      -> GPIO 20

  Upload lewat port USB-UART (COM) seperti biasa.

  CARA PAKAI
    1. Sambungkan HP/laptop ke WiFi  "ESP32-AI"  password "12345678"
    2. Buka http://192.168.4.1  -> isi WiFi rumah, API key Gemini, pilih model -> Simpan
    3. Ketik soal di keyboard, tekan ENTER untuk kirim
    4. Panah Atas/Bawah = scroll, PageUp/PageDown = scroll cepat, Home/End
    5. Tombol "~" (Shift + `) = ambil foto (tampil 6 detik), lalu ketik pertanyaan + ENTER
       (ENTER kosong = AI langsung menyelesaikan soal di foto)
    6. ESC = batalkan foto / hapus ketikan, F5 = hapus riwayat percakapan
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduinoJson.h>
#include <vector>
#include "esp_camera.h"
#include "img_converters.h"
#include "mbedtls/base64.h"
#include "usb/usb_host.h"

// ---------------- PIN ----------------
#define TFT_SCK   21
#define TFT_MOSI  47
#define TFT_RST   42
#define TFT_DC    41
#define TFT_CS    -1

// Kamera (pinout Freenove ESP32-S3 WROOM CAM / ESP32-S3-EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#define AP_SSID "ESP32-AI"
#define AP_PASS "12345678"
#define PHOTO_SHOW_MS 6000

// ---------------- Objek ----------------
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 *canvas = nullptr;
WebServer server(80);
Preferences prefs;

String cfgSsid, cfgPass, cfgKey, cfgModel;
bool camOk = false;

struct Line { String t; uint16_t c; };
std::vector<Line> lines;
int scrollTop = 0;
const int COLS = 39;
const int ANS_Y = 14, ANS_LINES = 17, LINE_H = 10;
const int IN_Y = 196, IN_LINES = 4;

String inputBuf;
String status = "";
uint8_t *photoJpg = nullptr; size_t photoLen = 0;
bool busy = false;

struct Turn { String role; String text; };
std::vector<Turn> history;

// keyboard queue
QueueHandle_t keyQ;
enum { K_ENTER = 0x100, K_BKSP, K_ESC, K_UP, K_DOWN, K_PGUP, K_PGDN, K_HOME, K_END, K_F5 };

// =====================================================================
//                         UTF-8 -> CP437 (simbol matematika)
// =====================================================================
const char *mapCp(uint32_t cp) {
  switch (cp) {
    case 0x221A: return "\xFB"; case 0xB2: return "\xFD"; case 0xB3: return "^3";
    case 0xB9: return "^1"; case 0x2070: return "^0"; case 0x207F: return "\xFC";
    case 0x3C0: return "\xE3"; case 0xB1: return "\xF1"; case 0x2265: return "\xF2";
    case 0x2264: return "\xF3"; case 0xF7: return "\xF6"; case 0xB0: return "\xF8";
    case 0x221E: return "\xEC"; case 0x3A3: case 0x2211: return "\xE4";
    case 0x3B1: return "\xE0"; case 0x3B2: return "\xE1"; case 0x3B3: return "y";
    case 0x393: return "\xE2"; case 0x3B4: return "\xEB"; case 0x394: case 0x2206: return "\x1E";
    case 0x3B5: case 0x2208: return "\xEE"; case 0x3B8: case 0x398: return "\xE9";
    case 0x3BC: case 0xB5: return "\xE6"; case 0x3A9: case 0x2126: return "\xEA";
    case 0x3C6: return "\xED"; case 0x3A6: return "\xE8"; case 0x3C3: return "\xE5";
    case 0x3C4: return "\xE7"; case 0x3BB: return "lambda"; case 0x3C9: return "w";
    case 0x3C1: return "p"; case 0x2248: return "\xF7"; case 0x2261: return "\xF0";
    case 0x2260: return "!="; case 0xD7: return "x"; case 0x22C5: case 0xB7: case 0x2219: return "\xFA";
    case 0x2022: return "\x07"; case 0x2229: return "\xEF"; case 0x222A: return "U";
    case 0x222B: return "\xF4"; case 0x2202: return "d"; case 0x2192: return "->";
    case 0x2190: return "<-"; case 0x21D2: return "=>"; case 0x21D4: return "<=>";
    case 0x2194: return "<->"; case 0x2013: case 0x2014: case 0x2212: return "-";
    case 0x2018: case 0x2019: return "'"; case 0x201C: case 0x201D: return "\"";
    case 0x2026: return "..."; case 0x2220: return "<"; case 0x22A5: return "_|_";
    case 0x2225: return "||"; case 0x2200: return "A"; case 0x2203: return "E";
    case 0x2234: return ".:."; case 0xBD: return "\xAB"; case 0xBC: return "\xAC";
    case 0xE9: return "\x82"; case 0x2245: return "~="; case 0x223C: return "~";
    case 0xAC: return "\xAA"; case 0x221D: return "~"; case 0x2032: return "'";
    case 0xA0: return " ";
  }
  return nullptr;
}

String toCp437(const String &s) {
  String o; o.reserve(s.length());
  const uint8_t *p = (const uint8_t *)s.c_str(); size_t n = s.length(), i = 0;
  while (i < n) {
    uint32_t cp; uint8_t c = p[i];
    if (c < 0x80) { cp = c; i++; }
    else if ((c & 0xE0) == 0xC0 && i + 1 < n) { cp = ((c & 0x1F) << 6) | (p[i+1] & 0x3F); i += 2; }
    else if ((c & 0xF0) == 0xE0 && i + 2 < n) { cp = ((c & 0x0F) << 12) | ((p[i+1] & 0x3F) << 6) | (p[i+2] & 0x3F); i += 3; }
    else if ((c & 0xF8) == 0xF0 && i + 3 < n) { cp = ((c & 0x07) << 18) | ((p[i+1] & 0x3F) << 12) | ((p[i+2] & 0x3F) << 6) | (p[i+3] & 0x3F); i += 4; }
    else { i++; continue; }
    if (cp < 0x80) { if (cp == '\n' || cp >= 0x20) o += (char)cp; else if (cp == '\t') o += "  "; continue; }
    if (cp >= 0x2074 && cp <= 0x2079) { o += '^'; o += (char)('0' + cp - 0x2070); continue; }
    if (cp >= 0x2080 && cp <= 0x2089) { o += '_'; o += (char)('0' + cp - 0x2080); continue; }
    const char *m = mapCp(cp);
    if (m) o += m; else if (cp >= 0x1F000) {} else o += '?';
  }
  return o;
}

// ambil isi {...} seimbang mulai dari posisi '{'
int braceEnd(const String &s, int open) {
  int d = 0;
  for (int i = open; i < (int)s.length(); i++) {
    if (s[i] == '{') d++; else if (s[i] == '}') { d--; if (d == 0) return i; }
  }
  return -1;
}

void cleanLatex(String &s) {
  s.replace("\\left", ""); s.replace("\\right", "");
  s.replace("\\dfrac", "\\frac"); s.replace("\\tfrac", "\\frac");
  int p;
  while ((p = s.indexOf("\\frac{")) >= 0) {
    int a = p + 5, ae = braceEnd(s, a); if (ae < 0 || ae + 1 >= (int)s.length() || s[ae+1] != '{') { s.remove(p, 5); continue; }
    int be = braceEnd(s, ae + 1); if (be < 0) { s.remove(p, 5); continue; }
    String A = s.substring(a + 1, ae), B = s.substring(ae + 2, be);
    s = s.substring(0, p) + "(" + A + ")/(" + B + ")" + s.substring(be + 1);
  }
  while ((p = s.indexOf("\\sqrt{")) >= 0) {
    int e = braceEnd(s, p + 5); if (e < 0) { s.remove(p, 5); continue; }
    s = s.substring(0, p) + "\xE2\x88\x9A(" + s.substring(p + 6, e) + ")" + s.substring(e + 1);
  }
  const char *rep[][2] = {
    {"\\times","\xC3\x97"},{"\\cdot","\xC2\xB7"},{"\\pm","\xC2\xB1"},{"\\leq","\xE2\x89\xA4"},{"\\geq","\xE2\x89\xA5"},
    {"\\le","\xE2\x89\xA4"},{"\\ge","\xE2\x89\xA5"},{"\\neq","!="},{"\\ne","!="},{"\\pi","\xCF\x80"},{"\\theta","\xCE\xB8"},
    {"\\alpha","\xCE\xB1"},{"\\beta","\xCE\xB2"},{"\\Delta","\xCE\x94"},{"\\delta","\xCE\xB4"},{"\\infty","\xE2\x88\x9E"},
    {"\\sum","\xCE\xA3"},{"\\int","\xE2\x88\xAB"},{"\\approx","\xE2\x89\x88"},{"\\div","\xC3\xB7"},{"^\\circ","\xC2\xB0"},
    {"\\circ","\xC2\xB0"},{"\\rightarrow","->"},{"\\Rightarrow","=>"},{"\\to","->"},{"\\lambda","lambda"},{"\\mu","\xCE\xBC"},
    {"\\sigma","\xCF\x83"},{"\\omega","w"},{"\\Omega","\xCE\xA9"},{"\\phi","\xCF\x86"},{"\\text",""},{"\\mathrm",""},
    {"\\quad","  "},{"\\,"," "},{"\\;"," "},{"\\!",""},{"\\\\","\n"},{"\\%","%"},{"$$",""},{"$",""}
  };
  for (auto &r : rep) s.replace(r[0], r[1]);
  s.replace("^{", "^("); s.replace("_{", "_(");
  s.replace("{", ""); s.replace("}", "");
}

String cleanMarkdown(const String &in) {
  String out; int start = 0;
  while (start <= (int)in.length()) {
    int nl = in.indexOf('\n', start); if (nl < 0) nl = in.length();
    String l = in.substring(start, nl);
    String t = l; t.trim();
    while (t.startsWith("#")) t.remove(0, 1);
    if (l.startsWith("#")) { t.trim(); l = t; }
    if (l.startsWith("* ") || l.startsWith("- ")) l = "\xE2\x80\xA2 " + l.substring(2);
    if (l.startsWith("  * ")) l = "   - " + l.substring(4);
    if (l == "---" || l == "***") l = "----------------------------------------";
    out += l; if (nl < (int)in.length()) out += '\n';
    start = nl + 1;
  }
  out.replace("**", ""); out.replace("__", ""); out.replace("`", "");
  return out;
}

// =====================================================================
//                               TAMPILAN
// =====================================================================
void addWrapped(const String &txtCp, uint16_t color) {
  int start = 0, n = txtCp.length();
  while (start <= n) {
    int nl = txtCp.indexOf('\n', start); if (nl < 0) nl = n;
    String para = txtCp.substring(start, nl);
    if (para.length() == 0) lines.push_back({"", color});
    while (para.length() > 0) {
      if ((int)para.length() <= COLS) { lines.push_back({para, color}); break; }
      int cut = para.lastIndexOf(' ', COLS);
      if (cut <= 0) cut = COLS;
      lines.push_back({para.substring(0, cut), color});
      para = para.substring(cut); if (para.startsWith(" ")) para.remove(0, 1);
    }
    start = nl + 1;
  }
}

int maxScroll() { return max(0, (int)lines.size() - ANS_LINES); }

void render() {
  GFXcanvas16 &c = *canvas;
  c.fillScreen(ST77XX_BLACK);
  // header
  c.fillRect(0, 0, 240, 12, 0x18E3);
  c.setTextSize(1); c.setTextColor(ST77XX_YELLOW); c.setCursor(2, 2);
  String m = cfgModel; m.replace("gemini-", "g-"); if (m.length() > 16) m = m.substring(0, 16);
  c.print(m);
  c.setCursor(110, 2);
  c.setTextColor(WiFi.status() == WL_CONNECTED ? ST77XX_GREEN : ST77XX_RED);
  c.print(WiFi.status() == WL_CONNECTED ? "WiFi" : "noWiFi");
  if (photoJpg) { c.setTextColor(ST77XX_MAGENTA); c.setCursor(150, 2); c.print("[FOTO]"); }
  c.setTextColor(ST77XX_WHITE); c.setCursor(192, 2);
  if (lines.size()) { int pct = maxScroll() ? scrollTop * 100 / maxScroll() : 100; c.print(pct); c.print('%'); }

  // jawaban
  for (int i = 0; i < ANS_LINES; i++) {
    int idx = scrollTop + i; if (idx >= (int)lines.size()) break;
    c.setTextColor(lines[idx].c); c.setCursor(1, ANS_Y + i * LINE_H); c.print(lines[idx].t);
  }
  // scrollbar
  if ((int)lines.size() > ANS_LINES) {
    int h = ANS_LINES * LINE_H;
    int bh = max(8, h * ANS_LINES / (int)lines.size());
    int by = ANS_Y + (h - bh) * scrollTop / max(1, maxScroll());
    c.fillRect(236, ANS_Y, 4, h, 0x2104); c.fillRect(236, by, 4, bh, 0x7BEF);
  }
  // input
  c.drawFastHLine(0, IN_Y - 3, 240, 0x39E7);
  if (busy || status.length()) {
    c.setTextColor(ST77XX_ORANGE); c.setCursor(2, IN_Y); c.print(status);
  } else {
    String s = "> " + toCp437(inputBuf) + "_";
    std::vector<String> w;
    for (int i = 0; i < (int)s.length(); i += 40) w.push_back(s.substring(i, i + 40));
    int st = max(0, (int)w.size() - IN_LINES);
    c.setTextColor(ST77XX_CYAN);
    for (int i = st; i < (int)w.size(); i++) { c.setCursor(0, IN_Y + (i - st) * LINE_H); c.print(w[i]); }
    if (inputBuf.length() == 0 && !photoJpg && lines.empty()) {
      c.setTextColor(0x7BEF); c.setCursor(0, IN_Y + 12); c.print("Ketik soal + ENTER.  ~ = foto");
    }
  }
  tft.drawRGBBitmap(0, 0, c.getBuffer(), 240, 240);
}

void setStatus(const String &s) { status = s; render(); }

// =====================================================================
//                               KAMERA
// =====================================================================
bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0; cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM; cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM; cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM; cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM; cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size = FRAMESIZE_XGA;      // 1024x768 agar tulisan soal terbaca AI
  cfg.jpeg_quality = 12;
  cfg.fb_count = 2;
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&cfg) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  if (s) { s->set_brightness(s, 1); s->set_saturation(s, 0); }
  return true;
}

void takePhoto() {
  if (!camOk) { setStatus("Kamera tidak terdeteksi!"); delay(1500); setStatus(""); return; }
  setStatus("Mengambil foto...");
  camera_fb_t *fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); // buang frame lama
  fb = esp_camera_fb_get();
  if (!fb) { setStatus("Gagal foto"); delay(1500); setStatus(""); return; }
  if (photoJpg) { free(photoJpg); photoJpg = nullptr; }
  photoJpg = (uint8_t *)ps_malloc(fb->len);
  if (photoJpg) { memcpy(photoJpg, fb->buf, fb->len); photoLen = fb->len; }
  esp_camera_fb_return(fb);

  // tampilkan preview (1024x768 /4 = 256x192)
  const int W = 256, H = 192;
  uint8_t *rgb = (uint8_t *)ps_malloc(W * H * 2);
  tft.fillScreen(ST77XX_BLACK);
  if (rgb && photoJpg && jpg2rgb565(photoJpg, photoLen, rgb, JPG_SCALE_4X)) {
    tft.startWrite();
    int y0 = (240 - H) / 2;
    for (int y = 0; y < H; y++) {
      tft.setAddrWindow(0, y0 + y, 240, 1);
      tft.writePixels((uint16_t *)(rgb + (y * W + 8) * 2), 240, true, true);
    }
    tft.endWrite();
  }
  if (rgb) free(rgb);
  tft.setTextSize(1); tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  unsigned long t0 = millis();
  while (millis() - t0 < PHOTO_SHOW_MS) {
    tft.setCursor(4, 4); tft.print("FOTO - ");
    tft.print((PHOTO_SHOW_MS - (millis() - t0)) / 1000 + 1); tft.print(" dtk ");
    tft.setCursor(4, 228); tft.print("ENTER kirim / ketik pertanyaan");
    server.handleClient(); delay(100);
  }
  xQueueReset(keyQ);
  status = "";
  render();
}

// =====================================================================
//                               GEMINI
// =====================================================================
String jsonEsc(const String &s) {
  String o; o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') o += "\\\""; else if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n"; else if (c == '\r') {} else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

const char *SYS_PROMPT =
  "Kamu asisten AI pintar di perangkat kecil dengan layar teks 39 kolom. "
  "Jawab dalam Bahasa Indonesia (kecuali diminta lain), lengkap, teliti, dan langkah demi langkah untuk soal rumit "
  "(matematika, fisika, kimia, logika, pemrograman). Periksa ulang hitunganmu dan tulis JAWABAN AKHIR dengan jelas. "
  "JANGAN gunakan LaTeX atau tabel markdown. Tulis rumus dalam teks biasa memakai simbol Unicode seperti "
  "√ ² π ∞ ≤ ≥ ≠ ± × ÷ ≈ Σ ∫ θ α β Δ ° dan pecahan sebagai (a)/(b), pangkat sebagai x^3. "
  "Jika ada foto, baca soal di foto dengan cermat lalu selesaikan.";

String askGemini(const String &question, bool withPhoto, String &err) {
  if (cfgKey.length() < 10) { err = "API key belum diisi. Buka web 192.168.4.1"; return ""; }
  if (WiFi.status() != WL_CONNECTED) { err = "WiFi belum terhubung"; return ""; }

  String body = "{\"systemInstruction\":{\"parts\":[{\"text\":\"" + jsonEsc(SYS_PROMPT) + "\"}]},\"contents\":[";
  for (auto &t : history)
    body += "{\"role\":\"" + t.role + "\",\"parts\":[{\"text\":\"" + jsonEsc(t.text) + "\"}]},";
  body += "{\"role\":\"user\",\"parts\":[";
  if (withPhoto && photoJpg) {
    size_t olen = 0;
    mbedtls_base64_encode(nullptr, 0, &olen, photoJpg, photoLen);
    char *b64 = (char *)ps_malloc(olen + 1);
    if (!b64) { err = "Memori tidak cukup"; return ""; }
    mbedtls_base64_encode((unsigned char *)b64, olen + 1, &olen, photoJpg, photoLen); b64[olen] = 0;
    body.reserve(body.length() + olen + 4096);
    body += "{\"inline_data\":{\"mime_type\":\"image/jpeg\",\"data\":\"";
    body += b64; free(b64);
    body += "\"}},";
  }
  body += "{\"text\":\"" + jsonEsc(question) + "\"}]}],";
  body += "\"generationConfig\":{\"maxOutputTokens\":8192}}";

  WiFiClientSecure cli; cli.setInsecure();
  HTTPClient http;
  String url = "https://generativelanguage.googleapis.com/v1beta/models/" + cfgModel + ":generateContent";
  http.begin(cli, url);
  http.setTimeout(180000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-goog-api-key", cfgKey);
  int code = http.POST((uint8_t *)body.c_str(), body.length());
  body = String();
  String resp = http.getString();
  http.end();

  JsonDocument filter;
  filter["candidates"][0]["content"]["parts"][0]["text"] = true;
  filter["candidates"][0]["content"]["parts"][0]["thought"] = true;
  filter["candidates"][0]["finishReason"] = true;
  filter["error"]["message"] = true;
  filter["promptFeedback"]["blockReason"] = true;
  JsonDocument doc;
  DeserializationError de = deserializeJson(doc, resp, DeserializationOption::Filter(filter));
  if (code != 200) {
    err = "HTTP " + String(code) + ": ";
    if (!de && doc["error"]["message"].is<const char *>()) err += doc["error"]["message"].as<String>();
    else err += resp.substring(0, 200);
    return "";
  }
  if (de) { err = "JSON error: " + String(de.c_str()); return ""; }
  String out;
  for (JsonObject p : doc["candidates"][0]["content"]["parts"].as<JsonArray>()) {
    if (p["thought"] | false) continue;
    out += p["text"].as<String>();
  }
  if (out.length() == 0) {
    err = "Jawaban kosong. ";
    if (doc["promptFeedback"]["blockReason"]) err += "Diblokir: " + doc["promptFeedback"]["blockReason"].as<String>();
    else err += "Alasan: " + doc["candidates"][0]["finishReason"].as<String>();
  }
  if (doc["candidates"][0]["finishReason"] == "MAX_TOKENS") out += "\n\n[jawaban terpotong - batas token]";
  return out;
}

void submit() {
  String q = inputBuf; q.trim();
  bool withPhoto = photoJpg != nullptr;
  if (q.length() == 0 && !withPhoto) return;
  if (q.length() == 0) q = "Baca dan selesaikan soal pada foto ini secara lengkap langkah demi langkah.";

  if (lines.size()) lines.push_back({"", ST77XX_WHITE});
  addWrapped(toCp437(String(withPhoto ? "[foto] " : "") + "> " + q), ST77XX_CYAN);
  inputBuf = "";
  scrollTop = maxScroll();
  busy = true; setStatus("AI sedang berpikir...");

  String err;
  String ans = askGemini(q, withPhoto, err);
  busy = false; status = "";
  int firstAnsLine = lines.size();
  if (ans.length()) {
    history.push_back({"user", (withPhoto ? "[pengguna mengirim foto] " : "") + q});
    history.push_back({"model", ans});
    while (history.size() > 8) history.erase(history.begin());
    String t = cleanMarkdown(ans); cleanLatex(t);
    addWrapped(toCp437(t), ST77XX_WHITE);
  } else {
    addWrapped(toCp437("ERROR: " + err), ST77XX_RED);
  }
  if (withPhoto) { free(photoJpg); photoJpg = nullptr; photoLen = 0; }
  scrollTop = min(max(0, firstAnsLine - 1), maxScroll());
  while (lines.size() > 3000) { lines.erase(lines.begin()); scrollTop = max(0, scrollTop - 1); }
  render();
}

// =====================================================================
//                              WEB CONFIG
// =====================================================================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ESP32 Gemini AI</title>
<style>body{font-family:sans-serif;background:#0f172a;color:#e2e8f0;max-width:460px;margin:auto;padding:16px}
h2{color:#38bdf8}label{display:block;margin-top:12px;font-size:14px}input,select{width:100%;padding:10px;margin-top:4px;
border-radius:8px;border:1px solid #334155;background:#1e293b;color:#fff;box-sizing:border-box}
button{margin-top:16px;width:100%;padding:12px;border:0;border-radius:8px;background:#0ea5e9;color:#fff;font-size:16px}
.s{font-size:12px;color:#94a3b8}</style></head><body>
<h2>ESP32-S3 Gemini AI</h2><p class="s">Status: %STATUS%</p>
<form method="POST" action="/save">
<label>WiFi SSID</label><input name="ssid" value="%SSID%" list="nets"><datalist id="nets">%NETS%</datalist>
<label>WiFi Password</label><input name="pass" type="password" placeholder="(kosongkan = tidak diubah)">
<label>Gemini API Key</label><input name="key" type="password" placeholder="%KEYHINT%">
<p class="s">Dapatkan di aistudio.google.com/apikey</p>
<label>Model</label><select name="model" id="model">%OPTS%</select>
<button type="button" onclick="lm()" style="background:#475569">Ambil daftar model dari API key</button>
<label>Atau ketik nama model manual</label><input name="custom" placeholder="contoh: gemini-2.5-pro">
<button type="submit">Simpan &amp; Restart</button></form>
<script>function lm(){fetch('/models').then(r=>r.json()).then(a=>{if(!a.length){alert('Gagal / API key salah / belum WiFi');return}
let s=document.getElementById('model'),c=s.value;s.innerHTML='';a.forEach(m=>{let o=document.createElement('option');o.text=m;if(m==c)o.selected=1;s.add(o)})})}</script>
</body></html>)HTML";

const char *MODELS[] = {"gemini-2.5-flash", "gemini-2.5-pro", "gemini-2.5-flash-lite", "gemini-2.0-flash", "gemini-flash-latest", "gemini-pro-latest"};

void handleRoot() {
  String p = FPSTR(PAGE);
  String st = WiFi.status() == WL_CONNECTED ? "Terhubung ke " + WiFi.SSID() + " IP " + WiFi.localIP().toString() : "Belum terhubung WiFi";
  p.replace("%STATUS%", st);
  p.replace("%SSID%", cfgSsid);
  p.replace("%KEYHINT%", cfgKey.length() ? "(tersimpan, kosongkan = tidak diubah)" : "AIza...");
  String opts; bool found = false;
  for (auto m : MODELS) { opts += "<option"; if (cfgModel == m) { opts += " selected"; found = true; } opts += ">" + String(m) + "</option>"; }
  if (!found) opts = "<option selected>" + cfgModel + "</option>" + opts;
  p.replace("%OPTS%", opts);
  String nets; int n = WiFi.scanComplete();
  if (n > 0) for (int i = 0; i < n; i++) nets += "<option value=\"" + WiFi.SSID(i) + "\">";
  if (n == -2) WiFi.scanNetworks(true);
  p.replace("%NETS%", nets);
  server.send(200, "text/html", p);
}

void handleSave() {
  prefs.begin("ai", false);
  if (server.hasArg("ssid")) prefs.putString("ssid", server.arg("ssid"));
  if (server.arg("pass").length()) prefs.putString("pass", server.arg("pass"));
  if (server.arg("key").length()) { String k = server.arg("key"); k.trim(); prefs.putString("key", k); }
  String m = server.arg("custom"); m.trim(); if (!m.length()) m = server.arg("model");
  m.replace("models/", ""); if (m.length()) prefs.putString("model", m);
  prefs.end();
  server.send(200, "text/html", "<meta charset='utf-8'><body style='font-family:sans-serif'><h3>Tersimpan! ESP32 restart...</h3></body>");
  delay(800); ESP.restart();
}

void handleModels() {
  String out = "[";
  if (WiFi.status() == WL_CONNECTED && cfgKey.length()) {
    WiFiClientSecure cli; cli.setInsecure(); HTTPClient http;
    http.begin(cli, "https://generativelanguage.googleapis.com/v1beta/models?pageSize=200");
    http.addHeader("x-goog-api-key", cfgKey);
    if (http.GET() == 200) {
      JsonDocument f; f["models"][0]["name"] = true; f["models"][0]["supportedGenerationMethods"] = true;
      JsonDocument d; deserializeJson(d, http.getString(), DeserializationOption::Filter(f));
      bool first = true;
      for (JsonObject m : d["models"].as<JsonArray>()) {
        bool ok = false;
        for (JsonVariant g : m["supportedGenerationMethods"].as<JsonArray>()) if (g == "generateContent") ok = true;
        String nm = m["name"].as<String>(); nm.replace("models/", "");
        if (!ok || nm.indexOf("gemini") < 0 || nm.indexOf("tts") >= 0 || nm.indexOf("image") >= 0) continue;
        if (!first) out += ","; out += "\"" + nm + "\""; first = false;
      }
    }
    http.end();
  }
  server.send(200, "application/json", out + "]");
}

// =====================================================================
//                          USB HOST KEYBOARD
// =====================================================================
static usb_host_client_handle_t clientHdl = nullptr;
static usb_device_handle_t devHdl = nullptr;
static volatile uint8_t newDevAddr = 0;
static volatile bool devGone = false;
static usb_transfer_t *kbXfer = nullptr, *ctrlXfer = nullptr;
static uint8_t kbIface = 0;
static bool kbClaimed = false;
static uint8_t prevKeys[6] = {0};
static volatile uint16_t heldKey = 0; static volatile uint32_t heldSince = 0;

const char hidLower[] = "abcdefghijklmnopqrstuvwxyz1234567890";
const char hidUpperNum[] = "!@#$%^&*()";

void pushKey(uint16_t k) { xQueueSend(keyQ, &k, 0); }

uint16_t hidToKey(uint8_t code, uint8_t mod) {
  bool sh = mod & 0x22;
  if (code >= 0x04 && code <= 0x1D) { char c = 'a' + code - 4; return sh ? c - 32 : c; }
  if (code >= 0x1E && code <= 0x27) { int i = code - 0x1E; return sh ? hidUpperNum[i] : hidLower[26 + i]; }
  switch (code) {
    case 0x28: case 0x58: return K_ENTER; case 0x29: return K_ESC; case 0x2A: return K_BKSP;
    case 0x2B: return ' '; case 0x2C: return ' ';
    case 0x2D: return sh ? '_' : '-'; case 0x2E: return sh ? '+' : '=';
    case 0x2F: return sh ? '{' : '['; case 0x30: return sh ? '}' : ']';
    case 0x31: return sh ? '|' : '\\'; case 0x33: return sh ? ':' : ';';
    case 0x34: return sh ? '"' : '\''; case 0x35: return sh ? '~' : '`';
    case 0x36: return sh ? '<' : ','; case 0x37: return sh ? '>' : '.';
    case 0x38: return sh ? '?' : '/';
    case 0x52: return K_UP; case 0x51: return K_DOWN; case 0x4B: return K_PGUP; case 0x4E: return K_PGDN;
    case 0x4A: return K_HOME; case 0x4D: return K_END; case 0x3E: return K_F5;
    case 0x54: return '/'; case 0x55: return '*'; case 0x56: return '-'; case 0x57: return '+';
    case 0x62: return '0'; case 0x63: return '.';
  }
  if (code >= 0x59 && code <= 0x61) return '1' + code - 0x59;
  return 0;
}

void kbCallback(usb_transfer_t *t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes >= 8) {
    uint8_t *r = t->data_buffer; uint8_t mod = r[0];
    bool anyHeld = false;
    for (int i = 2; i < 8; i++) {
      uint8_t k = r[i]; if (k < 4) continue;
      bool was = false; for (int j = 0; j < 6; j++) if (prevKeys[j] == k) was = true;
      if (!was) {
        uint16_t key = hidToKey(k, mod);
        if (key) { pushKey(key); heldKey = key; heldSince = millis(); }
      }
      anyHeld = true;
    }
    if (!anyHeld) heldKey = 0;
    memcpy(prevKeys, r + 2, 6);
  }
  if (devHdl && !devGone) usb_host_transfer_submit(t);
}

void ctrlCallback(usb_transfer_t *t) {}

void openDevice(uint8_t addr) {
  if (usb_host_device_open(clientHdl, addr, &devHdl) != ESP_OK) { devHdl = nullptr; return; }
  const usb_config_desc_t *cfg;
  if (usb_host_get_active_config_descriptor(devHdl, &cfg) != ESP_OK) return;
  const uint8_t *p = (const uint8_t *)cfg; int total = cfg->wTotalLength, off = 0;
  int curIface = -1; bool isKb = false; uint8_t ep = 0; uint16_t mps = 8;
  while (off < total) {
    uint8_t len = p[off], type = p[off + 1]; if (len == 0) break;
    if (type == 0x04) { curIface = p[off + 2]; isKb = (p[off + 5] == 3 && p[off + 7] == 1); }
    else if (type == 0x05 && isKb && !ep && (p[off + 2] & 0x80) && (p[off + 3] & 3) == 3) {
      ep = p[off + 2]; mps = p[off + 4] | (p[off + 5] << 8); kbIface = curIface;
    }
    off += len;
  }
  if (!ep) { Serial.println("USB: bukan keyboard"); return; }
  if (usb_host_interface_claim(clientHdl, devHdl, kbIface, 0) != ESP_OK) return;
  kbClaimed = true;
  // SET_PROTOCOL (boot)
  usb_host_transfer_alloc(64, 0, &ctrlXfer);
  usb_setup_packet_t *sp = (usb_setup_packet_t *)ctrlXfer->data_buffer;
  sp->bmRequestType = 0x21; sp->bRequest = 0x0B; sp->wValue = 0; sp->wIndex = kbIface; sp->wLength = 0;
  ctrlXfer->num_bytes = sizeof(usb_setup_packet_t);
  ctrlXfer->device_handle = devHdl; ctrlXfer->bEndpointAddress = 0;
  ctrlXfer->callback = ctrlCallback; ctrlXfer->context = nullptr;
  usb_host_transfer_submit_control(clientHdl, ctrlXfer);
  delay(20);
  usb_host_transfer_alloc(mps < 8 ? 8 : mps, 0, &kbXfer);
  kbXfer->num_bytes = mps < 8 ? 8 : mps;
  kbXfer->device_handle = devHdl; kbXfer->bEndpointAddress = ep;
  kbXfer->callback = kbCallback; kbXfer->context = nullptr;
  memset(prevKeys, 0, 6);
  usb_host_transfer_submit(kbXfer);
  Serial.println("USB: keyboard siap");
}

void closeDevice() {
  if (kbXfer) { usb_host_transfer_free(kbXfer); kbXfer = nullptr; }
  if (ctrlXfer) { usb_host_transfer_free(ctrlXfer); ctrlXfer = nullptr; }
  if (devHdl) {
    if (kbClaimed) usb_host_interface_release(clientHdl, devHdl, kbIface);
    usb_host_device_close(clientHdl, devHdl);
  }
  kbClaimed = false; devHdl = nullptr; heldKey = 0;
  Serial.println("USB: keyboard dilepas");
}

void clientEvent(const usb_host_client_event_msg_t *msg, void *arg) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) newDevAddr = msg->new_dev.address;
  else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) devGone = true;
}

void usbLibTask(void *) {
  usb_host_config_t hc = {}; hc.skip_phy_setup = false; hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
  usb_host_install(&hc);
  while (true) { uint32_t f; usb_host_lib_handle_events(portMAX_DELAY, &f); if (f & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all(); }
}

void usbClientTask(void *) {
  delay(200);
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false; cc.max_num_event_msg = 5;
  cc.async.client_event_callback = clientEvent; cc.async.callback_arg = nullptr;
  while (usb_host_client_register(&cc, &clientHdl) != ESP_OK) delay(100);
  while (true) {
    usb_host_client_handle_events(clientHdl, pdMS_TO_TICKS(10));
    if (newDevAddr) { uint8_t a = newDevAddr; newDevAddr = 0; if (!devHdl) openDevice(a); }
    if (devGone) { closeDevice(); devGone = false; }
  }
}

// =====================================================================
//                               INPUT
// =====================================================================
void handleKey(uint16_t k) {
  if (busy) return;
  switch (k) {
    case K_ENTER: submit(); return;
    case K_BKSP:
      if (inputBuf.length()) {
        int n = inputBuf.length() - 1;
        while (n > 0 && ((uint8_t)inputBuf[n] & 0xC0) == 0x80) n--;
        inputBuf.remove(n);
      }
      break;
    case K_ESC:
      if (photoJpg) { free(photoJpg); photoJpg = nullptr; photoLen = 0; } else inputBuf = "";
      break;
    case K_UP: scrollTop = max(0, scrollTop - 1); break;
    case K_DOWN: scrollTop = min(maxScroll(), scrollTop + 1); break;
    case K_PGUP: scrollTop = max(0, scrollTop - (ANS_LINES - 1)); break;
    case K_PGDN: scrollTop = min(maxScroll(), scrollTop + ANS_LINES - 1); break;
    case K_HOME: scrollTop = 0; break;
    case K_END: scrollTop = maxScroll(); break;
    case K_F5: history.clear(); lines.clear(); scrollTop = 0; break;
    case '~': takePhoto(); return;
    default:
      if (k >= 0x20 && k < 0x7F && inputBuf.length() < 1500) inputBuf += (char)k;
  }
  render();
}

// =====================================================================
//                            SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  keyQ = xQueueCreate(64, sizeof(uint16_t));

  SPI.begin(TFT_SCK, -1, TFT_MOSI, -1);
  tft.init(240, 240, SPI_MODE3);
  tft.setSPISpeed(40000000);
  tft.setRotation(2);   // ganti 0..3 jika tampilan terbalik
  tft.cp437(true);
  tft.fillScreen(ST77XX_BLACK);
  canvas = new GFXcanvas16(240, 240);
  canvas->cp437(true);
  canvas->setTextWrap(false);

  prefs.begin("ai", true);
  cfgSsid = prefs.getString("ssid", ""); cfgPass = prefs.getString("pass", "");
  cfgKey = prefs.getString("key", ""); cfgModel = prefs.getString("model", "gemini-2.5-flash");
  prefs.end();

  setStatus("Memulai kamera...");
  camOk = initCamera();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  if (cfgSsid.length()) {
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
    setStatus("Menghubungkan " + cfgSsid + "...");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  }
  WiFi.scanNetworks(true);

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/models", handleModels);
  server.begin();

  xTaskCreatePinnedToCore(usbLibTask, "usblib", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(usbClientTask, "usbcli", 6144, nullptr, 5, nullptr, 0);

  status = "";
  addWrapped("=== ESP32-S3 GEMINI AI ===", ST77XX_YELLOW);
  addWrapped(toCp437("Setting: WiFi \"" AP_SSID "\" pass " AP_PASS), 0xBDF7);
  addWrapped("lalu buka http://192.168.4.1", 0xBDF7);
  if (WiFi.status() == WL_CONNECTED) addWrapped("atau http://" + WiFi.localIP().toString(), ST77XX_GREEN);
  else addWrapped("WiFi rumah belum terhubung", ST77XX_RED);
  if (!cfgKey.length()) addWrapped("API key Gemini belum diisi!", ST77XX_RED);
  addWrapped(camOk ? "Kamera OK" : "Kamera GAGAL", camOk ? ST77XX_GREEN : ST77XX_RED);
  addWrapped("Panah=scroll  ~=foto  ESC=batal  F5=reset", 0x7BEF);
  render();
}

unsigned long lastRepeat = 0, lastStatusDraw = 0;

void loop() {
  server.handleClient();
  uint16_t k;
  bool got = false;
  while (xQueueReceive(keyQ, &k, 0) == pdTRUE) { handleKey(k); got = true; }
  // auto-repeat untuk panah & backspace
  uint16_t h = heldKey;
  if (!got && (h == K_UP || h == K_DOWN || h == K_BKSP || h == K_PGUP || h == K_PGDN) &&
      millis() - heldSince > 400 && millis() - lastRepeat > 60) {
    lastRepeat = millis(); handleKey(h);
  }
  if (millis() - lastStatusDraw > 5000) { lastStatusDraw = millis(); render(); }
  delay(5);
}
