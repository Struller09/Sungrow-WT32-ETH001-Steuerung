#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include <errno.h>
#include <fcntl.h>
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include "driver/gpio.h"

static const char *TAG = "SUNGROW_ETH_AP";

// ===================== KONFIGURATION =====================
// Standardwerte (gelten beim ersten Start; danach per Weboberfläche änderbar)
#define DEFAULT_SUNGROW_IP   "192.168.188.60"
#define DEFAULT_WEB_PASS     "admin"          // Login: Benutzer "admin"
#define WEB_USER             "admin"
#define SUNGROW_PORT         502
#define SUNGROW_SLAVE_ID     1

// WLAN-Hotspot (SoftAP)
#define AP_WIFI_SSID         "Sungrow-Steuerung"
#define AP_WIFI_PASS         "sungrow2026"    // Mindestens 8 Zeichen!
#define AP_WIFI_CHANNEL      6
#define AP_MAX_CONN          4

// Sungrow Modbus Register (0-basiert; falls Fehler, 13023 bzw. 13100 testen)
#define REG_BATTERY_SOC      13022            // Input Register (FC 0x04), Faktor 0.1 (%)
#define REG_BACKUP_SOC       13099            // Holding Register (FC 0x03 / 0x06), Faktor 1 (%)

// Batterie-Strom (0,1 A) und -Leistung (W): zwei aufeinanderfolgende Input Register (FC 0x04)
// Laut Sungrow-Protokoll 1-basiert 13021 / 13022 -> hier 0-basiert 13020 / 13021
#define REG_BATTERY_CURRENT  13020
#define REG_BATTERY_POWER    13021
// Richtung (Laden/Entladen) kommt aus dem Betriebszustand der Batterie (Input Register 13000):
// Bit 1 (Wert 2) = lädt, Bit 2 (Wert 4) = entlädt.
#define REG_RUNNING_STATE    13000
// Nur als Rückfall, falls keines der beiden Bits gesetzt ist: Vorzeichen des Batteriestroms.
// 1 = negativ bedeutet Laden, 0 = positiv bedeutet Laden
#define BATTERY_CURRENT_NEG_IS_CHARGING  0

// Hausverbrauch (Input Register, 0-basiert; im Sungrow-Protokoll steht jeweils die Adresse + 1)
#define REG_LOAD_POWER       13007   // S32, W (niederwertiges Wort zuerst)
#define REG_DIRECT_ENERGY    13016   // Tag (U16) + Gesamt (U32), 0,1 kWh: direkt aus PV verbraucht
#define REG_DISCHARGE_ENERGY 13025   // Tag (U16) + Gesamt (U32), 0,1 kWh: aus der Batterie entladen
#define REG_IMPORT_ENERGY    13035   // Tag (U16) + Gesamt (U32), 0,1 kWh: aus dem Netz bezogen
#define REG_CLOCK            4999    // Holding: Jahr, Monat, Tag der Wechselrichter-Uhr

// Wallbox Sungrow AC011E-01 / AC22E-01 (Modbus TCP über eigene IP, hier nur lesend)
#define WALLBOX_PORT         502
#define WALLBOX_UNIT_ID      248
#define WB_REG_MODE          21262   // Input: 0 Netzwerk, 2 Plug&Play, 6 EMS
#define WB_REG_PHASE_STATE   21269   // Input: 0 dreiphasig, 1 einphasig
#define WB_REG_POWER         21307   // Input: 32 Bit (niederwertiges Wort zuerst), Watt
#define WB_REG_STATE         21316   // Input: 1..9, siehe wb_state_text()
#define WB_REG_SET_CURRENT   21202   // Holding: Ladestrom-Sollwert (Einheit noch zu prüfen)

// Erlaubte Backup-SOC-Werte für die Buttons
#define SOC_NORMAL           15
#define SOC_RELEASED         0

// Modbus-Abfrageintervall
#define DEFAULT_POLL_INTERVAL_S  10   // Standard-Abfrageintervall (in Weboberfläche änderbar)
#define POLL_MIN_S               2
#define POLL_MAX_S               3600

// Passwort-Längen
#define PASS_MIN_LEN         8
#define PASS_MAX_LEN         32

// WT32-ETH01 Hardware-Pins für LAN8720
#define ETH_PHY_ADDR         1
#define ETH_MDC_GPIO         23
#define ETH_MDIO_GPIO        18
#define ETH_POWER_GPIO       16
// =========================================================

#define ETH_GOT_IP_BIT       BIT0

// ---- Laufzeit-Konfiguration (aus NVS geladen) ----
static char g_mb_ip[16]                = DEFAULT_SUNGROW_IP;
static char g_wb_ip[16]                = "";   // leer = Wallbox-Abfrage aus
static char g_web_pass[PASS_MAX_LEN+1] = DEFAULT_WEB_PASS;
static volatile uint32_t g_poll_interval_s = DEFAULT_POLL_INTERVAL_S;
static TaskHandle_t s_modbus_task = NULL;

// ---- Modbus-Zustand ----
static esp_netif_t *s_eth_netif = NULL;
static char s_eth_ip[16] = "";              // per DHCP erhaltene Ethernet-IP ("" = keine)
static volatile bool s_modbus_ready = false;
static SemaphoreHandle_t s_mb_mutex = NULL;     // serialisiert alle Modbus-Zugriffe
static SemaphoreHandle_t s_cache_mutex = NULL;  // schützt den Cache
static EventGroupHandle_t s_net_events = NULL;

typedef struct {
    bool valid;            // Wallbox erreichbar und Status gelesen
    int  state;            // 1..9 (siehe wb_state_text)
    int  power_w;          // -1 = unbekannt
    int  cur_da[3];        // Phasenströme in 0,1 A, -1 = unbekannt
    int  phases;           // 1 oder 3, 0 = unbekannt
    int  mode;             // 0 / 2 / 6, -1 = unbekannt
    int  setpoint;         // Sollwert-Register roh, -1 = unbekannt
} wb_data_t;

typedef struct {
    bool     valid;        // aktuelle Leistung gelesen
    int      load_w;       // aktuelle Hausleistung in W
    int      day_dkwh;     // Verbrauch heute in 0,1 kWh, -1 = unbekannt
    int      year_dkwh;    // Verbrauch im Jahr in 0,1 kWh, -1 = unbekannt
    uint32_t year_since;   // Beginn der Jahreszählung (JJJJMMTT), 0 = unbekannt
} cons_data_t;

typedef struct {
    float   bat_soc;       // -1 = ungültig
    int     backup_soc;    // -1 = ungültig
    int64_t updated_us;    // 0 = noch nie erfolgreich
    int     bat_power_w;   // > 0 = Laden, < 0 = Entladen
    bool    power_valid;
    wb_data_t wb;          // Wallbox
    cons_data_t cons;      // Hausverbrauch
} soc_cache_t;
static soc_cache_t s_cache = { -1.0f, -1, 0, 0, false,
                               { false, 0, -1, { -1, -1, -1 }, 0, -1, -1 },
                               { false, 0, -1, -1, 0 } };

// =====================================================
//                    NVS-Konfiguration
// =====================================================
static void config_load(void) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "Keine gespeicherte Konfiguration, nutze Standardwerte.");
        return;
    }
    size_t len = sizeof(g_mb_ip);
    if (nvs_get_str(h, "mb_ip", g_mb_ip, &len) != ESP_OK) {
        strlcpy(g_mb_ip, DEFAULT_SUNGROW_IP, sizeof(g_mb_ip));
    } else {
        ESP_LOGI(TAG, "Gespeicherte Sungrow-IP aus dem Flash geladen: %s", g_mb_ip);
    }
    len = sizeof(g_web_pass);
    if (nvs_get_str(h, "web_pass", g_web_pass, &len) != ESP_OK) {
        strlcpy(g_web_pass, DEFAULT_WEB_PASS, sizeof(g_web_pass));
    }
    uint32_t poll = 0;
    if (nvs_get_u32(h, "poll_s", &poll) == ESP_OK && poll >= POLL_MIN_S && poll <= POLL_MAX_S) {
        g_poll_interval_s = poll;
    }
    len = sizeof(g_wb_ip);
    if (nvs_get_str(h, "wb_ip", g_wb_ip, &len) != ESP_OK) {
        g_wb_ip[0] = '\0';
    }
    nvs_close(h);
}

static esp_err_t config_save_str(const char *key, const char *value) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, key, value);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t config_save_u32(const char *key, uint32_t value) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(h, key, value);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

// Prüft "a.b.c.d" mit je 0..255
static bool is_valid_ipv4(const char *s) {
    int a, b, c, d, n = 0;
    if (strlen(s) > 15) return false;
    if (sscanf(s, "%3d.%3d.%3d.%3d%n", &a, &b, &c, &d, &n) != 4) return false;
    if (s[n] != '\0') return false;
    return a >= 0 && a <= 255 && b >= 0 && b <= 255 &&
           c >= 0 && c <= 255 && d >= 0 && d <= 255;
}

// =====================================================
//                       Modbus
// =====================================================
#define MB_IO_TIMEOUT_MS       3000
#define MB_CONNECT_TIMEOUT_MS  3000

// Eine Modbus-TCP-Verbindung (Wechselrichter oder Wallbox)
typedef struct {
    const char *name;      // für Log-Meldungen
    const char *ip;        // zeigt auf die gespeicherte IP-Zeichenkette
    uint16_t    port;
    uint8_t     unit_id;
    int         sock;
    uint16_t    tid;
} mb_dev_t;

static mb_dev_t s_inv = { "Wechselrichter", g_mb_ip, SUNGROW_PORT, SUNGROW_SLAVE_ID, -1, 0 };
static mb_dev_t s_wb  = { "Wallbox",        g_wb_ip, WALLBOX_PORT, WALLBOX_UNIT_ID,  -1, 0 };

static void mb_close(mb_dev_t *dev) {
    if (dev->sock >= 0) {
        close(dev->sock);
        dev->sock = -1;
    }
}

// Verbindung aufbauen (nicht-blockierend mit Timeout), falls noch keine besteht
static esp_err_t mb_connect(mb_dev_t *dev) {
    if (dev->sock >= 0) return ESP_OK;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(dev->port);
    if (inet_pton(AF_INET, dev->ip, &dest.sin_addr) != 1) return ESP_ERR_INVALID_ARG;

    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (s < 0) return ESP_FAIL;

    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    int rc = connect(s, (struct sockaddr *)&dest, sizeof(dest));
    if (rc != 0 && errno != EINPROGRESS) {
        close(s);
        return ESP_FAIL;
    }
    if (rc != 0) {
        fd_set wset;
        FD_ZERO(&wset);
        FD_SET(s, &wset);
        struct timeval tv = {
            .tv_sec  = MB_CONNECT_TIMEOUT_MS / 1000,
            .tv_usec = (MB_CONNECT_TIMEOUT_MS % 1000) * 1000
        };
        if (select(s + 1, NULL, &wset, NULL, &tv) <= 0) {
            close(s);
            return ESP_ERR_TIMEOUT;
        }
        int soerr = 0;
        socklen_t sl = sizeof(soerr);
        if (getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &sl) != 0 || soerr != 0) {
            close(s);
            return ESP_FAIL;
        }
    }

    fcntl(s, F_SETFL, flags);   // wieder blockierend, dafür mit Timeouts
    struct timeval io_tv = {
        .tv_sec  = MB_IO_TIMEOUT_MS / 1000,
        .tv_usec = (MB_IO_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &io_tv, sizeof(io_tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &io_tv, sizeof(io_tv));
    int nodelay = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    dev->sock = s;
    ESP_LOGI(TAG, "Modbus-TCP (%s) verbunden mit %s:%d", dev->name, dev->ip, dev->port);
    return ESP_OK;
}

static bool mb_recv_all(mb_dev_t *dev, uint8_t *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        int r = recv(dev->sock, buf + got, n - got, 0);
        if (r <= 0) return false;
        got += (size_t)r;
    }
    return true;
}

// Eine Modbus-TCP-Transaktion (FC 0x03/0x04 lesen, FC 0x06 schreiben).
// arg = Anzahl Register (FC 03/04) bzw. zu schreibender Wert (FC 06).
// result: Array mit arg Werten (FC 03/04, max. 4 Register) bzw. zurückgemeldeter Wert (FC 06).
static esp_err_t mb_transaction(mb_dev_t *dev, uint8_t fc, uint16_t reg, uint16_t arg, uint16_t *result) {
    if ((fc == 0x03 || fc == 0x04) && (arg == 0 || arg > 4)) return ESP_ERR_INVALID_ARG;
    esp_err_t err = mb_connect(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Modbus-Verbindung (%s) zu %s fehlgeschlagen: 0x%x", dev->name, dev->ip, err);
        return err;
    }

    uint16_t tid = ++dev->tid;
    uint8_t req[12] = {
        (uint8_t)(tid >> 8), (uint8_t)(tid & 0xFF),   // Transaction ID
        0, 0,                                          // Protocol ID
        0, 6,                                          // Länge (Unit + PDU)
        dev->unit_id, fc,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(arg >> 8), (uint8_t)(arg & 0xFF)
    };
    if (send(dev->sock, req, sizeof(req), 0) != (int)sizeof(req)) {
        ESP_LOGE(TAG, "Modbus (%s): Senden fehlgeschlagen", dev->name);
        mb_close(dev);
        return ESP_FAIL;
    }

    uint8_t rsp[32];
    if (!mb_recv_all(dev, rsp, 7)) {                   // MBAP-Header (6) + Unit-ID (1)
        ESP_LOGE(TAG, "Modbus (%s): Keine Antwort (Timeout)", dev->name);
        mb_close(dev);
        return ESP_ERR_TIMEOUT;
    }
    uint16_t rtid  = (uint16_t)((rsp[0] << 8) | rsp[1]);
    uint16_t proto = (uint16_t)((rsp[2] << 8) | rsp[3]);
    uint16_t len   = (uint16_t)((rsp[4] << 8) | rsp[5]);
    if (rtid != tid || proto != 0 || len < 2 || len > sizeof(rsp) - 6) {
        ESP_LOGE(TAG, "Modbus (%s): Ungültige Antwort (tid %u/%u, len %u)", dev->name, rtid, tid, len);
        mb_close(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (!mb_recv_all(dev, rsp + 7, len - 1)) {
        ESP_LOGE(TAG, "Modbus (%s): Antwort unvollständig", dev->name);
        mb_close(dev);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t rfc = rsp[7];
    if (rfc & 0x80) {
        ESP_LOGE(TAG, "Modbus-Exception (%s): FC 0x%02x, Code 0x%02x (Register %u)",
                 dev->name, fc, len >= 3 ? rsp[8] : 0, reg);
        return ESP_ERR_INVALID_RESPONSE;               // Verbindung bleibt bestehen
    }
    if (rfc != fc) {
        ESP_LOGE(TAG, "Modbus (%s): Unerwarteter Funktionscode 0x%02x", dev->name, rfc);
        mb_close(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (fc == 0x03 || fc == 0x04) {
        uint8_t bytes = (uint8_t)(arg * 2);
        if (len < 3 + bytes || rsp[8] != bytes) {
            ESP_LOGE(TAG, "Modbus (%s): Unerwartete Datenlänge", dev->name);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (result) {
            for (uint16_t i = 0; i < arg; i++) {
                result[i] = (uint16_t)((rsp[9 + 2 * i] << 8) | rsp[10 + 2 * i]);
            }
        }
    } else if (fc == 0x06) {
        if (len < 6) {
            ESP_LOGE(TAG, "Modbus (%s): Unerwartete Schreibantwort", dev->name);
            return ESP_ERR_INVALID_RESPONSE;
        }
        uint16_t echo = (uint16_t)((rsp[10] << 8) | rsp[11]);
        if (result) *result = echo;
        if (echo != arg) return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static float modbus_read_battery_soc_locked(void) {
    uint16_t raw = 0;
    esp_err_t err = mb_transaction(&s_inv, 0x04, REG_BATTERY_SOC, 1, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Lesen des Batterie-SOC: 0x%x", err);
        return -1.0f;
    }
    return (float)raw / 10.0f;
}

static int modbus_read_backup_soc_locked(void) {
    uint16_t raw = 0;
    esp_err_t err = mb_transaction(&s_inv, 0x03, REG_BACKUP_SOC, 1, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Lesen des Backup-SOC: 0x%x", err);
        return -1;
    }
    return (int)raw;
}

// Betrag aus der Batterieleistung (W), Richtung aus dem Betriebszustand (Register 13000).
// Ergebnis: > 0 = Laden, < 0 = Entladen
static bool modbus_read_battery_power_locked(int *watts) {
    uint16_t r[2] = { 0, 0 };   // r[0] = Strom (0,1 A), r[1] = Leistung (W)
    uint16_t state = 0;
    esp_err_t err = mb_transaction(&s_inv, 0x04, REG_BATTERY_CURRENT, 2, r);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Lesen der Batterieleistung: 0x%x", err);
        return false;
    }
    bool have_state = (mb_transaction(&s_inv, 0x04, REG_RUNNING_STATE, 1, &state) == ESP_OK);

    int current = (int)(int16_t)r[0];
    int power   = abs((int)(int16_t)r[1]);
    ESP_LOGD(TAG, "Batterie: Strom=%d (0,1 A), Leistung=%d W, Zustand=0x%04x%s",
             current, power, state, have_state ? "" : " (nicht lesbar)");

    if (power == 0) {
        *watts = 0;
        return true;
    }

    bool charging;
    if (have_state && (state & 0x0002)) {
        charging = true;
    } else if (have_state && (state & 0x0004)) {
        charging = false;
    } else {
        // Rückfall: Richtung aus dem Vorzeichen des Batteriestroms
        charging = BATTERY_CURRENT_NEG_IS_CHARGING ? (current < 0) : (current > 0);
    }
    *watts = charging ? power : -power;
    return true;
}

static void cache_update(float bat, int backup) {
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.bat_soc    = bat;
    s_cache.backup_soc = backup;
    if (bat >= 0.0f || backup >= 0) s_cache.updated_us = esp_timer_get_time();
    xSemaphoreGive(s_cache_mutex);
}

static void cache_update_power(bool valid, int watts) {
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.power_valid = valid;
    s_cache.bat_power_w = valid ? watts : 0;
    if (valid) s_cache.updated_us = esp_timer_get_time();
    xSemaphoreGive(s_cache_mutex);
}

static soc_cache_t cache_get(void) {
    soc_cache_t c;
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    c = s_cache;
    xSemaphoreGive(s_cache_mutex);
    return c;
}

static esp_err_t set_sungrow_backup_soc(uint16_t soc_percent) {
    if (soc_percent > 100) return ESP_ERR_INVALID_ARG;
    if (!s_modbus_ready) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_mb_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    uint16_t echo = 0;
    esp_err_t err = mb_transaction(&s_inv, 0x06, REG_BACKUP_SOC, soc_percent, &echo);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Backup-SOC erfolgreich auf %u%% gesetzt.", (unsigned)soc_percent);
        // Zur Kontrolle sofort zurücklesen und Cache aktualisieren
        int readback = modbus_read_backup_soc_locked();
        soc_cache_t c = cache_get();
        cache_update(c.bat_soc, readback);
    } else {
        ESP_LOGE(TAG, "Modbus Schreibfehler: 0x%x", err);
    }
    xSemaphoreGive(s_mb_mutex);
    return err;
}

// ---- Wallbox (Sungrow AC011E-01 / AC22E-01), nur Lesen ----
static const uint16_t WB_REG_CURRENTS[3] = { 21302, 21304, 21306 };   // Input, 0,1 A je Phase

static const char *wb_state_text(int s) {
    switch (s) {
        case 1:  return "Frei (kein Fahrzeug)";
        case 2:  return "Fahrzeug verbunden (Standby)";
        case 3:  return "Lädt";
        case 4:  return "Pausiert (Wallbox)";
        case 5:  return "Pausiert (Fahrzeug)";
        case 6:  return "Laden beendet";
        case 8:  return "Nicht verfügbar";
        case 9:  return "Fehler";
        default: return "Unbekannt";
    }
}

static const char *wb_mode_text(int m) {
    switch (m) {
        case 0:  return "Netzwerk";
        case 2:  return "Plug&amp;Play";
        case 6:  return "EMS";
        default: return "unbekannt";
    }
}

// Liest alle Wallbox-Werte. Nur der Status ist Pflicht, alle anderen Werte sind optional.
static bool wallbox_read(wb_data_t *d) {
    uint16_t r[2] = { 0, 0 };
    memset(d, 0, sizeof(*d));
    d->power_w = -1;
    d->cur_da[0] = d->cur_da[1] = d->cur_da[2] = -1;
    d->mode = -1;
    d->setpoint = -1;

    if (mb_transaction(&s_wb, 0x04, WB_REG_STATE, 1, r) != ESP_OK) return false;
    d->state = r[0];

    if (mb_transaction(&s_wb, 0x04, WB_REG_POWER, 2, r) == ESP_OK) {
        d->power_w = (int)(((uint32_t)r[1] << 16) | r[0]);   // niederwertiges Wort zuerst
    }
    for (int i = 0; i < 3; i++) {
        if (mb_transaction(&s_wb, 0x04, WB_REG_CURRENTS[i], 1, r) == ESP_OK) d->cur_da[i] = r[0];
    }
    if (mb_transaction(&s_wb, 0x04, WB_REG_PHASE_STATE, 1, r) == ESP_OK) d->phases = (r[0] == 0) ? 3 : 1;
    if (mb_transaction(&s_wb, 0x04, WB_REG_MODE, 1, r) == ESP_OK) d->mode = r[0];
    if (mb_transaction(&s_wb, 0x03, WB_REG_SET_CURRENT, 1, r) == ESP_OK) d->setpoint = r[0];

    d->valid = true;
    return true;
}

static void cache_update_wallbox(const wb_data_t *d) {
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.wb = *d;
    xSemaphoreGive(s_cache_mutex);
}

// ---- Hausverbrauch: aktuelle Leistung, Tag, Jahr ----
// 32-Bit-Werte: niederwertiges Wort zuerst
static uint32_t u32_le(const uint16_t *r) {
    return ((uint32_t)r[1] << 16) | r[0];
}

// Tageswert (1 Register) und Gesamtzähler (2 Register) ab Startregister, jeweils 0,1 kWh
static bool read_energy_triple(uint16_t reg, uint32_t *day, uint32_t *total) {
    uint16_t r[3];
    if (mb_transaction(&s_inv, 0x04, reg, 3, r) != ESP_OK) return false;
    *day = r[0];
    *total = u32_le(&r[1]);
    return true;
}

// Datum der Wechselrichter-Uhr (Holding Register)
static bool read_inverter_date(uint32_t *year, uint32_t *ymd) {
    uint16_t r[3];
    if (mb_transaction(&s_inv, 0x03, REG_CLOCK, 3, r) != ESP_OK) return false;
    if (r[0] < 2020 || r[0] > 2100 || r[1] < 1 || r[1] > 12 || r[2] < 1 || r[2] > 31) return false;
    *year = r[0];
    *ymd  = (uint32_t)r[0] * 10000 + (uint32_t)r[1] * 100 + r[2];
    return true;
}

// Jahreszählung: Gesamtverbrauch zu Beginn des Jahres wird im Flash gemerkt
static bool     s_year_loaded = false;
static uint32_t s_year_num   = 0;   // Jahr der Zählung
static uint32_t s_year_base  = 0;   // Gesamtverbrauch zu Beginn der Zählung (0,1 kWh)
static uint32_t s_year_since = 0;   // Beginn der Zählung (JJJJMMTT)

static void year_update(uint32_t total, uint32_t year, uint32_t ymd) {
    if (!s_year_loaded) {
        nvs_handle_t h;
        if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) {
            (void)nvs_get_u32(h, "yr_num",   &s_year_num);
            (void)nvs_get_u32(h, "yr_base",  &s_year_base);
            (void)nvs_get_u32(h, "yr_since", &s_year_since);
            nvs_close(h);
        }
        s_year_loaded = true;
    }
    // Neues Jahr (oder noch keine Basis / Zähler kleiner geworden): neue Basis setzen
    if (s_year_num != year || total < s_year_base) {
        s_year_num   = year;
        s_year_base  = total;
        s_year_since = ymd;
        config_save_u32("yr_num",   s_year_num);
        config_save_u32("yr_base",  s_year_base);
        config_save_u32("yr_since", s_year_since);
        ESP_LOGI(TAG, "Jahreszählung gestartet: %u, Basis %u (0,1 kWh)", (unsigned)year, (unsigned)total);
    }
}

// Verbrauch = direkt aus PV + aus Batterie entladen + aus dem Netz bezogen
static void consumption_read(cons_data_t *d) {
    uint16_t r[2] = { 0, 0 };
    d->valid = false;
    d->load_w = 0;
    d->day_dkwh = -1;
    d->year_dkwh = -1;
    d->year_since = 0;

    if (mb_transaction(&s_inv, 0x04, REG_LOAD_POWER, 2, r) == ESP_OK) {
        d->load_w = (int)(int32_t)u32_le(r);
        d->valid = true;
    }

    uint32_t day_direct, tot_direct, day_dis, tot_dis, day_imp, tot_imp;
    if (read_energy_triple(REG_DIRECT_ENERGY, &day_direct, &tot_direct) &&
        read_energy_triple(REG_DISCHARGE_ENERGY, &day_dis, &tot_dis) &&
        read_energy_triple(REG_IMPORT_ENERGY, &day_imp, &tot_imp)) {

        d->day_dkwh = (int)(day_direct + day_dis + day_imp);

        uint32_t total = tot_direct + tot_dis + tot_imp;
        uint32_t year, ymd;
        if (read_inverter_date(&year, &ymd)) {
            year_update(total, year, ymd);
            if (total >= s_year_base) {
                d->year_dkwh  = (int)(total - s_year_base);
                d->year_since = s_year_since;
            }
        }
    }
}

static void cache_update_cons(const cons_data_t *d) {
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.cons = *d;
    xSemaphoreGive(s_cache_mutex);
}

// Wartet auf eine IP per Event und fragt dann zyklisch ab
static void modbus_task(void *arg) {
    xEventGroupWaitBits(s_net_events, ETH_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Modbus-TCP-Client bereit (Ziel: %s:%d).", g_mb_ip, SUNGROW_PORT);
    s_modbus_ready = true;

    while (1) {
        if (xSemaphoreTake(s_mb_mutex, pdMS_TO_TICKS(30000)) == pdTRUE) {
            float bat = modbus_read_battery_soc_locked();
            int backup = modbus_read_backup_soc_locked();
            int watts = 0;
            bool power_ok = modbus_read_battery_power_locked(&watts);
            cons_data_t cons;
            consumption_read(&cons);
            xSemaphoreGive(s_mb_mutex);
            cache_update(bat, backup);
            cache_update_power(power_ok, watts);
            cache_update_cons(&cons);
        }

        // Wallbox (eigene Verbindung), nur wenn eine IP eingetragen ist
        if (g_wb_ip[0] != '\0') {
            wb_data_t wb;
            wallbox_read(&wb);
            cache_update_wallbox(&wb);
        }
        // Wartet das eingestellte Intervall; eine Benachrichtigung (neues Intervall) weckt sofort auf
        uint32_t secs = g_poll_interval_s;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(secs * 1000));
    }
}

// =====================================================
//               Authentifizierung / Sicherheit
// =====================================================
static bool const_time_equal(const char *a, const char *b) {
    size_t la = strlen(a), lb = strlen(b);
    if (la != lb) return false;
    unsigned char diff = 0;
    for (size_t i = 0; i < la; i++) diff |= (unsigned char)(a[i] ^ b[i]);
    return diff == 0;
}

static bool check_basic_auth(httpd_req_t *req) {
    char hdr[160];
    size_t hlen = httpd_req_get_hdr_value_len(req, "Authorization");
    if (hlen == 0 || hlen >= sizeof(hdr)) return false;
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK) return false;

    char cred[PASS_MAX_LEN + sizeof(WEB_USER) + 2];
    snprintf(cred, sizeof(cred), "%s:%s", WEB_USER, g_web_pass);

    unsigned char b64[128];
    size_t olen = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64) - 1, &olen, (const unsigned char *)cred, strlen(cred)) != 0) {
        return false;
    }
    b64[olen] = '\0';

    char expected[160];
    snprintf(expected, sizeof(expected), "Basic %s", (char *)b64);
    return const_time_equal(hdr, expected);
}

// Schutz gegen fremde Webseiten (CSRF): Origin muss zum Host passen
static bool check_same_origin(httpd_req_t *req) {
    char origin[96], host[64];
    if (httpd_req_get_hdr_value_len(req, "Origin") == 0) return true;   // kein Origin -> kein Browser-Cross-Site-Request
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK) return false;
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK) return false;
    const char *prefix = "http://";
    if (strncmp(origin, prefix, strlen(prefix)) != 0) return false;
    return strcmp(origin + strlen(prefix), host) == 0;
}

// Liefert true, wenn der Handler weitermachen darf. Sonst wurde bereits eine Antwort gesendet.
static bool guard(httpd_req_t *req, bool is_post) {
    if (!check_basic_auth(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Sungrow-Steuerung\"");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_send(req, "Anmeldung erforderlich", HTTPD_RESP_USE_STRLEN);
        return false;
    }
    if (is_post && !check_same_origin(req)) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Ungültiger Ursprung der Anfrage");
        return false;
    }
    return true;
}

// =====================================================
//                    Formular-Helfer
// =====================================================
static void url_decode(char *s) {
    char *o = s;
    while (*s) {
        if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char h[3] = { s[1], s[2], 0 };
            *o++ = (char)strtol(h, NULL, 16);
            s += 3;
        } else if (*s == '+') {
            *o++ = ' ';
            s++;
        } else {
            *o++ = *s++;
        }
    }
    *o = '\0';
}

// Liest den POST-Body (application/x-www-form-urlencoded) in buf
static bool read_body(httpd_req_t *req, char *buf, size_t size) {
    size_t total = req->content_len;
    if (total == 0 || total >= size) return false;
    size_t got = 0;
    int timeouts = 0;
    while (got < total) {
        int r = httpd_req_recv(req, buf + got, total - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 5) return false;
            continue;
        }
        if (r <= 0) return false;
        got += r;
    }
    buf[got] = '\0';
    return true;
}

static bool form_value(const char *body, const char *key, char *out, size_t size) {
    if (httpd_query_key_value(body, key, out, size) != ESP_OK) return false;
    url_decode(out);
    return true;
}

// =====================================================
//                      HTML-Seiten
// =====================================================
static const char *HTML_HEAD =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>Sungrow Steuerung</title>"
    "<style>"
    "body{font-family:sans-serif;text-align:center;margin:30px 10px;}"
    ".box{background:#f4f4f4;display:inline-block;padding:20px;border-radius:10px;margin-bottom:20px;max-width:420px;width:90%;}"
    ".box p{font-size:18px;margin:8px 0;}"
    "button{padding:14px 24px;font-size:16px;border:none;border-radius:5px;cursor:pointer;margin:6px;}"
    "input[type=text],input[type=password]{padding:10px;font-size:16px;width:80%;margin:6px 0;}"
    ".grn{background:#28a745;color:#fff;}.red{background:#d9534f;color:#fff;}.gry{background:#ddd;}"
    ".warn{background:#fff3cd;color:#856404;padding:10px;border-radius:8px;display:inline-block;margin-bottom:15px;}"
    "a{color:#0066cc;}"
    "</style></head><body>";

static const char *HTML_FOOT = "</body></html>";

static esp_err_t send_message_page(httpd_req_t *req, const char *title, const char *text, const char *link, const char *link_text) {
    char buf[512];
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    snprintf(buf, sizeof(buf), "<h2>%s</h2><div class='box'><p>%s</p></div><p><a href='%s'>%s</a></p>",
             title, text, link, link_text);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// Hauptseite (/)
static esp_err_t root_get_handler(httpd_req_t *req) {
    if (!guard(req, false)) return ESP_OK;

    soc_cache_t c = cache_get();
    char bat_str[32], backup_str[32], line[640];

    if (c.bat_soc >= 0.0f)  snprintf(bat_str, sizeof(bat_str), "%.1f %%", c.bat_soc);
    else                    snprintf(bat_str, sizeof(bat_str), "keine Daten");
    if (c.backup_soc >= 0)  snprintf(backup_str, sizeof(backup_str), "%d %%", c.backup_soc);
    else                    snprintf(backup_str, sizeof(backup_str), "keine Daten");

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<h2>Sungrow Batterie-Steuerung</h2>", HTTPD_RESP_USE_STRLEN);

    if (strcmp(g_web_pass, DEFAULT_WEB_PASS) == 0) {
        httpd_resp_send_chunk(req,
            "<div class='warn'>Standard-Passwort aktiv! Bitte unter "
            "<a href='/options'>Optionen</a> ändern.</div><br>", HTTPD_RESP_USE_STRLEN);
    }

    char pwr_str[48];
    if (!c.power_valid)         snprintf(pwr_str, sizeof(pwr_str), "keine Daten");
    else if (c.bat_power_w > 0) snprintf(pwr_str, sizeof(pwr_str), "Laden: %d W", c.bat_power_w);
    else if (c.bat_power_w < 0) snprintf(pwr_str, sizeof(pwr_str), "Entladen: %d W", -c.bat_power_w);
    else                        snprintf(pwr_str, sizeof(pwr_str), "0 W (Ruhe)");

    snprintf(line, sizeof(line),
        "<div class='box'>"
        "<p>Aktueller Akkustand: <b>%s</b></p>"
        "<p>Lade-/Entladeleistung: <b>%s</b></p>"
        "<p>Eingestellte Notreserve: <b>%s</b></p>"
        "</div><br>", bat_str, pwr_str, backup_str);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // Hausverbrauch
    {
        char load_str[24] = "keine Daten", day_str[24] = "keine Daten", year_str[160] = "keine Daten";
        if (c.cons.valid)         snprintf(load_str, sizeof(load_str), "%d W", c.cons.load_w);
        if (c.cons.day_dkwh >= 0) snprintf(day_str, sizeof(day_str), "%.1f kWh", c.cons.day_dkwh / 10.0);
        if (c.cons.year_dkwh >= 0) {
            unsigned yy = c.cons.year_since / 10000;
            unsigned mm = (c.cons.year_since / 100) % 100;
            unsigned dd = c.cons.year_since % 100;
            snprintf(year_str, sizeof(year_str),
                     "%.1f kWh <span style='font-size:12px;color:#666;'>(seit %02u.%02u.%04u)</span>",
                     c.cons.year_dkwh / 10.0, dd, mm, yy);
        }
        snprintf(line, sizeof(line),
            "<div class='box'><h3>Hausverbrauch</h3>"
            "<p>Aktuelle Leistung: <b>%s</b></p>"
            "<p>Verbrauch heute: <b>%s</b></p>"
            "<p>Verbrauch im Jahr: <b>%s</b></p>"
            "</div><br>", load_str, day_str, year_str);
        httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    }

    // Wallbox (nur wenn eine IP eingetragen ist)
    if (g_wb_ip[0] != '\0') {
        if (!c.wb.valid) {
            snprintf(line, sizeof(line),
                "<div class='box'><h3>Wallbox</h3>"
                "<p>Keine Verbindung zu <b>%s</b></p></div><br>", g_wb_ip);
        } else {
            char pwr[24] = "&ndash;", cur[96] = "&ndash;", phases[16] = "&ndash;", setp[24] = "&ndash;";
            if (c.wb.power_w >= 0) snprintf(pwr, sizeof(pwr), "%d W", c.wb.power_w);
            if (c.wb.cur_da[0] >= 0 && c.wb.cur_da[1] >= 0 && c.wb.cur_da[2] >= 0) {
                snprintf(cur, sizeof(cur), "%.1f / %.1f / %.1f A",
                         c.wb.cur_da[0] / 10.0, c.wb.cur_da[1] / 10.0, c.wb.cur_da[2] / 10.0);
            }
            if (c.wb.phases > 0)    snprintf(phases, sizeof(phases), "%d", c.wb.phases);
            if (c.wb.setpoint >= 0) snprintf(setp, sizeof(setp), "%d", c.wb.setpoint);

            snprintf(line, sizeof(line),
                "<div class='box'><h3>Wallbox</h3>"
                "<p>Status: <b>%s</b></p>"
                "<p>Ladeleistung: <b>%s</b></p>"
                "<p>Strom L1/L2/L3: <b>%s</b></p>"
                "<p style='font-size:13px;color:#666;'>Phasen: %s &middot; Betriebsart: %s &middot; "
                "Sollwert (roh): %s &middot; %s</p>"
                "</div><br>",
                wb_state_text(c.wb.state), pwr, cur, phases, wb_mode_text(c.wb.mode), setp, g_wb_ip);
        }
        httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    }

    // Freie Eingabe
    snprintf(line, sizeof(line),
        "<div class='box'><h3>Notreserve einstellen</h3>"
        "<form method='post' action='/set'>"
        "<input type='number' name='val' min='0' max='100' step='1' value='%d' required "
        "style='padding:10px;font-size:18px;width:90px;text-align:center;'> %%<br>"
        "<button class='grn' type='submit'>Übernehmen</button>"
        "</form></div><br>", c.backup_soc >= 0 ? c.backup_soc : SOC_NORMAL);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // Schnellwahl
    snprintf(line, sizeof(line),
        "<form method='post' action='/set'>"
        "<button class='gry' name='val' value='%d'>Normalbetrieb (%d %%)</button>"
        "<button class='red' name='val' value='%d'>Freigeben (%d %%)</button>"
        "</form>", SOC_NORMAL, SOC_NORMAL, SOC_RELEASED, SOC_RELEASED);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<p style='margin-top:30px;'><a href='/'>Werte aktualisieren</a> | "
        "<a href='/info'>Info</a> | "
        "<a href='/options'>Optionen</a></p>", HTTPD_RESP_USE_STRLEN);

    // Seite automatisch neu laden (nicht, solange ein Eingabefeld aktiv ist)
    snprintf(line, sizeof(line),
        "<script>setInterval(function(){var a=document.activeElement;"
        "if(a&&a.tagName==='INPUT')return;location.reload();},%u);</script>",
        (unsigned)((g_poll_interval_s < 5 ? 5 : g_poll_interval_s) * 1000));
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// Infoseite (/info)
static esp_err_t info_get_handler(httpd_req_t *req) {
    if (!guard(req, false)) return ESP_OK;

    soc_cache_t c = cache_get();
    char age_str[48], eth_ip[16], line[640];

    if (c.updated_us > 0) {
        long age_s = (long)((esp_timer_get_time() - c.updated_us) / 1000000);
        snprintf(age_str, sizeof(age_str), "vor %ld s", age_s);
    } else {
        snprintf(age_str, sizeof(age_str), s_modbus_ready ? "Warte auf erste Abfrage..." : "Modbus nicht verbunden");
    }
    strlcpy(eth_ip, s_eth_ip, sizeof(eth_ip));

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<h2>Info</h2>", HTTPD_RESP_USE_STRLEN);

    snprintf(line, sizeof(line),
        "<div class='box' style='text-align:left;'>"
        "<table cellpadding='6' style='font-size:17px;'>"
        "<tr><td>Stand der Werte:</td><td><b>%s</b></td></tr>"
        "<tr><td>Abfrage alle:</td><td><b>%u s</b></td></tr>"
        "<tr><td>Sungrow-IP:</td><td><b>%s</b></td></tr>"
        "<tr><td>Ethernet-IP:</td><td><b>%s</b></td></tr>"
        "<tr><td>WLAN-IP:</td><td><b>192.168.4.1</b></td></tr>"
        "</table></div>",
        age_str, (unsigned)g_poll_interval_s, g_mb_ip, eth_ip[0] ? eth_ip : "nicht verbunden");
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<p style='margin-top:30px;'><a href='/'>&larr; Zurück zur Steuerung</a></p>"
        "<script>setTimeout(function(){location.reload();},5000);</script>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// Backup-SOC setzen (POST /set, Feld "val")
static esp_err_t set_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[64], val_str[8];
    if (!read_body(req, body, sizeof(body)) || !form_value(body, "val", val_str, sizeof(val_str))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }

    // Strikt parsen: nur reine Ziffern, Wert 0..100 (kein atoi -> kein ungewolltes 0 bei Müll)
    char *endp = NULL;
    long val = strtol(val_str, &endp, 10);
    if (val_str[0] == '\0' || endp == NULL || *endp != '\0' || val < 0 || val > 100) {
        return send_message_page(req, "Fehler", "Ungültiger Wert. Erlaubt sind ganze Zahlen von 0 bis 100.",
                                 "/", "Zurück");
    }

    esp_err_t err = set_sungrow_backup_soc((uint16_t)val);
    if (err != ESP_OK) {
        return send_message_page(req, "Fehler",
            "Der Wert konnte nicht an den Wechselrichter geschrieben werden. Verbindung und IP prüfen.",
            "/", "Zurück");
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Optionsseite (/options)
static esp_err_t options_get_handler(httpd_req_t *req) {
    if (!guard(req, false)) return ESP_OK;

    char line[768];
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<h2>Optionen</h2>", HTTPD_RESP_USE_STRLEN);

    // --- Netzwerk-Adressen (für Zugriff und OTA-Update) ---
    char eth_ip[16];
    strlcpy(eth_ip, s_eth_ip, sizeof(eth_ip));
    if (eth_ip[0]) {
        snprintf(line, sizeof(line),
            "<div class='box'><h3>Adressen dieses Geräts</h3>"
            "<p style='font-size:15px;'>Ethernet (Heimnetz):<br>"
            "<a href='http://%s/options'>http://%s/options</a></p>"
            "<p style='font-size:15px;'>WLAN-Hotspot:<br>"
            "<a href='http://192.168.4.1/options'>http://192.168.4.1/options</a></p>"
            "</div><br>", eth_ip, eth_ip);
    } else {
        snprintf(line, sizeof(line),
            "<div class='box'><h3>Adressen dieses Geräts</h3>"
            "<p style='font-size:15px;'>Ethernet: <b>nicht verbunden</b></p>"
            "<p style='font-size:15px;'>WLAN-Hotspot:<br>"
            "<a href='http://192.168.4.1/options'>http://192.168.4.1/options</a></p>"
            "</div><br>");
    }
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // --- Modbus-IP ---
    snprintf(line, sizeof(line),
        "<div class='box'><h3>Sungrow IP-Adresse</h3>"
        "<form method='post' action='/config/ip'>"
        "<input type='text' name='ip' value='%s' maxlength='15' pattern='[0-9.]+' inputmode='decimal'><br>"
        "<button class='grn' type='submit'>Speichern &amp; Neustart</button>"
        "</form></div><br>", g_mb_ip);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // --- Wallbox-IP (optional) ---
    snprintf(line, sizeof(line),
        "<div class='box'><h3>Wallbox IP-Adresse</h3>"
        "<p style='font-size:14px;'>Sungrow AC011E-01 / AC22E-01 (Modbus-ID %d, Port %d). "
        "Leer lassen = Wallbox-Abfrage aus.</p>"
        "<form method='post' action='/config/wallbox'>"
        "<input type='text' name='ip' value='%s' maxlength='15' pattern='[0-9.]*' inputmode='decimal'><br>"
        "<button class='grn' type='submit'>Speichern &amp; Neustart</button>"
        "</form></div><br>", WALLBOX_UNIT_ID, WALLBOX_PORT, g_wb_ip);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // --- Abfrage-Intervall ---
    snprintf(line, sizeof(line),
        "<div class='box'><h3>Abfrage-Intervall</h3>"
        "<p style='font-size:14px;'>Wie oft der Wechselrichter per Modbus gelesen wird (%d&ndash;%d Sekunden).</p>"
        "<form method='post' action='/config/poll'>"
        "<input type='number' name='sec' min='%d' max='%d' step='1' value='%u' required "
        "style='padding:10px;font-size:16px;width:90px;text-align:center;'> s<br>"
        "<button class='grn' type='submit'>Speichern</button>"
        "</form></div><br>", POLL_MIN_S, POLL_MAX_S, POLL_MIN_S, POLL_MAX_S, (unsigned)g_poll_interval_s);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // --- Passwort ---
    snprintf(line, sizeof(line),
        "<div class='box'><h3>Passwort ändern</h3>"
        "<p style='font-size:14px;'>Benutzer: <b>%s</b> &middot; %d&ndash;%d Zeichen</p>"
        "<form method='post' action='/config/pass'>"
        "<input type='password' name='pw1' placeholder='Neues Passwort' maxlength='%d' autocomplete='new-password'><br>"
        "<input type='password' name='pw2' placeholder='Wiederholen' maxlength='%d' autocomplete='new-password'><br>"
        "<button class='grn' type='submit'>Passwort speichern</button>"
        "</form></div><br>", WEB_USER, PASS_MIN_LEN, PASS_MAX_LEN, PASS_MAX_LEN, PASS_MAX_LEN);
    httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);

    // --- OTA ---
    httpd_resp_send_chunk(req,
        "<div class='box'><h3>Firmware-Update (OTA)</h3>"
        "<p style='font-size:15px;'>Kompilierte <b>.bin</b> Datei auswählen:</p>"
        "<input type='file' id='fw_file' accept='.bin' style='margin:10px 0;'><br>"
        "<button id='btn_flash' class='grn' onclick='uploadFirmware()'>Firmware flashen</button>"
        "<p id='status' style='margin-top:12px;font-weight:bold;color:#333;'></p></div>"
        "<p style='margin-top:20px;'><a href='/'>&larr; Zurück zur Steuerung</a></p>"
        "<script>"
        "function uploadFirmware(){"
        "const f=document.getElementById('fw_file'),s=document.getElementById('status'),b=document.getElementById('btn_flash');"
        "if(!f.files.length){s.innerText='Bitte zuerst eine .bin Datei auswählen!';return;}"
        "b.disabled=true;s.innerText='Übertrage Firmware... Bitte warten.';"
        "fetch('/ota',{method:'POST',body:f.files[0]})"
        ".then(r=>r.text().then(t=>({ok:r.ok,text:t})))"
        ".then(x=>{s.innerText=x.text;if(x.ok){setTimeout(()=>{window.location.href='/';},8000);}else{b.disabled=false;}})"
        ".catch(e=>{s.innerText='Verbindungsfehler: '+e;b.disabled=false;});}"
        "</script>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// Modbus-IP speichern (POST /config/ip)
static void delayed_restart_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

// Neustart kurz verzögern, damit die HTTP-Antwort noch beim Browser ankommt.
// Falls der Task nicht angelegt werden kann, wird direkt neu gestartet.
static void schedule_restart(void) {
    if (xTaskCreate(delayed_restart_task, "restart", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Neustart-Task nicht erstellbar, starte direkt neu.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

static esp_err_t config_ip_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[64], ip[32];
    if (!read_body(req, body, sizeof(body)) || !form_value(body, "ip", ip, sizeof(ip))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }
    if (!is_valid_ipv4(ip)) {
        return send_message_page(req, "Fehler", "Ungültige IP-Adresse. Erwartet wird z. B. 192.168.188.60.",
                                 "/options", "Zurück zu den Optionen");
    }
    if (config_save_str("mb_ip", ip) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Speichern fehlgeschlagen");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Neue Sungrow-IP gespeichert: %s", ip);

    esp_err_t r = send_message_page(req, "Gespeichert",
        "Die IP wurde gespeichert. Das Gerät startet neu und ist in ca. 10 Sekunden wieder erreichbar.",
        "/", "Zur Startseite");
    schedule_restart();
    return r;
}

// Wallbox-IP speichern (POST /config/wallbox); leer = Wallbox-Abfrage aus
static esp_err_t config_wallbox_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[64], ip[32];
    if (!read_body(req, body, sizeof(body)) || !form_value(body, "ip", ip, sizeof(ip))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }
    if (ip[0] != '\0' && !is_valid_ipv4(ip)) {
        return send_message_page(req, "Fehler", "Ungültige IP-Adresse. Erwartet wird z. B. 192.168.188.70 oder leer.",
                                 "/options", "Zurück zu den Optionen");
    }
    if (config_save_str("wb_ip", ip) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Speichern fehlgeschlagen");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Neue Wallbox-IP gespeichert: %s", ip[0] ? ip : "(aus)");

    esp_err_t r = send_message_page(req, "Gespeichert",
        "Die Wallbox-IP wurde gespeichert. Das Gerät startet neu und ist in ca. 10 Sekunden wieder erreichbar.",
        "/", "Zur Startseite");
    schedule_restart();
    return r;
}

// Abfrage-Intervall speichern (POST /config/poll)
static esp_err_t config_poll_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[48], sec_str[12];
    if (!read_body(req, body, sizeof(body)) || !form_value(body, "sec", sec_str, sizeof(sec_str))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }
    char *endp = NULL;
    long sec = strtol(sec_str, &endp, 10);
    if (sec_str[0] == '\0' || endp == NULL || *endp != '\0' || sec < POLL_MIN_S || sec > POLL_MAX_S) {
        char msg[112];
        snprintf(msg, sizeof(msg), "Ungültiger Wert. Erlaubt sind ganze Zahlen von %d bis %d Sekunden.",
                 POLL_MIN_S, POLL_MAX_S);
        return send_message_page(req, "Fehler", msg, "/options", "Zurück zu den Optionen");
    }
    if (config_save_u32("poll_s", (uint32_t)sec) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Speichern fehlgeschlagen");
        return ESP_FAIL;
    }
    g_poll_interval_s = (uint32_t)sec;
    if (s_modbus_task != NULL) xTaskNotifyGive(s_modbus_task);   // neues Intervall sofort anwenden
    ESP_LOGI(TAG, "Neues Abfrage-Intervall gespeichert: %ld s", sec);

    return send_message_page(req, "Gespeichert", "Das neue Abfrage-Intervall ist sofort aktiv.",
                             "/options", "Zurück zu den Optionen");
}

// Passwort ändern (POST /config/pass)
static esp_err_t config_pass_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[192], pw1[PASS_MAX_LEN + 8], pw2[PASS_MAX_LEN + 8];
    if (!read_body(req, body, sizeof(body)) ||
        !form_value(body, "pw1", pw1, sizeof(pw1)) ||
        !form_value(body, "pw2", pw2, sizeof(pw2))) {
        return send_message_page(req, "Fehler", "Ungültige Eingabe (zu lang oder unvollständig).",
                                 "/options", "Zurück zu den Optionen");
    }
    size_t len = strlen(pw1);
    if (len < PASS_MIN_LEN || len > PASS_MAX_LEN) {
        return send_message_page(req, "Fehler", "Das Passwort muss 8 bis 32 Zeichen lang sein.",
                                 "/options", "Zurück zu den Optionen");
    }
    if (strcmp(pw1, pw2) != 0) {
        return send_message_page(req, "Fehler", "Die beiden Passwörter stimmen nicht überein.",
                                 "/options", "Zurück zu den Optionen");
    }
    if (config_save_str("web_pass", pw1) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Speichern fehlgeschlagen");
        return ESP_FAIL;
    }
    strlcpy(g_web_pass, pw1, sizeof(g_web_pass));
    ESP_LOGI(TAG, "Web-Passwort geändert.");

    return send_message_page(req, "Passwort geändert",
        "Das neue Passwort ist sofort aktiv. Beim nächsten Seitenaufruf musst du dich neu anmelden.",
        "/", "Zur Startseite");
}

// OTA-Empfang (POST /ota)
static esp_err_t ota_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Leere Datei!");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Keine OTA-Partition gefunden!");
        return ESP_FAIL;
    }
    if (req->content_len > update_partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datei ist größer als die OTA-Partition!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starte OTA-Update auf Partition: %s (Größe: %u Bytes)",
             update_partition->label, (unsigned)req->content_len);

    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin fehlgeschlagen: 0x%x", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA Start fehlgeschlagen!");
        return ESP_FAIL;
    }

    char *buf = malloc(4096);
    if (buf == NULL) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Kein freier Speicher!");
        return ESP_FAIL;
    }

    size_t remaining = req->content_len;
    int timeouts = 0;
    while (remaining > 0) {
        size_t to_read = (remaining < 4096) ? remaining : 4096;
        int received = httpd_req_recv(req, buf, to_read);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 10) received = -1;   // endgültig abbrechen
            else continue;
        }
        if (received <= 0) {
            ESP_LOGE(TAG, "Fehler beim Empfangen der OTA-Daten");
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Übertragung abgebrochen!");
            return ESP_FAIL;
        }
        timeouts = 0;

        err = esp_ota_write(ota_handle, (const void *)buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write fehlgeschlagen: 0x%x", err);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash-Schreibfehler!");
            return ESP_FAIL;
        }
        remaining -= received;
    }
    free(buf);

    if (esp_ota_end(ota_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end fehlgeschlagen (ungültige Image-Datei?)");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Firmware-Datei ungültig oder beschädigt!");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition fehlgeschlagen: 0x%x", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Boot-Partition konnte nicht gesetzt werden!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA-Update erfolgreich! Neustart...");
    httpd_resp_sendstr(req, "Update erfolgreich! Starte WT32-ETH01 neu...");
    schedule_restart();
    return ESP_OK;
}

// Browser fragen automatisch /favicon.ico an: leer beantworten, damit kein 404 im Log erscheint
static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 12;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Webserver konnte nicht gestartet werden!");
        return;
    }

    httpd_uri_t uris[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = root_get_handler },
        { .uri = "/set",         .method = HTTP_POST, .handler = set_post_handler },
        { .uri = "/options",     .method = HTTP_GET,  .handler = options_get_handler },
        { .uri = "/info",        .method = HTTP_GET,  .handler = info_get_handler },
        { .uri = "/config/ip",   .method = HTTP_POST, .handler = config_ip_post_handler },
        { .uri = "/config/pass", .method = HTTP_POST, .handler = config_pass_post_handler },
        { .uri = "/ota",         .method = HTTP_POST, .handler = ota_post_handler },
        { .uri = "/favicon.ico", .method = HTTP_GET,  .handler = favicon_get_handler },
        { .uri = "/config/poll", .method = HTTP_POST, .handler = config_poll_post_handler },
        { .uri = "/config/wallbox", .method = HTTP_POST, .handler = config_wallbox_post_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
}

// =====================================================
//                 Ethernet / WLAN / Events
// =====================================================
static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Ethernet IP erhalten: " IPSTR, IP2STR(&ev->ip_info.ip));
    snprintf(s_eth_ip, sizeof(s_eth_ip), IPSTR, IP2STR(&ev->ip_info.ip));
    xEventGroupSetBits(s_net_events, ETH_GOT_IP_BIT);
}

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (id == ETHERNET_EVENT_DISCONNECTED) {
        s_eth_ip[0] = '\0';
        ESP_LOGW(TAG, "Ethernet-Link getrennt");
    } else if (id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet-Link verbunden");
    }
}

static esp_netif_t *init_ethernet(void) {
    gpio_reset_pin(ETH_POWER_GPIO);
    gpio_set_direction(ETH_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num  = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = 0;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = -1;

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_LOGI(TAG, "Starte Ethernet (esp_eth_start)...");
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(TAG, "esp_eth_start fertig.");
    return eth_netif;
}

static void init_wifi_softap(void) {
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .channel = AP_WIFI_CHANNEL,
            .password = AP_WIFI_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    if (strlen(AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Sendeleistung begrenzen (Einheit 0,25 dBm; 40 = 10 dBm): kleinere Stromspitzen
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));

    ESP_LOGI(TAG, "WLAN-Hotspot gestartet. SSID: %s | IP: 192.168.4.1", AP_WIFI_SSID);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    config_load();
    ESP_LOGI(TAG, "Konfiguration: Sungrow-IP = %s, Wallbox-IP = %s", g_mb_ip, g_wb_ip[0] ? g_wb_ip : "(aus)");

    s_mb_mutex    = xSemaphoreCreateMutex();
    s_cache_mutex = xSemaphoreCreateMutex();
    s_net_events  = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Ethernet starten (kein festes Warten: Modbus-Task wartet auf das GOT_IP-Event)
    s_eth_netif = init_ethernet();
    ESP_LOGI(TAG, "Ethernet-Init abgeschlossen, starte WLAN...");

    // WLAN-Hotspot parallel starten
    init_wifi_softap();
    ESP_LOGI(TAG, "WLAN-Init abgeschlossen, starte Modbus-Task...");

    // Modbus-Task: wartet auf IP, initialisiert Modbus, pollt zyklisch
    xTaskCreate(modbus_task, "modbus_task", 6144, NULL, 5, &s_modbus_task);

    // Webserver lauscht auf Ethernet UND WLAN-Hotspot
    start_webserver();
    ESP_LOGI(TAG, "Webserver gestartet.");

    // Rollback-Schutz: erst jetzt gilt die Firmware als gültig
    // (wirkt nur, wenn CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE gesetzt ist)
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (running != NULL && esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "Neue Firmware als gültig markiert.");
    }
}
