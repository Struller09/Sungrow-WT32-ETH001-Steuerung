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
#include <stdarg.h>
#include <stdint.h>
#include <limits.h>

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
#define REG_CLOCK            4999    // Holding: Jahr, Monat, Tag, Stunde, Minute, Sekunde der Wechselrichter-Uhr

// Weitere Wechselrichter-Werte (0-basiert = Protokolladresse - 1)
#define REG_SYS_STATE        12999   // Systemzustand (U16)
#define REG_PV_DAY           13001   // PV-Erzeugung: Tag (U16) + Gesamt (U32), 0,1 kWh [3 Register]
#define REG_PV_POWER         5016    // PV-Leistung, U32, W
#define REG_GRID_POWER       13009   // Netzleistung, S32, W
#define REG_CO2              13014   // CO2-Einsparung, U32, 0,1 kg
#define REG_BAT_VOLTAGE      13019   // Batteriespannung, U16, 0,1 V
#define REG_BAT_SOH          13023   // Batterie-Gesundheit, U16, 0,1 % (danach: Temperatur, S16, 0,1 Grad)
#define REG_SELF_USE         13028   // Eigenverbrauchsquote heute, U16, 0,1 %
#define REG_GRID_STATE       13029   // 0x55 = am Netz, 0xAA = Inselbetrieb
#define REG_PHASE_I          13030   // Phasenströme, 3 x S16, 0,1 A
#define REG_BAT_DAY_CHARGE   13039   // Batterie geladen heute, U16, 0,1 kWh
#define REG_EXPORT_ENERGY    13044   // Einspeisung: Tag (U16) + Gesamt (U32), 0,1 kWh [3 Register]
#define REG_FAULTS           13049   // 14 Fehler-/Alarmgruppen zu je 32 Bit [28 Register]
#define REG_GRID_FREQ        5035    // Netzfrequenz, U16, 0,1 Hz
#define REG_GRID_VOLT        5018    // Netzspannungen, 3 x U16, 0,1 V
#define REG_INV_TEMP         5007    // Innentemperatur, S16, 0,1 Grad
#define REG_DEV_TYPE         4999    // Gerätetyp-Code (Input Register)
#define REG_SERIAL           4989    // Seriennummer, 10 Register ASCII
#define REG_FW_ARM           4953    // Firmware, 15 Register ASCII
#define REG_MAX_SOC          13057   // Holding: Max-SOC, U16, 0,1 % (70.0..100.0)
#define REG_MIN_SOC          13058   // Holding: Min-SOC, U16, 0,1 % (0.0..50.0)
// Netzleistung: 1 = positiver Rohwert bedeutet Einspeisung (negativ = Netzbezug), 0 = umgekehrt
#define GRID_RAW_POSITIVE_IS_EXPORT  1

// Wallbox Sungrow AC011E-01 / AC22E-01 (Modbus TCP über eigene IP, hier nur lesend)
#define WALLBOX_PORT         502
#define WALLBOX_UNIT_ID      248
#define WB_REG_MODE          21262   // Input: 0 Netzwerk, 2 Plug&Play, 6 EMS
#define WB_REG_PHASE_STATE   21269   // Input: 0 dreiphasig, 1 einphasig
#define WB_REG_POWER         21307   // Input: 32 Bit (niederwertiges Wort zuerst), Watt
#define WB_REG_STATE         21316   // Input: 1..9, siehe wb_state_text()
#define WB_REG_SET_CURRENT   21202   // Holding: Ladestrom-Sollwert (Einheit noch zu prüfen)
#define WB_REG_TOTAL_ENERGY  21299   // Input: 32 Bit (niederwertiges Wort zuerst), Wh

// Erlaubte Backup-SOC-Werte für die Buttons
#define SOC_NORMAL           20
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
// Einstellungen für Berechnungen (in der Weboberfläche änderbar)
static volatile uint32_t g_bat_kwh10  = 0;   // Batteriekapazität in 0,1 kWh (0 = aus)
static volatile uint32_t g_price_buy  = 0;   // Strompreis in 0,01 ct/kWh (0 = aus)
static volatile uint32_t g_price_feed = 0;   // Einspeisevergütung in 0,01 ct/kWh (0 = aus)
static volatile bool s_eth_link = false;     // Ethernet-Link aktiv
static int s_run_state = -1;                 // zuletzt gelesener Batterie-Betriebszustand (-1 = unbekannt)

// ---- Modbus-Zustand ----
static esp_netif_t *s_eth_netif = NULL;
static char s_eth_ip[16] = "";              // per DHCP erhaltene Ethernet-IP ("" = keine)
static volatile bool s_modbus_ready = false;
static SemaphoreHandle_t s_mb_mutex = NULL;     // serialisiert alle Modbus-Zugriffe
static SemaphoreHandle_t s_cache_mutex = NULL;  // schützt den Cache
static EventGroupHandle_t s_net_events = NULL;

#define UNK  INT32_MIN   // "unbekannt" bei Messwerten

typedef struct {
    bool valid;            // Wallbox erreichbar und Status gelesen
    int  state;            // 1..9 (siehe wb_state_text)
    int  power_w;          // -1 = unbekannt
    int  cur_da[3];        // Phasenströme in 0,1 A, -1 = unbekannt
    int  volt_dv[3];       // Phasenspannungen in 0,1 V, -1 = unbekannt
    int  phases;           // 1 oder 3, 0 = unbekannt
    int  mode;             // 0 / 2 / 6, -1 = unbekannt
    int  setpoint;         // Sollwert-Register roh, -1 = unbekannt
    int  total_wh;         // Gesamtzähler in Wh, -1 = unbekannt
} wb_data_t;

// Erweiterte Wechselrichter-Werte (UNK = unbekannt)
typedef struct {
    // Zustand
    int sys_state;            // Systemzustand (Rohwert)
    int grid_state;           // 0x55 = am Netz, 0xAA = Inselbetrieb
    int run_state;            // Betriebszustands-Bits
    // PV und Netz
    int pv_power_w;
    int pv_day_dkwh, pv_total_dkwh;
    int grid_w;               // > 0 Netzbezug, < 0 Einspeisung
    int imp_day_dkwh, imp_total_dkwh;
    int exp_day_dkwh, exp_total_dkwh;
    // Haus
    int load_w;
    int day_direct_dkwh;      // heute direkt aus PV verbraucht
    int day_dis_dkwh;         // heute aus der Batterie entladen
    int year_cons_dkwh;       // Jahresverbrauch
    uint32_t year_since;      // Beginn der Jahreszählung (JJJJMMTT), 0 = unbekannt
    int self_use_d;           // Eigenverbrauchsquote PV heute (0,1 %)
    // Batterie
    int bat_v_dv, bat_soh_d, bat_temp_dc;
    int bat_day_charge_dkwh;
    int min_soc_d, max_soc_d; // Entladegrenze / Ladegrenze (0,1 %)
    // Netz-Qualität und Sonstiges
    int grid_freq_dhz, inv_temp_dc;
    int phase_i_da[3], grid_v_dv[3];
    int co2_dkg;
    // Fehlerregister
    bool faults_valid;
    uint32_t faults[14];
    // Uhr und Geräteinfo
    bool clock_valid;
    uint16_t clock[6];        // Jahr, Monat, Tag, Stunde, Minute, Sekunde
    bool dev_valid;
    int  dev_type;
    char serial[24];
    char fw[32];
} inv_ext_t;

typedef struct {
    float   bat_soc;       // -1 = ungültig
    int     backup_soc;    // -1 = ungültig
    int64_t updated_us;    // 0 = noch nie erfolgreich
    int     bat_power_w;   // > 0 = Laden, < 0 = Entladen
    bool    power_valid;
    float   bat_soc_min;   // niedrigster Stand seit Start, -1 = unbekannt
    wb_data_t  wb;         // Wallbox
    inv_ext_t  ext;        // weitere Wechselrichter-Werte
} soc_cache_t;
static soc_cache_t s_cache;

static void ext_init(inv_ext_t *e) {
    memset(e, 0, sizeof(*e));
    e->sys_state = e->grid_state = e->run_state = UNK;
    e->pv_power_w = e->pv_day_dkwh = e->pv_total_dkwh = UNK;
    e->grid_w = e->imp_day_dkwh = e->imp_total_dkwh = e->exp_day_dkwh = e->exp_total_dkwh = UNK;
    e->load_w = e->day_direct_dkwh = e->day_dis_dkwh = e->year_cons_dkwh = e->self_use_d = UNK;
    e->bat_v_dv = e->bat_soh_d = e->bat_temp_dc = e->bat_day_charge_dkwh = UNK;
    e->min_soc_d = e->max_soc_d = UNK;
    e->grid_freq_dhz = e->inv_temp_dc = e->co2_dkg = e->dev_type = UNK;
    for (int i = 0; i < 3; i++) {
        e->phase_i_da[i] = UNK;
        e->grid_v_dv[i] = UNK;
    }
}

static void cache_init(void) {
    memset(&s_cache, 0, sizeof(s_cache));
    s_cache.bat_soc = -1.0f;
    s_cache.backup_soc = -1;
    s_cache.bat_soc_min = -1.0f;
    s_cache.wb.power_w = -1;
    s_cache.wb.mode = -1;
    s_cache.wb.setpoint = -1;
    s_cache.wb.total_wh = -1;
    for (int i = 0; i < 3; i++) {
        s_cache.wb.cur_da[i] = -1;
        s_cache.wb.volt_dv[i] = -1;
    }
    ext_init(&s_cache.ext);
}

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
    uint32_t v32 = 0;
    if (nvs_get_u32(h, "cap10", &v32) == ESP_OK && v32 <= 2000)   g_bat_kwh10 = v32;
    if (nvs_get_u32(h, "pbuy", &v32) == ESP_OK && v32 <= 20000)   g_price_buy = v32;
    if (nvs_get_u32(h, "pfeed", &v32) == ESP_OK && v32 <= 20000)  g_price_feed = v32;
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
    // Statistik (Info-Seite)
    uint32_t    ok, err, exc, connects;
    esp_err_t   last_err;
    int64_t     last_err_us;
} mb_dev_t;

static mb_dev_t s_inv = { .name = "Wechselrichter", .ip = g_mb_ip, .port = SUNGROW_PORT,
                          .unit_id = SUNGROW_SLAVE_ID, .sock = -1 };
static mb_dev_t s_wb  = { .name = "Wallbox", .ip = g_wb_ip, .port = WALLBOX_PORT,
                          .unit_id = WALLBOX_UNIT_ID, .sock = -1 };

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
    dev->connects++;
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

// Modbus-Exceptions (z. B. "Register nicht vorhanden") nur einmal pro Register melden
static void log_exception_once(mb_dev_t *dev, uint8_t fc, uint8_t code, uint16_t reg) {
    static uint32_t seen[32];
    static int count = 0;
    uint32_t key = ((dev == &s_wb) ? 0x1000000u : 0u) | ((uint32_t)fc << 16) | reg;
    for (int i = 0; i < count; i++) {
        if (seen[i] == key) return;
    }
    if (count < 32) seen[count++] = key;
    ESP_LOGW(TAG, "Modbus-Exception (%s): FC 0x%02x, Code 0x%02x, Register %u (wird nicht erneut gemeldet)",
             dev->name, fc, code, reg);
}

// Eine Modbus-TCP-Transaktion (FC 0x03/0x04 lesen, FC 0x06 schreiben).
// arg = Anzahl Register (FC 03/04) bzw. zu schreibender Wert (FC 06).
// result: Array mit arg Werten (FC 03/04, max. 56 Register) bzw. zurückgemeldeter Wert (FC 06).
static esp_err_t mb_transaction_raw(mb_dev_t *dev, uint8_t fc, uint16_t reg, uint16_t arg, uint16_t *result) {
    if ((fc == 0x03 || fc == 0x04) && (arg == 0 || arg > 56)) return ESP_ERR_INVALID_ARG;
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

    uint8_t rsp[128];
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
        log_exception_once(dev, fc, len >= 3 ? rsp[8] : 0, reg);
        return ESP_ERR_NOT_SUPPORTED;                  // Verbindung bleibt bestehen
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

// Zählt Erfolge, Fehler und nicht unterstützte Register mit (Anzeige auf der Info-Seite)
static esp_err_t mb_transaction(mb_dev_t *dev, uint8_t fc, uint16_t reg, uint16_t arg, uint16_t *result) {
    esp_err_t e = mb_transaction_raw(dev, fc, reg, arg, result);
    if (e == ESP_OK) {
        dev->ok++;
    } else if (e == ESP_ERR_NOT_SUPPORTED) {
        dev->exc++;
    } else {
        dev->err++;
        dev->last_err = e;
        dev->last_err_us = esp_timer_get_time();
    }
    return e;
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
    s_run_state = have_state ? (int)state : -1;

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
    if (bat >= 0.0f && (s_cache.bat_soc_min < 0.0f || bat < s_cache.bat_soc_min)) s_cache.bat_soc_min = bat;
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
static const uint16_t WB_REG_VOLTAGES[3] = { 21301, 21303, 21305 };   // Input, 0,1 V je Phase

// Liest alle Wallbox-Werte. Nur der Status ist Pflicht, alle anderen Werte sind optional.
static bool wallbox_read(wb_data_t *d) {
    uint16_t r[2] = { 0, 0 };
    memset(d, 0, sizeof(*d));
    d->power_w = -1;
    d->cur_da[0] = d->cur_da[1] = d->cur_da[2] = -1;
    d->mode = -1;
    d->setpoint = -1;
    d->volt_dv[0] = d->volt_dv[1] = d->volt_dv[2] = -1;
    d->total_wh = -1;

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
    for (int i = 0; i < 3; i++) {
        if (mb_transaction(&s_wb, 0x04, WB_REG_VOLTAGES[i], 1, r) == ESP_OK) d->volt_dv[i] = r[0];
    }
    if (mb_transaction(&s_wb, 0x04, WB_REG_TOTAL_ENERGY, 2, r) == ESP_OK) {
        d->total_wh = (int)(((uint32_t)r[1] << 16) | r[0]);
    }

    d->valid = true;
    return true;
}

static void cache_update_wallbox(const wb_data_t *d) {
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.wb = *d;
    xSemaphoreGive(s_cache_mutex);
}

// ---- Erweiterte Wechselrichter-Werte ----
// 32-Bit-Werte: niederwertiges Wort zuerst
static uint32_t u32_le(const uint16_t *r) {
    return ((uint32_t)r[1] << 16) | r[0];
}

// Lesen am Wechselrichter, true bei Erfolg
static bool rd(uint8_t fc, uint16_t reg, uint16_t n, uint16_t *r) {
    return mb_transaction(&s_inv, fc, reg, n, r) == ESP_OK;
}

// Tageswert (1 Register) und Gesamtzähler (2 Register) ab Startregister, jeweils 0,1 kWh
static bool read_energy_triple(uint16_t reg, int *day, int *total) {
    uint16_t r[3];
    if (!rd(0x04, reg, 3, r)) {
        *day = UNK;
        *total = UNK;
        return false;
    }
    *day = r[0];
    *total = (int)u32_le(&r[1]);
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

// Zeichen aus Registern (2 Zeichen pro Register, höheres Byte zuerst)
static void ascii_from_regs(const uint16_t *r, int n, char *out, size_t outsz) {
    size_t k = 0;
    for (int i = 0; i < n && k + 2 < outsz; i++) {
        char hi = (char)(r[i] >> 8);
        char lo = (char)(r[i] & 0xFF);
        if (hi >= 32 && hi < 127 && hi != '"' && hi != '\\') out[k++] = hi;
        if (lo >= 32 && lo < 127 && lo != '"' && lo != '\\') out[k++] = lo;
    }
    out[k] = '\0';
}

#define SLOW_INTERVAL_US  (60LL * 1000000LL)   // langsam veränderliche Werte nur jede Minute
static int64_t s_last_slow_us = 0;

// Ein Abfragezyklus für den Wechselrichter (Aufruf mit gehaltenem s_mb_mutex).
// Bricht bei Verbindungsproblemen früh ab, damit nicht jede Abfrage einzeln in den Timeout läuft.
static void inverter_poll(void) {
    inv_ext_t e;
    uint16_t r[28];
    uint16_t fwr[15];
    int tot_direct = UNK, tot_dis = UNK, tot_imp = UNK;
    bool ok_dir, ok_dis, ok_imp;
    float bat;
    int backup;
    int watts = 0;
    bool power_ok;
    const uint32_t err0 = s_inv.err;
    const int64_t now = esp_timer_get_time();
    const bool slow = (s_last_slow_us == 0) || (now - s_last_slow_us >= SLOW_INTERVAL_US);
#define COMM_FAIL() (s_inv.err != err0)

    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    e = s_cache.ext;
    xSemaphoreGive(s_cache_mutex);

    // ---- schnelle Werte (jeder Zyklus) ----
    bat = modbus_read_battery_soc_locked();
    if (COMM_FAIL()) goto done;          // Wechselrichter nicht erreichbar: sofort abbrechen
    backup = modbus_read_backup_soc_locked();
    power_ok = modbus_read_battery_power_locked(&watts);
    cache_update(bat, backup);
    cache_update_power(power_ok, watts);
    if (COMM_FAIL()) goto done;

    e.run_state = (s_run_state >= 0) ? s_run_state : UNK;
    e.load_w     = rd(0x04, REG_LOAD_POWER, 2, r) ? (int)(int32_t)u32_le(r) : UNK;
    e.pv_power_w = rd(0x04, REG_PV_POWER, 2, r)   ? (int)u32_le(r) : UNK;
    e.grid_state = rd(0x04, REG_GRID_STATE, 1, r) ? (int)r[0] : UNK;
    e.sys_state  = rd(0x04, REG_SYS_STATE, 1, r)  ? (int)r[0] : UNK;
    if (rd(0x04, REG_GRID_POWER, 2, r)) {
        // Rohwert mit Vorzeichen; Ergebnis: > 0 = Netzbezug, < 0 = Einspeisung
        int raw = (int)(int32_t)u32_le(r);
        e.grid_w = GRID_RAW_POSITIVE_IS_EXPORT ? -raw : raw;
    } else {
        e.grid_w = UNK;
    }
    if (COMM_FAIL() || !slow) goto done;

    // ---- langsam veränderliche Werte (jede Minute) ----
    if (rd(0x04, REG_PV_DAY, 3, r)) {
        e.pv_day_dkwh = r[0];
        e.pv_total_dkwh = (int)u32_le(&r[1]);
    } else {
        e.pv_day_dkwh = e.pv_total_dkwh = UNK;
    }
    ok_dir = read_energy_triple(REG_DIRECT_ENERGY, &e.day_direct_dkwh, &tot_direct);
    ok_dis = read_energy_triple(REG_DISCHARGE_ENERGY, &e.day_dis_dkwh, &tot_dis);
    ok_imp = read_energy_triple(REG_IMPORT_ENERGY, &e.imp_day_dkwh, &tot_imp);
    e.imp_total_dkwh = tot_imp;
    read_energy_triple(REG_EXPORT_ENERGY, &e.exp_day_dkwh, &e.exp_total_dkwh);
    if (COMM_FAIL()) goto done;

    // Uhr des Wechselrichters und Jahresverbrauch (aus den Gesamtzählern)
    e.year_cons_dkwh = UNK;
    if (rd(0x03, REG_CLOCK, 6, r)) {
        for (int i = 0; i < 6; i++) e.clock[i] = r[i];
        e.clock_valid = true;
    } else {
        e.clock_valid = false;
    }
    if (ok_dir && ok_dis && ok_imp && e.clock_valid) {
        uint32_t y = e.clock[0], mo = e.clock[1], d = e.clock[2];
        if (y >= 2020 && y <= 2100 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
            uint32_t total = (uint32_t)(tot_direct + tot_dis + tot_imp);
            year_update(total, y, y * 10000 + mo * 100 + d);
            if (total >= s_year_base) {
                e.year_cons_dkwh = (int)(total - s_year_base);
                e.year_since = s_year_since;
            }
        }
    }
    if (COMM_FAIL()) goto done;

    // Batterie
    e.bat_v_dv = rd(0x04, REG_BAT_VOLTAGE, 1, r) ? (int)r[0] : UNK;
    if (rd(0x04, REG_BAT_SOH, 2, r)) {
        e.bat_soh_d = r[0];
        e.bat_temp_dc = (int)(int16_t)r[1];
    } else {
        e.bat_soh_d = e.bat_temp_dc = UNK;
    }
    e.bat_day_charge_dkwh = rd(0x04, REG_BAT_DAY_CHARGE, 1, r) ? (int)r[0] : UNK;
    e.self_use_d = rd(0x04, REG_SELF_USE, 1, r) ? (int)r[0] : UNK;
    if (rd(0x03, REG_MAX_SOC, 2, r)) {
        e.max_soc_d = r[0];
        e.min_soc_d = r[1];
    } else {
        e.max_soc_d = e.min_soc_d = UNK;
    }
    if (COMM_FAIL()) goto done;

    // Netz, Temperatur, CO2
    e.grid_freq_dhz = rd(0x04, REG_GRID_FREQ, 1, r) ? (int)r[0] : UNK;
    e.inv_temp_dc   = rd(0x04, REG_INV_TEMP, 1, r) ? (int)(int16_t)r[0] : UNK;
    if (rd(0x04, REG_GRID_VOLT, 3, r)) {
        for (int i = 0; i < 3; i++) e.grid_v_dv[i] = r[i];
    } else {
        for (int i = 0; i < 3; i++) e.grid_v_dv[i] = UNK;
    }
    if (rd(0x04, REG_PHASE_I, 3, r)) {
        for (int i = 0; i < 3; i++) e.phase_i_da[i] = (int)(int16_t)r[i];
    } else {
        for (int i = 0; i < 3; i++) e.phase_i_da[i] = UNK;
    }
    e.co2_dkg = rd(0x04, REG_CO2, 2, r) ? (int)u32_le(r) : UNK;
    if (COMM_FAIL()) goto done;

    // Fehler- und Alarmregister (14 x 32 Bit)
    if (rd(0x04, REG_FAULTS, 28, r)) {
        for (int i = 0; i < 14; i++) e.faults[i] = u32_le(&r[2 * i]);
        e.faults_valid = true;
    } else {
        e.faults_valid = false;
    }
    if (COMM_FAIL()) goto done;

    // Geräteinformationen (nur einmal)
    if (!e.dev_valid) {
        e.dev_type = rd(0x04, REG_DEV_TYPE, 1, r) ? (int)r[0] : UNK;
        if (rd(0x04, REG_SERIAL, 10, r)) ascii_from_regs(r, 10, e.serial, sizeof(e.serial));
        if (rd(0x04, REG_FW_ARM, 15, fwr)) ascii_from_regs(fwr, 15, e.fw, sizeof(e.fw));
        e.dev_valid = true;
    }
    s_last_slow_us = now;

done:
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    s_cache.ext = e;
    xSemaphoreGive(s_cache_mutex);
#undef COMM_FAIL
}

// ---- Verlauf: ein Messpunkt alle 5 Minuten, 24 Stunden im RAM ----
#define HIST_N        288
#define HIST_STEP_US  (300LL * 1000000LL)

typedef struct {
    int16_t soc;    // 0,1 %
    int16_t pv;     // W
    int16_t load;   // W
    int16_t grid;   // W (+ Bezug, - Einspeisung)
    int16_t bat;    // W (+ Laden, - Entladen)
} hist_t;            // INT16_MIN = unbekannt

static hist_t  s_hist[HIST_N];
static int     s_hist_head = 0;     // Index des nächsten Eintrags
static int     s_hist_count = 0;
static int64_t s_hist_last_us = 0;

static int16_t clamp16(int v) {
    if (v == UNK) return INT16_MIN;
    if (v > 32000) return 32000;
    if (v < -32000) return -32000;
    return (int16_t)v;
}

static void history_tick(void) {
    int64_t now = esp_timer_get_time();
    if (s_hist_last_us != 0 && now - s_hist_last_us < HIST_STEP_US) return;

    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    if (s_cache.updated_us != 0) {
        hist_t h;
        h.soc  = (s_cache.bat_soc >= 0.0f) ? (int16_t)(s_cache.bat_soc * 10.0f + 0.5f) : INT16_MIN;
        h.pv   = clamp16(s_cache.ext.pv_power_w);
        h.load = clamp16(s_cache.ext.load_w);
        h.grid = clamp16(s_cache.ext.grid_w);
        h.bat  = s_cache.power_valid ? clamp16(s_cache.bat_power_w) : INT16_MIN;
        s_hist[s_hist_head] = h;
        s_hist_head = (s_hist_head + 1) % HIST_N;
        if (s_hist_count < HIST_N) s_hist_count++;
        s_hist_last_us = now;
    }
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
            inverter_poll();
            xSemaphoreGive(s_mb_mutex);
        }

        // Wallbox (eigene Verbindung), nur wenn eine IP eingetragen ist
        if (g_wb_ip[0] != '\0') {
            wb_data_t wb;
            wallbox_read(&wb);
            cache_update_wallbox(&wb);
        }

        history_tick();

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
    "<link rel='apple-touch-icon' href='/apple-touch-icon.png'>"
    "<meta name='apple-mobile-web-app-title' content='Solar-App'>"
    "<style>"
    "body{font-family:sans-serif;text-align:center;margin:30px 10px;}"
    ".box{background:#f4f4f4;display:block;padding:20px;border-radius:10px;margin:0 auto 20px auto;max-width:420px;width:90%;}"
    ".box p{font-size:18px;margin:8px 0;}"
    "button{padding:14px 24px;font-size:16px;border:none;border-radius:5px;cursor:pointer;margin:6px;}"
    "input[type=text],input[type=password]{padding:10px;font-size:16px;width:80%;margin:6px 0;}"
    ".grn{background:#28a745;color:#fff;}.red{background:#d9534f;color:#fff;}.gry{background:#ddd;}"
    ".warn{background:#fff3cd;color:#856404;padding:10px;border-radius:8px;display:inline-block;margin-bottom:15px;}"
    ".bad{color:#d9534f;font-weight:bold;}"
    ".alert{background:#f8d7da;color:#721c24;font-weight:bold;padding:10px;border-radius:8px;max-width:420px;width:90%;margin:0 auto 15px auto;}"
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
// =====================================================
//     Hilfen, JSON-Schnittstellen und Seiten
// =====================================================
static bool known(int v) { return v != UNK; }

__attribute__((format(printf, 2, 3)))
static void send_fmt(httpd_req_t *req, const char *fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
}

// Dezimalzahl mit Punkt oder Komma, leer = 0
static bool parse_decimal(const char *s, double lo, double hi, double *out) {
    char tmp[24];
    strlcpy(tmp, s, sizeof(tmp));
    for (char *p = tmp; *p; p++) {
        if (*p == ',') *p = '.';
    }
    if (tmp[0] == '\0') {
        *out = 0.0;
        return true;
    }
    char *end = NULL;
    double v = strtod(tmp, &end);
    if (end == tmp || *end != '\0' || v < lo || v > hi) return false;
    *out = v;
    return true;
}

static const char *dev_type_name(int t) {
    switch (t) {
        case 0xE00: return "SH5.0RT";
        case 0xE01: return "SH6.0RT";
        case 0xE02: return "SH8.0RT";
        case 0xE03: return "SH10RT";
        default:    return "";
    }
}

// ---- kleiner JSON-Schreiber ----
typedef struct {
    char  *b;
    size_t cap;
    size_t off;
} jb_t;

__attribute__((format(printf, 2, 3)))
static void jb_raw(jb_t *j, const char *fmt, ...) {
    if (j->off + 1 >= j->cap) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(j->b + j->off, j->cap - j->off, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= j->cap - j->off) j->off = j->cap - 1;
    else j->off += (size_t)n;
}

// Zahl oder null (UNK); div = 1 -> ganzzahlig, sonst eine Nachkommastelle
static void jb_num(jb_t *j, const char *key, int v, int div) {
    if (v == UNK)       jb_raw(j, "\"%s\":null,", key);
    else if (div == 1)  jb_raw(j, "\"%s\":%d,", key, v);
    else                jb_raw(j, "\"%s\":%.1f,", key, v / (double)div);
}

static void jb_num3(jb_t *j, const char *key, const int *v, int div) {
    jb_raw(j, "\"%s\":[", key);
    for (int i = 0; i < 3; i++) {
        if (v[i] == UNK)      jb_raw(j, "%snull", i ? "," : "");
        else if (div == 1)    jb_raw(j, "%s%d", i ? "," : "", v[i]);
        else                  jb_raw(j, "%s%.1f", i ? "," : "", v[i] / (double)div);
    }
    jb_raw(j, "],");
}

static void jb_bool(jb_t *j, const char *key, bool v) {
    jb_raw(j, "\"%s\":%s,", key, v ? "true" : "false");
}

static void jb_str(jb_t *j, const char *key, const char *s) {
    jb_raw(j, "\"%s\":\"%s\",", key, s ? s : "");
}

static void jb_close(jb_t *j) {
    if (j->off > 0 && j->b[j->off - 1] == ',') j->off--;
    jb_raw(j, "}");
}

static int unk_if_neg(int v) { return v < 0 ? UNK : v; }

static esp_err_t json_send(httpd_req_t *req, jb_t *j) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, j->b, (ssize_t)j->off);
    free(j->b);
    return r;
}

// ---- /api/status: alle Live-Werte der Hauptseite ----
static esp_err_t api_status_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    soc_cache_t c = cache_get();
    const inv_ext_t *x = &c.ext;
    const size_t cap = 3072;
    char *buf = malloc(cap);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Kein Speicher");
        return ESP_FAIL;
    }
    jb_t j = { buf, cap, 0 };

    bool fault = (x->sys_state == 0x0100);
    if (x->faults_valid) {
        for (int i = 0; i < 14; i++) {
            if (x->faults[i]) fault = true;
        }
    }

    jb_raw(&j, "{");
    jb_bool(&j, "defpw", strcmp(g_web_pass, DEFAULT_WEB_PASS) == 0);
    jb_num(&j, "soc", c.bat_soc >= 0.0f ? (int)(c.bat_soc * 10.0f + 0.5f) : UNK, 10);
    jb_num(&j, "soc_min", c.bat_soc_min >= 0.0f ? (int)(c.bat_soc_min * 10.0f + 0.5f) : UNK, 10);
    jb_num(&j, "backup", c.backup_soc >= 0 ? c.backup_soc : UNK, 1);
    jb_num(&j, "bat_w", c.power_valid ? c.bat_power_w : UNK, 1);
    jb_num(&j, "soh", x->bat_soh_d, 10);
    jb_num(&j, "temp", x->bat_temp_dc, 10);
    jb_num(&j, "bat_v", x->bat_v_dv, 10);
    jb_num(&j, "min_soc", x->min_soc_d, 10);
    jb_num(&j, "pv_w", x->pv_power_w, 1);
    jb_num(&j, "grid_w", x->grid_w, 1);
    jb_num(&j, "load_w", x->load_w, 1);
    jb_num(&j, "grid_state", x->grid_state, 1);
    jb_num(&j, "sys_state", x->sys_state, 1);
    jb_bool(&j, "fault", fault);
    jb_bool(&j, "fault_known", x->faults_valid || x->sys_state != UNK);
    jb_num(&j, "pv_day", x->pv_day_dkwh, 10);
    jb_num(&j, "pv_total", x->pv_total_dkwh, 10);
    jb_num(&j, "imp_day", x->imp_day_dkwh, 10);
    jb_num(&j, "imp_total", x->imp_total_dkwh, 10);
    jb_num(&j, "exp_day", x->exp_day_dkwh, 10);
    jb_num(&j, "exp_total", x->exp_total_dkwh, 10);
    jb_num(&j, "direct_day", x->day_direct_dkwh, 10);
    jb_num(&j, "dis_day", x->day_dis_dkwh, 10);
    jb_num(&j, "chg_day", x->bat_day_charge_dkwh, 10);
    jb_num(&j, "self_use", x->self_use_d, 10);
    jb_num(&j, "year_cons", x->year_cons_dkwh, 10);
    jb_raw(&j, "\"year_since\":%lu,", (unsigned long)x->year_since);
    jb_raw(&j, "\"cap\":%.1f,", g_bat_kwh10 / 10.0);
    jb_raw(&j, "\"pbuy\":%.2f,", g_price_buy / 100.0);
    jb_raw(&j, "\"pfeed\":%.2f,", g_price_feed / 100.0);

    // Wallbox
    const wb_data_t *w = &c.wb;
    int wi[3] = { unk_if_neg(w->cur_da[0]), unk_if_neg(w->cur_da[1]), unk_if_neg(w->cur_da[2]) };
    int wv[3] = { unk_if_neg(w->volt_dv[0]), unk_if_neg(w->volt_dv[1]), unk_if_neg(w->volt_dv[2]) };
    jb_raw(&j, "\"wb\":{");
    jb_bool(&j, "on", g_wb_ip[0] != '\0');
    jb_bool(&j, "ok", w->valid);
    jb_str(&j, "ip", g_wb_ip);
    jb_num(&j, "state", w->valid ? w->state : UNK, 1);
    jb_num(&j, "pw", unk_if_neg(w->power_w), 1);
    jb_num3(&j, "i", wi, 10);
    jb_num3(&j, "v", wv, 10);
    jb_num(&j, "ph", w->phases > 0 ? w->phases : UNK, 1);
    jb_num(&j, "mode", unk_if_neg(w->mode), 1);
    jb_num(&j, "set", unk_if_neg(w->setpoint), 1);
    jb_num(&j, "tot", unk_if_neg(w->total_wh), 1000);
    jb_close(&j);
    jb_raw(&j, ",");

    jb_close(&j);
    return json_send(req, &j);
}

static void jb_mb(jb_t *j, const char *key, const mb_dev_t *d) {
    long last_age = d->last_err_us ? (long)((esp_timer_get_time() - d->last_err_us) / 1000000) : -1;
    jb_raw(j, "\"%s\":{\"ok\":%lu,\"err\":%lu,\"exc\":%lu,\"conn\":%lu,\"last\":%d,\"last_age\":%ld},",
           key, (unsigned long)d->ok, (unsigned long)d->err, (unsigned long)d->exc,
           (unsigned long)d->connects, (int)d->last_err, last_age);
}

// ---- /api/info: Geräte-, Netz- und Systeminformationen ----
static esp_err_t api_info_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    soc_cache_t c = cache_get();
    const inv_ext_t *x = &c.ext;
    const size_t cap = 3072;
    char *buf = malloc(cap);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Kein Speicher");
        return ESP_FAIL;
    }
    jb_t j = { buf, cap, 0 };

    jb_raw(&j, "{");
    jb_num(&j, "age", c.updated_us > 0 ? (int)((esp_timer_get_time() - c.updated_us) / 1000000) : UNK, 1);
    jb_bool(&j, "ready", s_modbus_ready);
    jb_raw(&j, "\"poll\":%u,", (unsigned)g_poll_interval_s);
    jb_str(&j, "inv_ip", g_mb_ip);
    jb_str(&j, "eth_ip", s_eth_ip);
    jb_str(&j, "wb_ip", g_wb_ip);

    // Wechselrichter
    jb_num(&j, "dev_type", x->dev_type, 1);
    jb_str(&j, "dev_name", known(x->dev_type) ? dev_type_name(x->dev_type) : "");
    jb_str(&j, "serial", x->serial);
    jb_str(&j, "fw", x->fw);
    if (x->clock_valid) {
        jb_raw(&j, "\"clock\":[%u,%u,%u,%u,%u,%u],", x->clock[0], x->clock[1], x->clock[2],
               x->clock[3], x->clock[4], x->clock[5]);
    } else {
        jb_raw(&j, "\"clock\":null,");
    }
    jb_num(&j, "sys_state", x->sys_state, 1);
    jb_num(&j, "grid_state", x->grid_state, 1);
    jb_num(&j, "inv_temp", x->inv_temp_dc, 10);
    jb_num(&j, "co2", x->co2_dkg, 10);

    // Netz
    jb_num(&j, "freq", x->grid_freq_dhz, 10);
    jb_num3(&j, "gv", x->grid_v_dv, 10);
    jb_num3(&j, "gi", x->phase_i_da, 10);
    jb_num(&j, "pv_total", x->pv_total_dkwh, 10);
    jb_num(&j, "imp_total", x->imp_total_dkwh, 10);
    jb_num(&j, "exp_total", x->exp_total_dkwh, 10);

    // Batterie
    jb_num(&j, "bat_v", x->bat_v_dv, 10);
    jb_num(&j, "temp", x->bat_temp_dc, 10);
    jb_num(&j, "soh", x->bat_soh_d, 10);
    jb_num(&j, "min_soc", x->min_soc_d, 10);
    jb_num(&j, "max_soc", x->max_soc_d, 10);
    jb_num(&j, "backup", c.backup_soc >= 0 ? c.backup_soc : UNK, 1);
    jb_num(&j, "soc_min", c.bat_soc_min >= 0.0f ? (int)(c.bat_soc_min * 10.0f + 0.5f) : UNK, 10);

    // Fehlerregister
    if (x->faults_valid) {
        jb_raw(&j, "\"faults\":[");
        for (int i = 0; i < 14; i++) {
            jb_raw(&j, "%s%lu", i ? "," : "", (unsigned long)x->faults[i]);
        }
        jb_raw(&j, "],");
    } else {
        jb_raw(&j, "\"faults\":null,");
    }

    // ESP32
    wifi_sta_list_t sl;
    int clients = (esp_wifi_ap_get_sta_list(&sl) == ESP_OK) ? sl.num : -1;
    const esp_partition_t *rp = esp_ota_get_running_partition();
    jb_raw(&j, "\"up\":%ld,", (long)(esp_timer_get_time() / 1000000));
    jb_raw(&j, "\"heap\":%u,\"heap_min\":%u,", (unsigned)(esp_get_free_heap_size() / 1024),
           (unsigned)(esp_get_minimum_free_heap_size() / 1024));
    jb_str(&j, "build", __DATE__ " " __TIME__);
    jb_str(&j, "idf", esp_get_idf_version());
    jb_str(&j, "part", rp ? rp->label : "?");
    jb_bool(&j, "eth_link", s_eth_link);
    jb_raw(&j, "\"clients\":%d,", clients);

    // Modbus-Statistik
    jb_mb(&j, "mb_inv", &s_inv);
    jb_mb(&j, "mb_wb", &s_wb);

    jb_close(&j);
    return json_send(req, &j);
}

// ---- /api/history ----
static esp_err_t api_history_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    hist_t *h = malloc(sizeof(hist_t) * HIST_N);
    if (!h) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Kein Speicher");
        return ESP_FAIL;
    }
    int n;
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    n = s_hist_count;
    int start = (s_hist_head - n + HIST_N) % HIST_N;
    for (int i = 0; i < n; i++) h[i] = s_hist[(start + i) % HIST_N];
    xSemaphoreGive(s_cache_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    send_fmt(req, "{\"step\":%d,\"n\":%d", (int)(HIST_STEP_US / 1000000), n);

    static const char *const names[5] = { "soc", "pv", "load", "grid", "bat" };
    char buf[400];
    jb_t j = { buf, sizeof(buf), 0 };
    for (int s = 0; s < 5; s++) {
        send_fmt(req, ",\"%s\":[", names[s]);
        j.off = 0;
        for (int i = 0; i < n; i++) {
            int16_t v;
            switch (s) {
                case 0:  v = h[i].soc;  break;
                case 1:  v = h[i].pv;   break;
                case 2:  v = h[i].load; break;
                case 3:  v = h[i].grid; break;
                default: v = h[i].bat;  break;
            }
            if (v == INT16_MIN)  jb_raw(&j, "%snull", i ? "," : "");
            else if (s == 0)     jb_raw(&j, "%s%.1f", i ? "," : "", v / 10.0);
            else                 jb_raw(&j, "%s%d", i ? "," : "", (int)v);
            if (j.off > sizeof(buf) - 32) {
                httpd_resp_send_chunk(req, buf, j.off);
                j.off = 0;
            }
        }
        if (j.off) httpd_resp_send_chunk(req, buf, j.off);
        httpd_resp_send_chunk(req, "]", 1);
    }
    httpd_resp_send_chunk(req, "}", 1);
    free(h);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// =====================================================
//   Hauptseite (/): statisches HTML, Werte kommen per /api/status
// =====================================================
static esp_err_t root_get_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<h2>Sungrow Batterie-Steuerung</h2>"
        "<div id='a_pw' class='warn' style='display:none'>Standard-Passwort aktiv! Bitte unter "
        "<a href='/options'>Optionen</a> &auml;ndern.</div>"
        "<div id='a_grid' class='alert' style='display:none'>NOTBETRIEB: Der Wechselrichter l&auml;uft im Inselbetrieb (kein Netz).</div>"
        "<div id='a_fault' class='alert' style='display:none'>St&ouml;rung gemeldet &ndash; Details unter <a href='/info'>Info</a>.</div>"
        "", HTTPD_RESP_USE_STRLEN);

    // Energie-Übersicht (oberste Box)
    httpd_resp_send_chunk(req,
        "<style>"
        ".ov{max-width:560px!important;padding:12px!important;background:#eef1f5!important;}"
        ".ov h3{margin:2px 0 10px 0;font-size:16px;color:#445;}"
        ".og{display:grid;grid-template-columns:1fr 18px 1fr 18px 1fr;grid-template-rows:auto 22px auto 22px auto;align-items:stretch;}"
        ".oc{background:#fff;border-radius:12px;padding:8px 4px 8px 4px;box-shadow:0 1px 3px rgba(0,0,0,.12);text-align:center;border-top:4px solid var(--c);min-width:0;}"
        ".oc svg{width:42px;height:42px;stroke:var(--c);fill:none;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round;}"
        ".oc .t{font-weight:bold;font-size:13px;color:var(--c);margin:0 0 2px 0;}"
        ".oc .l{font-size:10px;color:#889;margin-top:5px;line-height:1.1;}"
        ".oc .v{font-size:14px;font-weight:normal;color:#223;line-height:1.2;word-break:break-word;}"
        ".oc .v.s{font-size:12px;font-weight:bold;color:#445;}"
        ".oc.ph{opacity:.55;}"
        ".o-sol{grid-area:1/3;--c:#f0a30a;}.o-bat{grid-area:3/1;--c:#2e9e4f;}.o-hou{grid-area:3/3;--c:#3f68a8;}"
        ".o-wb{grid-area:3/5;--c:#16958c;}.o-grd{grid-area:5/3;--c:#7b5ea7;}"
        ".fl{position:relative;justify-self:center;align-self:center;opacity:.25;}"
        ".fl.v{width:6px;height:100%;background:repeating-linear-gradient(to bottom,var(--c) 0 6px,transparent 6px 12px);background-size:6px 12px;}"
        ".fl.h{height:6px;width:100%;background:repeating-linear-gradient(to right,var(--c) 0 6px,transparent 6px 12px);background-size:12px 6px;}"
        ".fl.on{opacity:1;}"
        ".fl.v.on{animation:fdv .7s linear infinite;}.fl.v.on.rev{animation-direction:reverse;}"
        ".fl.h.on{animation:fdh .7s linear infinite;}.fl.h.on.rev{animation-direction:reverse;}"
        "@keyframes fdv{from{background-position:0 0}to{background-position:0 12px}}"
        "@keyframes fdh{from{background-position:0 0}to{background-position:12px 0}}"
        ".f-sol{grid-area:2/3;--c:#f0a30a;}.f-bat{grid-area:3/2;--c:#2e9e4f;}.f-wb{grid-area:3/4;--c:#16958c;}.f-grd{grid-area:4/3;--c:#7b5ea7;}"
        "</style>"
        "<div class='box ov'><h3>Energie&uuml;bersicht &ndash; heute</h3>"
        "<div class='og'>"
        "<div class='oc o-sol'>"
        "<svg viewBox='0 0 48 48'><circle cx='13' cy='12' r='5'/><path d='M13 2v2.5M3 12h2.5M20.5 12H23M6 5l1.8 1.8M20 5l-1.8 1.8M6 19l1.8-1.8M20 19l-1.8-1.8'/><path d='M8 43L14 25H42L36 43Z'/><path d='M23.3 25L17.3 43M32.7 25L26.7 43M11 34H39'/></svg>"
        "<div class='t'>Solar</div><div class='v s' id='o_pvw'>&ndash;</div>"
        "<div class='l'>PV erzeugt</div><div class='v' id='o_pvday'>&ndash;</div>"
        "<div class='l'>Eigenverbrauchsquote</div><div class='v' id='o_self'>&ndash;</div>"
        "</div>"
        "<div class='fl v f-sol' id='f_sol'></div>"
        "<div class='oc o-bat'>"
        "<svg viewBox='0 0 48 48'><rect x='6' y='14' width='30' height='22' rx='3'/><path d='M36 21h4v8h-4'/><rect id='o_fill' x='9' y='17' width='0' height='16' rx='1' style='fill:var(--c);stroke:none'/></svg>"
        "<div class='t'>Batterie</div><div class='v s' id='o_batw'>&ndash;</div>"
        "<div class='l'>Akkustand</div><div class='v' id='o_soc'>&ndash;</div>"
        "<div class='l'>Einspeise-Verdienst</div><div class='v' id='o_rev'>&ndash;</div>"
        "</div>"
        "<div class='fl h f-bat' id='f_bat'></div>"
        "<div class='oc o-hou'>"
        "<svg viewBox='0 0 48 48'><path d='M5 24L24 7L43 24'/><path d='M10 21V41H38V21'/><path d='M34 15V8H39V19'/><rect x='19' y='28' width='10' height='13'/><path d='M24 28v13'/></svg>"
        "<div class='t'>Haus</div><div class='v s' id='o_load'>&ndash;</div>"
        "<div class='l'>Verbrauch</div><div class='v' id='o_cons'>&ndash;</div>"
        "<div class='l'>Autarkie</div><div class='v' id='o_aut'>&ndash;</div>"
        "<div class='l'>Netzbezug-Kosten</div><div class='v' id='o_cost'>&ndash;</div>"
        "<div class='l'>Ersparnis Eigenverbrauch</div><div class='v' id='o_save'>&ndash;</div>"
        "</div>"
        "<div class='fl h f-wb' id='f_wb'></div>"
        "<div class='oc o-wb ph'>"
        "<svg viewBox='0 0 48 48'><rect x='10' y='5' width='20' height='35' rx='4'/><circle cx='20' cy='13' r='2.5'/><path d='M22 20L16 29H21L18 36'/><path d='M30 28H35Q40 28 40 33V38'/><rect x='37' y='38' width='6' height='6' rx='1'/></svg>"
        "<div class='t'>Wallbox</div><div class='v s'>folgt</div>"
        "<div class='l'>Geladen heute</div><div class='v'>&ndash;</div>"
        "<div class='l'>Ladekosten</div><div class='v'>&ndash;</div>"
        "</div>"
        "<div class='fl v f-grd' id='f_grd'></div>"
        "<div class='oc o-grd'>"
        "<svg viewBox='0 0 48 48'><path d='M24 5L14 44M24 5L34 44M8 14H40M12 24H36M19.3 24L31.9 36M28.7 24L16.1 36M16 36H32M10 14v4M38 14v4M14 24v4M34 24v4'/></svg>"
        "<div class='t'>Netz</div><div class='v s' id='o_gw'>&ndash;</div>"
        "<div class='l'>Netzbezug</div><div class='v' id='o_imp'>&ndash;</div>"
        "<div class='l'>Einspeisung</div><div class='v' id='o_exp'>&ndash;</div>"
        "</div>"
        "</div></div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class='box'><h3>Heute</h3>"
        "<p>PV-Erzeugung: <b id='pvday'>&ndash;</b></p>"
        "<p>Netzbezug: <b id='impday'>&ndash;</b></p>"
        "<p>Einspeisung: <b id='expday'>&ndash;</b></p>"
        "<p>Batterie geladen: <b id='chgday'>&ndash;</b></p>"
        "<p>Batterie entladen: <b id='disday'>&ndash;</b></p>"
        "<p>Eigenverbrauchsquote PV: <b id='selfuse'>&ndash;</b></p>"
        "<p id='cost_buy_row' style='display:none'>Netzbezug-Kosten: <b id='cost_buy'></b></p>"
        "<p id='rev_row' style='display:none'>Einspeise-Erl&ouml;s: <b id='rev'></b></p>"
        "<p id='save_row' style='display:none'>Ersparnis durch Eigenverbrauch: <b id='save'></b></p>"
        "</div>"

        "<div class='box'><h3>Gesamt</h3>"
        "<p>PV-Erzeugung: <b id='pvtot'>&ndash;</b></p>"
        "<p>Netzbezug: <b id='imptot'>&ndash;</b></p>"
        "<p>Einspeisung: <b id='exptot'>&ndash;</b></p>"
        "</div>"

        "<div class='box' id='wbbox' style='display:none'><h3>Wallbox</h3>"
        "<p id='wbnoconn' style='display:none'>Keine Verbindung zur Wallbox</p>"
        "<div id='wbdetail'>"
        "<p>Status: <b id='wbstate'>&ndash;</b></p>"
        "<p>Ladeleistung: <b id='wbpw'>&ndash;</b></p>"
        "<p id='wbshare' style='display:none;font-size:13px;color:#666;'></p>"
        "<p>Strom L1/L2/L3: <b id='wbi'>&ndash;</b></p>"
        "<p>Spannung L1/L2/L3: <b id='wbv'>&ndash;</b></p>"
        "<p>Z&auml;hlerstand gesamt: <b id='wbtot'>&ndash;</b></p>"
        "<p id='wbmisc' style='font-size:13px;color:#666;'></p>"
        "</div></div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<p style='margin-top:30px;'><a href='/chart'>Verlauf</a> | "
        "<a href='/info'>Info</a> | <a href='/options'>Optionen</a></p>", HTTPD_RESP_USE_STRLEN);

    // JavaScript: holt alle Werte regelmäßig als JSON und trägt sie ein
    httpd_resp_send_chunk(req,
        "<script>"
        "var first=true;"
        "function $(i){return document.getElementById(i);}"
        "function T(i,s){var e=$(i);if(e)e.innerHTML=s;}"
        "function V(v,f){return(v===null||v===undefined)?'&ndash;':f(v);}"
        "function W(v){return Math.round(v)+' W';}"
        "function K(v){return v.toFixed(1)+' kWh';}"
        "function P(v){return v.toFixed(1)+' %';}"
        "function E(v){return v.toFixed(2)+' &euro;';}"
        "function show(i,on){var e=$(i);if(e)e.style.display=on?'':'none';}"
        "var SYS={2:'Stopp',8:'Standby',16:'Initialisierung',32:'Start',64:'L&auml;uft',256:'ST&Ouml;RUNG',"
        "1024:'Wartungsmodus',2048:'Zwangsbetrieb',4096:'L&auml;uft im Notbetrieb',9473:'Neustart',16384:'Externer EMS-Modus'};"
        "var WBS={1:'Frei (kein Fahrzeug)',2:'Fahrzeug verbunden (Standby)',3:'L&auml;dt',4:'Pausiert (Wallbox)',"
        "5:'Pausiert (Fahrzeug)',6:'Laden beendet',8:'Nicht verf&uuml;gbar',9:'Fehler'};"
        "var WBM={0:'Netzwerk',2:'Plug&amp;Play',6:'EMS'};"
        "function flow(i,on,rev){var e=$(i);if(!e)return;e.className=e.className.replace(/ on| rev/g,'')+(on?' on':'')+(on&&rev?' rev':'');}"
        "function ovflow(d){"
        "var pv=Number(d.pv_w)||0,gr=Number(d.grid_w)||0,bt=Number(d.bat_w)||0,wp=(d.wb&&d.wb.on)?(Number(d.wb.pw)||0):0;"
        "flow('f_sol',pv>20,false);"
        "flow('f_grd',Math.abs(gr)>20,gr>0);"
        "flow('f_bat',Math.abs(bt)>20,bt>0);"
        "flow('f_wb',wp>50,false);"
        "}"
        "function ovfill(d,dc){"
        "try{ovflow(d);}catch(e){}"
        "T('o_pvw',d.pv_w===null?'&ndash;':'aktuell '+Math.round(d.pv_w)+' W');"
        "T('o_pvday',V(d.pv_day,K));T('o_self',V(d.self_use,P));"
        "T('o_load',d.load_w===null?'&ndash;':'aktuell '+Math.round(d.load_w)+' W');"
        "T('o_cons',V(dc,K));"
        "T('o_aut',(dc!==null&&dc>0)?Math.round(100*(d.direct_day+d.dis_day)/dc)+' %':'&ndash;');"
        "T('o_cost',(d.pbuy>0&&d.imp_day!==null)?E(d.imp_day*d.pbuy/100):'&ndash;');"
        "T('o_save',(d.pbuy>0&&d.direct_day!==null&&d.dis_day!==null)?E((d.direct_day+d.dis_day)*d.pbuy/100):'&ndash;');"
        "T('o_imp',V(d.imp_day,K));T('o_exp',V(d.exp_day,K));"
        "T('o_gw',d.grid_w===null?'&ndash;':(d.grid_w>0?'Bezug '+d.grid_w+' W':(d.grid_w<0?'Einsp. '+(-d.grid_w)+' W':'0 W')));"
        "T('o_soc',V(d.soc,P));"
        "T('o_batw',d.bat_w===null?'&ndash;':(d.bat_w>0?'l&auml;dt '+d.bat_w+' W':(d.bat_w<0?'entl&auml;dt '+(-d.bat_w)+' W':'Ruhe')));"
        "T('o_rev',(d.pfeed>0&&d.exp_day!==null)?E(d.exp_day*d.pfeed/100):'&ndash;');"
        "var f=$('o_fill');if(f&&d.soc!==null){var s=Math.max(0,Math.min(100,d.soc));f.setAttribute('width',(24*s/100).toFixed(1));f.style.fill=s<15?'#d9534f':(s<30?'#f0a30a':'#2e9e4f');}"
        "}", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "function fill(d){"
        "show('a_pw',d.defpw);show('a_grid',d.grid_state===170);show('a_fault',d.fault===true);"
        "var dc=(d.direct_day!==null&&d.dis_day!==null&&d.imp_day!==null)?d.direct_day+d.dis_day+d.imp_day:null;"
        "T('pvday',V(d.pv_day,K));T('impday',V(d.imp_day,K));T('expday',V(d.exp_day,K));"
        "T('chgday',V(d.chg_day,K));T('disday',V(d.dis_day,K));T('selfuse',V(d.self_use,P));"
        "show('cost_buy_row',d.pbuy>0&&d.imp_day!==null);if(d.imp_day!==null)T('cost_buy',E(d.imp_day*d.pbuy/100));"
        "show('rev_row',d.pfeed>0&&d.exp_day!==null);if(d.exp_day!==null)T('rev',E(d.exp_day*d.pfeed/100));"
        "var sv=d.pbuy>0&&d.direct_day!==null&&d.dis_day!==null;show('save_row',sv);"
        "if(sv)T('save',E((d.direct_day+d.dis_day)*d.pbuy/100));"
        "T('pvtot',V(d.pv_total,K));T('imptot',V(d.imp_total,K));T('exptot',V(d.exp_total,K));"
        "ovfill(d,dc);", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "var w=d.wb;show('wbbox',w.on);"
        "if(w.on){"
        "show('wbnoconn',!w.ok);show('wbdetail',w.ok);"
        "if(!w.ok)T('wbnoconn','Keine Verbindung zu '+w.ip);"
        "if(w.ok){"
        "T('wbstate',WBS[w.state]||'Unbekannt');T('wbpw',V(w.pw,W));"
        "if(w.pw>50&&d.grid_w!==null){var fg=Math.min(Math.max(d.grid_w,0),w.pw);"
        "T('wbshare','ca. '+fg+' W aus dem Netz, ca. '+(w.pw-fg)+' W aus PV/Akku');show('wbshare',true);}"
        "else show('wbshare',false);"
        "T('wbi',(w.i[0]===null||w.i[1]===null||w.i[2]===null)?'&ndash;':w.i[0].toFixed(1)+' / '+w.i[1].toFixed(1)+' / '+w.i[2].toFixed(1)+' A');"
        "T('wbv',(w.v[0]===null||w.v[1]===null||w.v[2]===null)?'&ndash;':w.v[0].toFixed(0)+' / '+w.v[1].toFixed(0)+' / '+w.v[2].toFixed(0)+' V');"
        "T('wbtot',V(w.tot,K));"
        "T('wbmisc','Phasen: '+(w.ph===null?'&ndash;':w.ph)+' &middot; Betriebsart: '+(w.mode===null?'&ndash;':(WBM[w.mode]||'unbekannt'))"
        "+' &middot; Sollwert (roh): '+(w.set===null?'&ndash;':w.set)+' &middot; '+w.ip);"
        "}}"
        "}"
        "function upd(){fetch('/api/status',{cache:'no-store'}).then(function(r){return r.json();}).then(fill).catch(function(){});}"
        "upd();setInterval(upd,3000);"
        "</script>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// =====================================================
//   Infoseite (/info): Inhalt kommt per /api/info
// =====================================================
static esp_err_t info_get_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<h2>Info</h2>"
"<div class='box'><h3>Batterie</h3>"
        "<p>Akkustand: <b id='soc'>&ndash;</b></p>"
        "<p>Lade-/Entladeleistung: <b id='bat_pw'>&ndash;</b></p>"
        "<p>Notreserve: <b id='backup'>&ndash;</b></p>"
        "<p>Temperatur: <b id='temp'>&ndash;</b> &middot; SOH: <b id='soh'>&ndash;</b></p>"
        "<p>Spannung: <b id='batv'>&ndash;</b></p>"
        "<p id='range_row' style='display:none'>Reichweite bei aktuellem Verbrauch: <b id='range'>&ndash;</b></p>"
        "<p style='font-size:13px;color:#666;'>Tiefster Stand seit Start: <span id='socmin'>&ndash;</span></p>"
        "</div>"

        "<div class='box'><h3>Netz &amp; PV</h3>"
        "<p>Netzstatus: <b id='gstate'>&ndash;</b></p>"
        "<p>PV-Leistung: <b id='pv'>&ndash;</b></p>"
        "<p>Netz: <b id='grid'>&ndash;</b></p>"
        "<p>Systemzustand: <b id='sys'>&ndash;</b></p>"
        "<p>St&ouml;rungen: <b id='flt'>&ndash;</b></p>"
        "</div>"

        "<div class='box'><h3>Hausverbrauch</h3>"
        "<p>Aktuelle Leistung: <b id='load'>&ndash;</b></p>"
        "<p>Verbrauch heute: <b id='daycons'>&ndash;</b></p>"
        "<p>Verbrauch im Jahr: <b id='yearcons'>&ndash;</b></p>"
        "<p>Autarkie heute: <b id='autarky'>&ndash;</b></p>"
        "</div>",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<div id='c'>Lade...</div>"
        "<p style='margin-top:30px;'><a href='/'>&larr; Zur&uuml;ck zur Steuerung</a></p>"
        "<script>"
        "function R(l,v){return '<tr><td>'+l+'</td><td><b>'+v+'</b></td></tr>';}"
        "function S(t,r){return '<div class=box style=text-align:left><h3 style=text-align:center>'+t+'</h3>'"
        "+'<table cellpadding=4 style=font-size:15px>'+r+'</table></div>';}"
        "function U(v,f){return(v===null||v===undefined)?'&ndash;':f(v);}"
        "function F1(u){return function(v){return v.toFixed(1)+' '+u;};}"
        "function J3(a,u,n){return(a[0]===null||a[1]===null||a[2]===null)?'&ndash;':a[0].toFixed(n)+' / '+a[1].toFixed(n)+' / '+a[2].toFixed(n)+' '+u;}"
        "function Z(n){return n<10?'0'+n:''+n;}"
        "var SYS={2:'Stopp',8:'Standby',16:'Initialisierung',32:'Start',64:'L&auml;uft',256:'ST&Ouml;RUNG',"
        "1024:'Wartungsmodus',2048:'Zwangsbetrieb',4096:'L&auml;uft im Notbetrieb',9473:'Neustart',16384:'Externer EMS-Modus'};"
        "var FN=['Wechselrichter-Alarm','Netzseitiger Fehler','Systemfehler 1','Systemfehler 2','DC-seitiger Fehler',"
        "'Dauerfehler','BDC-Fehler','BDC-Dauerfehler','Batteriefehler','Batteriealarm','BMS-Alarm','BMS-Schutz',"
        "'BMS-Fehler 1','BMS-Fehler 2'];", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "function MB(n,m){var la=m.last_age<0?'nie':(m.last_age+' s her');"
        "return R(n+': Antworten OK / Fehler / nicht unterst.',m.ok+' / '+m.err+' / '+m.exc)"
        "+R(n+': Verbindungen',m.conn)+R(n+': letzter Fehler',m.err>0?('0x'+m.last.toString(16)+' ('+la+')'):'keiner');}"
        "function build(d){"
        "var h='';"
        "h+=S('Verbindung',R('Stand der Werte',d.age===null?(d.ready?'Warte auf erste Abfrage...':'Modbus nicht verbunden'):('vor '+d.age+' s'))"
        "+R('Abfrage alle',d.poll+' s')+R('Sungrow-IP',d.inv_ip)+R('Ethernet-IP',d.eth_ip||'nicht verbunden')"
        "+R('WLAN-IP','192.168.4.1')+(d.wb_ip?R('Wallbox-IP',d.wb_ip):''));"
        "var tp=d.dev_type===null?'&ndash;':((d.dev_name?d.dev_name+' ':'')+'(0x'+d.dev_type.toString(16).toUpperCase()+')');"
        "var ck=d.clock?(Z(d.clock[2])+'.'+Z(d.clock[1])+'.'+d.clock[0]+' '+Z(d.clock[3])+':'+Z(d.clock[4])+':'+Z(d.clock[5])):'&ndash;';"
        "h+=S('Wechselrichter',R('Typ',tp)+R('Seriennummer',d.serial||'&ndash;')+R('Firmware',d.fw||'&ndash;')"
        "+R('Uhr',ck)+R('Systemzustand',d.sys_state===null?'&ndash;':(SYS[d.sys_state]||'unbekannt'))"
        "+R('Netzstatus',d.grid_state===85?'am Netz':(d.grid_state===170?'NOTBETRIEB':'&ndash;'))"
        "+R('Innentemperatur',U(d.inv_temp,F1('&deg;C')))+R('CO&#8322;-Einsparung gesamt',U(d.co2,F1('kg'))));"
        "h+=S('Netz',R('Frequenz',U(d.freq,F1('Hz')))+R('Spannung L1/L2/L3',J3(d.gv,'V',1))"
        "+R('Strom L1/L2/L3',J3(d.gi,'A',1))+R('PV-Erzeugung gesamt',U(d.pv_total,F1('kWh')))"
        "+R('Netzbezug gesamt',U(d.imp_total,F1('kWh')))+R('Einspeisung gesamt',U(d.exp_total,F1('kWh'))));"
        "h+=S('Batterie',R('Spannung',U(d.bat_v,F1('V')))+R('Temperatur',U(d.temp,F1('&deg;C')))"
        "+R('Gesundheit (SOH)',U(d.soh,F1('%')))"
        "+R('Min-SOC / Max-SOC',(d.min_soc===null||d.max_soc===null)?'&ndash;':(d.min_soc.toFixed(1)+' % / '+d.max_soc.toFixed(1)+' %'))"
        "+R('Notreserve',U(d.backup,function(v){return v+' %';}))+R('Tiefster Stand seit Start',U(d.soc_min,F1('%'))));", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "var fr='',cnt=0;"
        "if(d.faults===null){fr=R('Fehlerregister','nicht lesbar');}"
        "else{for(var i=0;i<d.faults.length;i++){if(d.faults[i]){cnt++;"
        "fr+=R(FN[i]||('Gruppe '+i),'0x'+('00000000'+d.faults[i].toString(16).toUpperCase()).slice(-8));}}"
        "if(!cnt)fr=R('Fehlerregister','keine Eintr&auml;ge');}"
        "h+=S('St&ouml;rungen',fr);"
        "var u=d.up,ut=Math.floor(u/86400)+' d '+Math.floor((u%86400)/3600)+' h '+Math.floor((u%3600)/60)+' min';"
        "h+=S('Ger&auml;t (ESP32)',R('Laufzeit',ut)+R('Speicher frei',d.heap+' kB (Minimum '+d.heap_min+' kB)')"
        "+R('Build',d.build)+R('ESP-IDF',d.idf)+R('Aktive Partition',d.part)"
        "+R('Ethernet-Link',d.eth_link?'verbunden':'getrennt')+R('WLAN-Clients',d.clients<0?'&ndash;':d.clients));"
        "h+=S('Modbus-Statistik',MB('Wechselrichter',d.mb_inv)+(d.wb_ip?MB('Wallbox',d.mb_wb):''));"
        "document.getElementById('c').innerHTML=h;}"
        "function upd(){fetch('/api/info',{cache:'no-store'}).then(function(r){return r.json();}).then(build).catch(function(){});}"
        "upd();setInterval(upd,5000);"
        "function $(i){return document.getElementById(i);}"
        "function T(i,s){var e=$(i);if(e)e.innerHTML=s;}"
        "function V(v,f){return(v===null||v===undefined)?'&ndash;':f(v);}"
        "function W(v){return Math.round(v)+' W';}"
        "function K(v){return v.toFixed(1)+' kWh';}"
        "function P(v){return v.toFixed(1)+' %';}"
        "function show(i,on){var e=$(i);if(e)e.style.display=on?'':'none';}"
        "function live(d){"
        "T('soc',V(d.soc,P));"
        "T('bat_pw',d.bat_w===null?'keine Daten':(d.bat_w>0?'Laden: '+d.bat_w+' W':(d.bat_w<0?'Entladen: '+(-d.bat_w)+' W':'0 W (Ruhe)')));"
        "T('backup',V(d.backup,function(v){return v+' %';}));"
        "T('temp',V(d.temp,function(v){return v.toFixed(1)+' &deg;C';}));"
        "T('soh',V(d.soh,P));"
        "T('batv',V(d.bat_v,function(v){return v.toFixed(1)+' V';}));"
        "T('socmin',V(d.soc_min,P));"
        "if(d.cap>0&&d.soc!==null&&d.load_w!==null){"
        "var ms=d.min_soc===null?0:d.min_soc;var use=d.cap*Math.max(0,d.soc-ms)/100;"
        "var h=d.load_w>50?use/(d.load_w/1000):999;"
        "T('range',h>99?'&gt; 99 h':'ca. '+h.toFixed(1)+' h');show('range_row',true);"
        "}else show('range_row',false);"
        "T('gstate',d.grid_state===85?'am Netz':(d.grid_state===170?'<span class=bad>NOTBETRIEB</span>':'&ndash;'));"
        "T('pv',V(d.pv_w,W));"
        "T('grid',d.grid_w===null?'&ndash;':(d.grid_w>0?'Bezug: '+d.grid_w+' W':(d.grid_w<0?'Einspeisung: '+(-d.grid_w)+' W':'0 W')));"
        "T('sys',d.sys_state===null?'&ndash;':(SYS[d.sys_state]||'unbekannt'));"
        "T('flt',d.fault===true?'<span class=bad>aktiv</span>':(d.fault_known?'keine':'&ndash;'));"
        "T('load',V(d.load_w,W));"
        "var dc=(d.direct_day!==null&&d.dis_day!==null&&d.imp_day!==null)?d.direct_day+d.dis_day+d.imp_day:null;"
        "T('daycons',V(dc,K));"
        "var ys='&ndash;';"
        "if(d.year_cons!==null){ys=K(d.year_cons);if(d.year_since){var s=''+d.year_since;"
        "ys+=' <span style=font-size:12px;color:#666>(seit '+s.substr(6,2)+'.'+s.substr(4,2)+'.'+s.substr(0,4)+')</span>';}}"
        "T('yearcons',ys);"
        "T('autarky',(dc!==null&&dc>0)?Math.round(100*(d.direct_day+d.dis_day)/dc)+' %':'&ndash;');"
        "}"
        "function updl(){fetch('/api/status',{cache:'no-store'}).then(function(r){return r.json();}).then(live).catch(function(){});}"
        "updl();setInterval(updl,3000);"
        "</script>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, HTML_FOOT, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// =====================================================
//   Verlauf (/chart): Diagramme per Canvas aus /api/history
// =====================================================
static esp_err_t chart_get_handler(httpd_req_t *req) {
    // öffentlich (nur lesend): kein Login nötig

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<h2>Verlauf</h2>"
        "<div class='box' style='max-width:900px;'><h3>Leistung</h3>"
        "<canvas id='c1' width='860' height='260' style='max-width:100%;height:auto;'></canvas>"
        "<p style='font-size:13px;color:#666;'>"
        "<b style='color:#e6a700'>&#9644; PV</b> &middot; "
        "<b style='color:#d9534f'>&#9644; Haus</b> &middot; "
        "<b style='color:#555'>&#9644; Netz</b> (+ Bezug / &ndash; Einspeisung) &middot; "
        "<b style='color:#28a745'>&#9644; Akku</b> (+ Laden / &ndash; Entladen)</p></div>"
        "<div class='box' style='max-width:900px;'><h3>Akkustand</h3>"
        "<canvas id='c2' width='860' height='200' style='max-width:100%;height:auto;'></canvas></div>"
        "<p style='font-size:13px;color:#666;'>Ein Punkt alle 5 Minuten, bis zu 24 Stunden. "
        "Nach einem Neustart beginnt der Verlauf neu.</p>"
        "<p><a href='/'>&larr; Zur&uuml;ck zur Steuerung</a></p>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<script>"
        "function draw(id,keys,d,unit,zero){"
        "var cv=document.getElementById(id),x=cv.getContext('2d'),W=cv.width,H=cv.height,L=52,B=22,T=8,n=d.n;"
        "x.clearRect(0,0,W,H);x.font='12px sans-serif';"
        "var lo=1e9,hi=-1e9;"
        "keys.forEach(function(k){d[k[0]].forEach(function(v){if(v!==null){if(v<lo)lo=v;if(v>hi)hi=v;}});});"
        "x.fillStyle='#555';"
        "if(lo>hi){x.fillText('Noch keine Daten (erster Messpunkt nach kurzer Zeit)',L,H/2);return;}"
        "if(zero){if(lo>0)lo=0;if(hi<0)hi=0;}"
        "if(hi<=lo)hi=lo+1;"
        "var ph=H-T-B,pw=W-L-6;"
        "x.strokeStyle='#ddd';"
        "for(var i=0;i<=4;i++){var y=T+ph-ph*i/4;x.beginPath();x.moveTo(L,y);x.lineTo(W-6,y);x.stroke();"
        "x.fillText(Math.round(lo+(hi-lo)*i/4)+unit,2,y+4);}"
        "if(zero&&lo<=0&&hi>=0){var z=T+ph-ph*(0-lo)/(hi-lo);x.strokeStyle='#222';x.lineWidth=1.5;x.beginPath();x.moveTo(L,z);x.lineTo(W-6,z);x.stroke();"
        "x.lineWidth=1;x.fillStyle='#222';x.font='bold 12px sans-serif';x.fillText('0',L-14,z+4);x.font='12px sans-serif';x.fillStyle='#555';}"
        "keys.forEach(function(k){x.strokeStyle=k[1];x.lineWidth=2;x.beginPath();var pen=false;"
        "d[k[0]].forEach(function(v,i){if(v===null){pen=false;return;}"
        "var px=L+pw*i/Math.max(1,n-1),py=T+ph-ph*(v-lo)/(hi-lo);"
        "if(pen)x.lineTo(px,py);else{x.moveTo(px,py);pen=true;}});x.stroke();});"
        "x.lineWidth=1;x.fillStyle='#555';"
        "x.fillText('vor '+(n*d.step/3600).toFixed(1)+' h',L,H-6);x.fillText('jetzt',W-36,H-6);}"
        "function load(){fetch('/api/history',{cache:'no-store'}).then(function(r){return r.json();}).then(function(d){"
        "draw('c1',[['pv','#e6a700'],['load','#d9534f'],['grid','#555'],['bat','#28a745']],d,' W',true);"
        "draw('c2',[['soc','#0066cc']],d,' %');}).catch(function(){});}"
        "load();setInterval(load,60000);"
        "</script>", HTTPD_RESP_USE_STRLEN);
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
                                 "/options", "Zurück");
    }

    esp_err_t err = set_sungrow_backup_soc((uint16_t)val);
    if (err != ESP_OK) {
        return send_message_page(req, "Fehler",
            "Der Wert konnte nicht an den Wechselrichter geschrieben werden. Verbindung und IP prüfen.",
            "/options", "Zurück");
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/options");
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

    // --- Notreserve (Backup-SOC) ---
    {
        soc_cache_t nc = cache_get();
        int cur = (nc.backup_soc >= 0 && nc.backup_soc <= 100) ? nc.backup_soc : SOC_NORMAL;
        send_fmt(req,
            "<div class='box'><h3>Notreserve einstellen</h3>"
            "<p style='font-size:14px;'>Aktuell im Wechselrichter: <b>%d %%</b></p>"
            "<form method='post' action='/set'>"
            "<input type='number' id='val' name='val' min='0' max='100' step='1' value='%d' required "
            "style='padding:10px;font-size:18px;width:90px;text-align:center;'> %%<br>"
            "<button class='grn' type='submit'>&Uuml;bernehmen</button>"
            "</form>"
            "<form method='post' action='/set'>"
            "<button class='gry' name='val' value='%d'>Normalbetrieb (%d %%)</button>"
            "<button class='red' name='val' value='%d'>Freigeben (%d %%)</button>"
            "</form></div><br>", cur, cur, SOC_NORMAL, SOC_NORMAL, SOC_RELEASED, SOC_RELEASED);
    }

    // --- Batterie-Grenzen (Min-/Max-SOC) ---
    {
        soc_cache_t oc = cache_get();
        char lim[400];
        int cur_min = 10, cur_max = 100;
        if (known(oc.ext.min_soc_d) && known(oc.ext.max_soc_d)) {
            snprintf(lim, sizeof(lim), "Aktuell im Wechselrichter: Min %.1f %%, Max %.1f %%",
                     oc.ext.min_soc_d / 10.0, oc.ext.max_soc_d / 10.0);
            cur_min = oc.ext.min_soc_d / 10;
            cur_max = oc.ext.max_soc_d / 10;
        } else {
            snprintf(lim, sizeof(lim), "Aktuelle Werte noch nicht gelesen (Vorbelegung)");
        }
        send_fmt(req,
            "<div class='box'><h3>Batterie-Grenzen</h3>"
            "<p style='font-size:14px;'>%s</p>"
            "<form method='post' action='/config/soclimits'>"
            "<p style='font-size:15px;'>Min-SOC (0&ndash;50 %%): "
            "<input type='number' name='min' min='0' max='50' step='1' value='%d' required "
            "style='padding:8px;font-size:16px;width:70px;text-align:center;'></p>"
            "<p style='font-size:15px;'>Max-SOC (70&ndash;100 %%): "
            "<input type='number' name='max' min='70' max='100' step='1' value='%d' required "
            "style='padding:8px;font-size:16px;width:70px;text-align:center;'></p>"
            "<button class='grn' type='submit'>Speichern</button>"
            "</form></div><br>", lim, cur_min, cur_max);
    }

    // --- Berechnungen (Reichweite, Kosten) ---
    send_fmt(req,
        "<div class='box'><h3>Berechnungen</h3>"
        "<p style='font-size:14px;'>F&uuml;r die Reichweite im Notbetrieb sowie Kosten und Ersparnis. 0 = aus.</p>"
        "<form method='post' action='/config/calc'>"
        "<p style='font-size:15px;'>Batteriekapazit&auml;t (kWh):<br>"
        "<input type='text' name='cap' value='%.1f' inputmode='decimal' style='width:100px;'></p>"
        "<p style='font-size:15px;'>Strompreis (ct/kWh):<br>"
        "<input type='text' name='pbuy' value='%.2f' inputmode='decimal' style='width:100px;'></p>"
        "<p style='font-size:15px;'>Einspeiseverg&uuml;tung (ct/kWh):<br>"
        "<input type='text' name='pfeed' value='%.2f' inputmode='decimal' style='width:100px;'></p>"
        "<button class='grn' type='submit'>Speichern</button>"
        "</form></div><br>",
        g_bat_kwh10 / 10.0, g_price_buy / 100.0, g_price_feed / 100.0);

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

// Min-/Max-SOC speichern (POST /config/soclimits)
static esp_err_t config_soclimits_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[64], smin[12], smax[12];
    if (!read_body(req, body, sizeof(body)) ||
        !form_value(body, "min", smin, sizeof(smin)) ||
        !form_value(body, "max", smax, sizeof(smax))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }
    char *e1 = NULL, *e2 = NULL;
    long vmin = strtol(smin, &e1, 10);
    long vmax = strtol(smax, &e2, 10);
    if (smin[0] == '\0' || smax[0] == '\0' || *e1 != '\0' || *e2 != '\0' ||
        vmin < 0 || vmin > 50 || vmax < 70 || vmax > 100) {
        return send_message_page(req, "Fehler",
            "Ungültige Werte. Min-SOC: 0 bis 50 %, Max-SOC: 70 bis 100 %.",
            "/options", "Zurück zu den Optionen");
    }
    if (!s_modbus_ready) {
        return send_message_page(req, "Fehler", "Keine Verbindung zum Wechselrichter.",
                                 "/options", "Zurück zu den Optionen");
    }
    if (xSemaphoreTake(s_mb_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        return send_message_page(req, "Fehler", "Der Wechselrichter ist gerade beschäftigt. Bitte erneut versuchen.",
                                 "/options", "Zurück zu den Optionen");
    }

    uint16_t echo = 0;
    uint16_t r[2] = { 0, 0 };
    esp_err_t err = mb_transaction(&s_inv, 0x06, REG_MAX_SOC, (uint16_t)(vmax * 10), &echo);
    if (err == ESP_OK) err = mb_transaction(&s_inv, 0x06, REG_MIN_SOC, (uint16_t)(vmin * 10), &echo);
    bool readback = (mb_transaction(&s_inv, 0x03, REG_MAX_SOC, 2, r) == ESP_OK);
    xSemaphoreGive(s_mb_mutex);

    if (readback) {
        xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
        s_cache.ext.max_soc_d = r[0];
        s_cache.ext.min_soc_d = r[1];
        xSemaphoreGive(s_cache_mutex);
    }
    ESP_LOGI(TAG, "Min-/Max-SOC gesetzt: %ld / %ld %% (Ergebnis 0x%x)", vmin, vmax, err);

    if (err != ESP_OK) {
        return send_message_page(req, "Fehler",
            "Die Werte konnten nicht in den Wechselrichter geschrieben werden.",
            "/options", "Zurück zu den Optionen");
    }
    return send_message_page(req, "Gespeichert", "Min-SOC und Max-SOC wurden in den Wechselrichter geschrieben.",
                             "/options", "Zurück zu den Optionen");
}

// Einstellungen für Berechnungen speichern (POST /config/calc)
static esp_err_t config_calc_post_handler(httpd_req_t *req) {
    if (!guard(req, true)) return ESP_OK;

    char body[128], scap[24], sbuy[24], sfeed[24];
    if (!read_body(req, body, sizeof(body)) ||
        !form_value(body, "cap", scap, sizeof(scap)) ||
        !form_value(body, "pbuy", sbuy, sizeof(sbuy)) ||
        !form_value(body, "pfeed", sfeed, sizeof(sfeed))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ungültige Anfrage");
        return ESP_FAIL;
    }
    double cap = 0, pb = 0, pf = 0;
    if (!parse_decimal(scap, 0, 200, &cap) || !parse_decimal(sbuy, 0, 200, &pb) ||
        !parse_decimal(sfeed, 0, 200, &pf)) {
        return send_message_page(req, "Fehler",
            "Ungültige Eingabe. Erlaubt sind Zahlen von 0 bis 200 (Dezimalpunkt oder -komma).",
            "/options", "Zurück zu den Optionen");
    }
    uint32_t c10 = (uint32_t)(cap * 10.0 + 0.5);
    uint32_t b100 = (uint32_t)(pb * 100.0 + 0.5);
    uint32_t f100 = (uint32_t)(pf * 100.0 + 0.5);
    if (config_save_u32("cap10", c10) != ESP_OK ||
        config_save_u32("pbuy", b100) != ESP_OK ||
        config_save_u32("pfeed", f100) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Speichern fehlgeschlagen");
        return ESP_FAIL;
    }
    g_bat_kwh10 = c10;
    g_price_buy = b100;
    g_price_feed = f100;
    return send_message_page(req, "Gespeichert", "Die Einstellungen sind sofort aktiv.",
                             "/options", "Zurück zu den Optionen");
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

// Apple-Touch-Icon (180x180 PNG, 5328 Bytes)
static const uint8_t APPLE_ICON[] = {
    0x89,0x50,0x4e,0x47,0x0d,0x0a,0x1a,0x0a,0x00,0x00,0x00,0x0d,0x49,0x48,0x44,0x52,
    0x00,0x00,0x00,0xb4,0x00,0x00,0x00,0xb4,0x08,0x03,0x00,0x00,0x00,0x0a,0x13,0xf6,
    0x00,0x00,0x00,0x00,0xc0,0x50,0x4c,0x54,0x45,0xff,0xff,0xff,0xff,0xff,0xfd,0xfe,
    0xff,0xff,0xff,0xfe,0xfe,0xfe,0xfe,0xff,0xfe,0xfe,0xfe,0xfd,0xfe,0xff,0xfe,0xfe,
    0xfc,0xfc,0xfe,0xff,0xfc,0xfe,0xfe,0xfc,0xfd,0xfe,0xfb,0xfd,0xfe,0xfd,0xfc,0xfb,
    0xfb,0xfb,0xfb,0xfe,0xf8,0xf4,0xf8,0xfb,0xfd,0xf6,0xf7,0xf8,0xf5,0xf5,0xf5,0xf4,
    0xef,0xec,0xef,0xf5,0xf8,0xef,0xee,0xef,0xef,0xee,0xee,0xee,0xed,0xed,0xed,0xed,
    0xed,0xe2,0xf0,0xf7,0xe8,0xe5,0xe4,0xe0,0xdf,0xdf,0xde,0xd6,0xd3,0xce,0xde,0xe6,
    0xd0,0xce,0xce,0xc8,0xc5,0xc3,0xc5,0xc0,0xbe,0xb4,0xdc,0xf2,0xad,0xc0,0xcb,0x87,
    0xca,0xec,0xa7,0xb0,0xb4,0x7f,0xa6,0xba,0x55,0xae,0xdc,0x2b,0xa2,0xe1,0x6f,0x94,
    0xa7,0x5c,0x82,0x96,0x25,0x8d,0xc3,0x23,0x71,0x99,0x06,0x98,0xe2,0x06,0x94,0xdd,
    0x06,0x93,0xdc,0x05,0x92,0xdb,0x03,0x91,0xdb,0x09,0x85,0xc7,0x09,0x71,0xa7,0x00,
    0x99,0xe7,0x00,0x94,0xdf,0x00,0x92,0xde,0x01,0x92,0xdb,0x01,0x92,0xda,0x01,0x90,
    0xdc,0x00,0x8f,0xda,0x00,0x8c,0xda,0x00,0x8b,0xd9,0x00,0x8a,0xd9,0x00,0x89,0xd7,
    0x00,0x87,0xd5,0x00,0x85,0xd2,0x00,0x77,0xb9,0x08,0xbc,0x6a,0xb1,0x00,0x00,0x13,
    0xcb,0x49,0x44,0x41,0x54,0x78,0xda,0xed,0x5c,0x0b,0x7b,0xa2,0xca,0xb2,0x45,0x46,
    0x91,0xa8,0x01,0xc5,0xec,0x28,0x12,0x7c,0xa2,0xa0,0x22,0xf8,0x1c,0x41,0x14,0xff,
    0xff,0xbf,0xba,0xab,0xba,0x01,0x35,0x31,0xb3,0xef,0x9d,0x3d,0xec,0x13,0xcf,0x4d,
    0x7d,0x33,0x89,0x81,0x7e,0x2c,0x8a,0xaa,0x55,0x55,0x4d,0xa3,0x20,0x7c,0xcb,0xb7,
    0x7c,0xcb,0xb7,0x7c,0xcb,0xb7,0x7c,0xcb,0xb7,0x7c,0xcb,0xb7,0xfc,0x97,0x4b,0xa9,
    0xf4,0x70,0x88,0x85,0xe7,0x5e,0xef,0x59,0x28,0x3d,0x1a,0xe6,0x4e,0xe7,0xb1,0x50,
    0x33,0xcc,0x8d,0xc6,0x43,0xa1,0xe6,0x98,0x5f,0x5f,0x1f,0x09,0x35,0xc7,0xdc,0xfa,
    0xeb,0xaf,0xd6,0xe3,0xa0,0xce,0x30,0x3f,0x10,0xea,0x2b,0xcc,0x0f,0x83,0xfa,0x82,
    0xf9,0xf5,0xf5,0x51,0x50,0x5f,0x61,0x6e,0x34,0x1e,0x04,0xf5,0x05,0x73,0xab,0x6e,
    0x9a,0xf5,0xd6,0x23,0xa0,0xbe,0xd2,0x33,0x30,0x03,0xf5,0x03,0xe8,0xfa,0x16,0xb3,
    0x20,0x3c,0x02,0xea,0x77,0x98,0xcb,0xe5,0x07,0x40,0xfd,0x1e,0xb3,0x20,0x7c,0x7d,
    0xd4,0x1f,0x31,0x7f,0x7d,0xd4,0xf7,0x30,0x7f,0x75,0xd4,0xf7,0x31,0x7f,0x6d,0xd4,
    0xef,0x30,0x97,0xae,0xcf,0x7c,0x55,0xd4,0x9f,0x63,0xfe,0xba,0xba,0xfe,0x15,0xe6,
    0xaf,0xaa,0xeb,0x5f,0x63,0xfe,0x9a,0xa8,0x3f,0xf3,0xc1,0xaf,0x6c,0x21,0x7f,0x8f,
    0xf9,0x3f,0x8d,0xba,0x56,0x85,0x3c,0x97,0x2a,0xcf,0x55,0x2e,0x95,0xca,0x5d,0xcc,
    0xa5,0x4a,0xf5,0x49,0x96,0xe5,0x6a,0xad,0xf4,0x0e,0x75,0x8b,0x50,0xd7,0x4b,0xe5,
    0xb4,0x7f,0xad,0xf2,0x9f,0x51,0x7b,0xfd,0x23,0xe6,0x0a,0xfd,0x28,0x14,0xe8,0xe7,
    0x53,0xe9,0x1a,0xf5,0x6b,0xa3,0xa6,0x08,0xaf,0xe6,0x4d,0x7f,0xb1,0xf2,0x23,0x5f,
    0x80,0xcf,0x23,0x26,0xaf,0x5d,0x6b,0x94,0x88,0xa9,0x99,0xfa,0x3b,0xcc,0x25,0x51,
    0x50,0x7a,0x96,0xed,0x38,0x8e,0x3d,0x32,0x92,0x2b,0xe0,0xa8,0x1b,0x8d,0x9e,0x6d,
    0x59,0x23,0x7b,0xa4,0x67,0xfd,0xbb,0x7a,0x39,0x6b,0x92,0x93,0xf5,0x6a,0xb3,0x78,
    0xb9,0x8c,0x57,0xa6,0x73,0x5e,0x32,0x39,0x9f,0x57,0x33,0xc7,0x54,0x5e,0x5e,0xaf,
    0x78,0xa3,0x24,0xd4,0x46,0xb3,0x73,0x7c,0x0c,0xc3,0x30,0x3e,0xaf,0x9c,0x4e,0x0a,
    0x89,0x38,0xa4,0xe0,0x9c,0x31,0xc0,0xd9,0x19,0xad,0xe2,0x6c,0x00,0x67,0xa4,0xe4,
    0x89,0x9a,0x40,0xaf,0xe7,0xf3,0xf5,0xd9,0xb4,0xfd,0x39,0x97,0xf5,0x7a,0x7d,0x5c,
    0xd9,0xda,0x4b,0xeb,0x82,0x59,0x52,0x9c,0xf3,0x7a,0x3d,0xdf,0xb8,0xae,0x8b,0xf3,
    0xa7,0x59,0xef,0x0a,0xb5,0x61,0x9f,0xd0,0xe9,0x60,0xf7,0xce,0xeb,0xac,0xff,0xfe,
    0xec,0x18,0x39,0xa2,0x66,0xa0,0x7f,0xfe,0x24,0xd0,0x27,0x77,0x97,0xca,0x66,0x7d,
    0x76,0xb4,0x0b,0x3f,0x57,0x6a,0xce,0x72,0xed,0xef,0x76,0xb8,0x2c,0xb4,0xf9,0x39,
    0x5f,0xcf,0xcc,0x0b,0x6a,0xc3,0x39,0x6d,0x76,0xee,0x89,0x40,0xff,0x4c,0xfb,0xbb,
    0xeb,0xed,0xa5,0x49,0x4e,0xa0,0x77,0xbb,0x2b,0x4d,0x6f,0x76,0xbb,0xed,0x16,0x57,
    0x61,0x1b,0x19,0x66,0x61,0x04,0x40,0xdb,0xdd,0x7a,0x13,0xc7,0xe1,0x7a,0xb7,0xdd,
    0xb9,0x5b,0x47,0x91,0x92,0x01,0xaa,0x82,0x1d,0xcf,0xb7,0x9b,0xd3,0x45,0xd3,0x2e,
    0xfa,0x03,0xf6,0x4c,0xcb,0x6d,0x89,0xf5,0x02,0x9a,0xdb,0x74,0xbc,0x8c,0x37,0x6b,
    0x86,0x7a,0xd5,0x13,0x6a,0xac,0xc9,0x0f,0x51,0x99,0x6d,0x7c,0x60,0x3e,0xcf,0x6c,
    0xcb,0x59,0x6d,0x76,0xf4,0x31,0x3d,0x29,0x54,0x4b,0x29,0x68,0xb2,0x69,0xfa,0xb7,
    0x5f,0x53,0x93,0xf9,0xd2,0xc6,0x05,0xe5,0x0d,0xba,0xcb,0x3c,0xdf,0xb1,0xec,0xd9,
    0x99,0x50,0xcf,0x63,0x2b,0x99,0xb4,0x22,0x74,0x57,0xac,0x89,0xad,0x80,0xf4,0x7a,
    0xf8,0xbc,0x75,0x8f,0x4e,0x7a,0xf3,0x13,0x4d,0x1f,0xed,0x8e,0x4d,0xdd,0x47,0x23,
    0x5c,0xd7,0x96,0x5d,0xd8,0x4c,0x13,0x4b,0x39,0x83,0xee,0xb2,0x3f,0xcd,0xd7,0xb2,
    0xa2,0xd9,0x84,0xda,0x87,0x09,0x88,0x9c,0x13,0x85,0xd1,0x72,0x4e,0x26,0x51,0x13,
    0xab,0x55,0x95,0x61,0xdc,0x6d,0x66,0xca,0xad,0x79,0x1c,0x1d,0xe5,0xb9,0x56,0x2b,
    0x13,0x5f,0x97,0xbb,0xb3,0xed,0x76,0xcb,0x86,0xac,0xe4,0x0d,0xba,0x52,0xaf,0xb0,
    0x98,0xd2,0x7a,0xa9,0xc1,0xb5,0x98,0xa6,0xb8,0x4d,0x03,0x16,0x7c,0x74,0x7e,0x1e,
    0x89,0xcf,0x08,0x9f,0x95,0xee,0x79,0xed,0x79,0xd9,0xc9,0x2b,0xd0,0xad,0x16,0x8f,
    0x8d,0x55,0x7e,0x67,0xd0,0x01,0xd7,0x9b,0xb7,0xa6,0x4b,0xe9,0xba,0x68,0x1d,0x3e,
    0x85,0x43,0xab,0x84,0xb5,0x64,0xc1,0x39,0xb8,0x64,0xc6,0x85,0x0a,0xd9,0x8a,0xb9,
    0x5a,0x87,0xe1,0x7a,0xd5,0xf9,0x08,0x3a,0xcd,0x43,0x24,0xe6,0x03,0x17,0xfb,0xca,
    0x13,0x74,0xb6,0x8e,0xf4,0xdc,0x5b,0xbd,0x03,0x1d,0x32,0xd0,0x02,0x03,0xdd,0x71,
    0x98,0xbc,0xde,0x82,0x0e,0x1c,0xa3,0x71,0x59,0x7b,0x42,0x87,0x7f,0x05,0xb4,0x59,
    0xca,0xd6,0xeb,0x8c,0x11,0xd7,0xb4,0xce,0x71,0x3d,0x5d,0x9b,0x07,0xcf,0x40,0x78,
    0x0e,0x72,0x05,0x1a,0x3c,0x6d,0x98,0xd9,0x3a,0x9f,0xe2,0xec,0x3d,0xd0,0xc7,0xbf,
    0x60,0x1e,0x26,0x30,0xbf,0xbe,0xbe,0xb6,0x1a,0x66,0x67,0x74,0x9e,0xef,0x7c,0xf8,
    0x9a,0x98,0xc0,0xb2,0xe2,0x39,0x85,0x0f,0xc0,0x27,0x06,0x94,0x49,0x9e,0xde,0x81,
    0x3e,0xda,0x94,0x87,0xb0,0x01,0x5e,0xf4,0x1e,0x1f,0xb2,0x97,0xbb,0x23,0x8e,0x5a,
    0xf5,0x97,0x06,0x44,0x35,0x3a,0x3a,0x1d,0x99,0xc7,0x29,0xcf,0xd6,0x04,0x6e,0xe3,
    0x33,0xfd,0x5e,0xb4,0xb8,0x80,0x46,0x44,0x57,0x69,0x84,0xba,0xe2,0xe0,0xc8,0xcf,
    0x7f,0x83,0xf2,0x7a,0x9a,0x41,0xd2,0xe9,0x8d,0x46,0xb3,0x60,0xb3,0x03,0x3f,0xe8,
    0xc9,0x9c,0x25,0x81,0x5f,0x05,0xd8,0xa0,0xf2,0x0b,0xd0,0x35,0x5d,0xeb,0xf5,0x3a,
    0x18,0xa2,0x87,0x3c,0x65,0x87,0x90,0x3a,0xca,0x2d,0x8e,0x5f,0xcc,0xc3,0x9e,0x71,
    0x59,0x9d,0xd7,0x1b,0xa4,0x17,0xab,0xcb,0xcd,0xad,0x14,0x9c,0xa3,0x4b,0x1c,0x98,
    0x5e,0x86,0x50,0xaa,0x55,0x2a,0x95,0x5a,0xe5,0xd6,0xa6,0xb3,0xfe,0x87,0x75,0xe0,
    0xae,0x97,0x8e,0x92,0x7f,0xee,0xd1,0xb5,0xb7,0xeb,0x44,0x76,0xeb,0xf5,0x72,0x76,
    0x65,0x90,0xcf,0x9c,0x78,0x37,0xb1,0x53,0xa8,0x7e,0xa6,0xe9,0x93,0x6d,0x9e,0xd3,
    0xfe,0xee,0x66,0x1d,0xae,0x1c,0xed,0x7e,0x95,0xf6,0x87,0x41,0x67,0x59,0xde,0x66,
    0x35,0x1b,0x69,0x17,0xcc,0x65,0x41,0xd3,0xc0,0xd4,0x14,0xe3,0x38,0x1f,0xc0,0x5e,
    0x46,0xbd,0x5e,0x0f,0xd5,0x40,0xe9,0x16,0x34,0xcf,0xf2,0x7e,0x6e,0x96,0x48,0xc8,
    0xf5,0x1c,0xeb,0xc6,0x1b,0xd0,0x3e,0x84,0x40,0x23,0x33,0x32,0x05,0x31,0x6b,0xa1,
    0xcf,0x1c,0x63,0xc5,0xd2,0xbc,0x55,0x97,0xf2,0xa4,0x9a,0x30,0x5a,0x9d,0x29,0xd5,
    0x27,0x4f,0xbd,0x06,0xed,0xb3,0x01,0x7e,0x6e,0x62,0x54,0x01,0x9a,0xde,0xab,0xe5,
    0x85,0xba,0x92,0x81,0x76,0xc2,0x0d,0x93,0xf5,0x1a,0xff,0x4e,0x2b,0xbb,0xf6,0x54,
    0xe0,0x05,0x80,0x36,0xdb,0x9e,0x47,0x44,0xdd,0x64,0xd6,0x06,0x20,0x03,0xf4,0x12,
    0x56,0x70,0xb0,0xdf,0x83,0xde,0xf2,0xfe,0xf3,0xcd,0x1a,0xd7,0x6d,0xe4,0xa6,0x6b,
    0xd4,0x51,0x09,0xa9,0x76,0xad,0x59,0xea,0x49,0x27,0x72,0xa4,0xb3,0x5d,0xa8,0x24,
    0x5e,0x18,0xc3,0x2d,0x0d,0xfc,0x44,0x72,0xbf,0x9d,0xe9,0x42,0x05,0xc4,0x8d,0x0c,
    0x6a,0xbe,0xb4,0x6e,0x40,0x1b,0xb3,0x8b,0x27,0xff,0xdc,0xad,0xf7,0xb3,0x4e,0x4e,
    0xa8,0xb1,0x56,0xd0,0x35,0x13,0xd0,0x46,0x57,0x61,0xa2,0x75,0x39,0x65,0xad,0x92,
    0xb0,0x8d,0x5c,0xe3,0xe7,0x66,0x33,0x33,0x9c,0x03,0x50,0xcf,0x43,0x64,0x4a,0x2c,
    0xd3,0x4b,0x13,0xa2,0x0b,0x4f,0x2b,0x49,0x7f,0x63,0x34,0x43,0xbe,0x35,0x8f,0x1c,
    0x25,0x97,0xf5,0x10,0x5a,0xdf,0xd0,0xf5,0x24,0xb8,0x18,0x82,0x48,0x42,0x9e,0x47,
    0x98,0x28,0x22,0x16,0x39,0x26,0x96,0xf2,0xa1,0x1d,0xd2,0xe4,0xdd,0xfc,0xe8,0x68,
    0x72,0xcd,0xd9,0x67,0xc9,0xc8,0x05,0xb4,0x2c,0xa6,0x03,0x50,0xee,0xc2,0xae,0x3a,
    0x07,0xd4,0x6c,0x4d,0xa6,0x6e,0x24,0xa0,0x85,0x64,0x15,0xa6,0xf4,0x2c,0xf2,0xeb,
    0x58,0x51,0x95,0xc7,0xaa,0x16,0x76,0x5a,0x30,0x66,0x1b,0x97,0x22,0xa5,0xa3,0x24,
    0xe7,0x0d,0x22,0xb5,0x0b,0xe8,0xb4,0x6a,0x2c,0xe3,0x4e,0x20,0xb7,0x45,0x31,0xa3,
    0xfc,0xf9,0xb5,0x27,0xbe,0x8e,0xf4,0xd2,0x49,0x13,0xa6,0x8c,0xe3,0xa4,0x32,0x69,
    0x92,0xdd,0xfe,0x24,0x1a,0x52,0xc1,0xf7,0xcc,0x50,0x43,0xe9,0x4b,0xa7,0xc3,0x68,
    0x9b,0xd7,0x01,0x1f,0x40,0x53,0xce,0xcd,0x32,0xae,0xcd,0x4c,0xfb,0xe3,0x2b,0x66,
    0xc9,0xda,0x57,0xa3,0x93,0xa6,0xa6,0xd9,0xa4,0xb2,0x48,0xb4,0xcc,0x92,0x8f,0x8a,
    0x00,0xb2,0x4b,0x92,0xa7,0x9a,0x00,0xf3,0x67,0x74,0x8d,0x7a,0x70,0x4d,0x09,0x54,
    0xf5,0x3e,0x68,0xd4,0x67,0x3c,0x59,0x61,0xc3,0xff,0x49,0xd4,0x09,0xe6,0xd7,0x7b,
    0xa0,0x0b,0x3c,0x1f,0xbe,0x80,0xe6,0xa5,0x57,0x4d,0xe8,0xa2,0x2d,0x55,0xea,0x23,
    0xa0,0x4e,0x6a,0xdb,0x7b,0xa0,0xa5,0x24,0xc3,0x32,0x1a,0xaf,0x7f,0x14,0x75,0x21,
    0xc9,0xf9,0xef,0x80,0xae,0x89,0x44,0xdd,0x3c,0x89,0x47,0xce,0xbf,0xe2,0xe6,0x41,
    0xab,0x8a,0x35,0xe6,0x62,0x54,0xa9,0x83,0xb7,0x93,0x24,0xee,0x23,0x68,0x18,0x12,
    0xab,0x23,0xe1,0xbc,0x58,0xa6,0xe2,0xa8,0x0b,0x7f,0x46,0xd1,0x6c,0x9f,0xcc,0x5f,
    0x57,0xa0,0xa5,0x2a,0x4b,0x94,0x65,0x5c,0x90,0xbd,0x9c,0xef,0x52,0x9b,0x4e,0x33,
    0x57,0x41,0xad,0x94,0x00,0x27,0x45,0xdd,0xb3,0x13,0x94,0x19,0xe8,0x92,0xca,0xfb,
    0xe3,0x96,0x10,0x8b,0x52,0xf1,0xab,0xbd,0xb0,0xa2,0x02,0xa8,0x4b,0x39,0x81,0x4e,
    0xb4,0x51,0x50,0x4c,0x87,0x47,0x3f,0xc6,0x1e,0xcc,0x29,0x49,0x6b,0xdd,0xa4,0x63,
    0x8f,0x02,0x3a,0xe3,0xb3,0xe7,0xf7,0xf9,0x74,0x81,0xf7,0xd7,0x47,0x20,0xc7,0xed,
    0x76,0x13,0xdb,0x4a,0xeb,0xcf,0x82,0xfe,0x60,0x1e,0xe6,0x88,0xd7,0x7e,0xce,0x6c,
    0x15,0xd3,0xba,0x47,0x52,0xb9,0x60,0xfd,0x00,0x65,0x0c,0xe2,0xf3,0xca,0xee,0x19,
    0xba,0x6e,0x5a,0x23,0xa6,0xeb,0x5d,0x86,0x3a,0x8b,0x88,0x49,0xed,0x88,0x01,0x90,
    0xdb,0xf2,0xd5,0x9e,0xe7,0x6c,0xfb,0x50,0x21,0x1f,0x47,0x34,0xed,0x38,0xc0,0xa2,
    0x68,0xb8,0xdf,0xd0,0x02,0xd1,0x36,0x4d,0xe2,0x19,0xe7,0x21,0x57,0xda,0xac,0xb1,
    0x5c,0x8a,0x10,0x1d,0xc3,0x9c,0x79,0xf2,0x84,0xc5,0xba,0xda,0x4d,0x6a,0xba,0xda,
    0x1c,0x68,0x00,0x64,0xb8,0x6c,0x5d,0x6c,0xef,0x90,0x75,0xe4,0x43,0x1f,0x17,0xd0,
    0x47,0x97,0x09,0x2d,0xe5,0x21,0x9e,0xc5,0x0e,0x4f,0xf2,0x93,0x85,0x3c,0x82,0x4d,
    0x32,0x5f,0x27,0xa8,0x5d,0x9e,0x3c,0xdd,0x24,0x4c,0x1e,0xf5,0xf7,0xd9,0x5a,0x20,
    0x4e,0xd3,0xd6,0x90,0x9c,0x88,0x9a,0x82,0x8b,0x0f,0xbd,0x1a,0x57,0xab,0xa6,0xee,
    0x9c,0x56,0x4d,0xa5,0x64,0xf9,0xb1,0x8c,0x15,0xa7,0xf4,0xd4,0xcf,0x9d,0x8f,0xdb,
    0x4e,0x97,0xb1,0xf3,0xb6,0x33,0x5a,0x63,0x20,0xd0,0xee,0xce,0x8d,0x2f,0xf9,0x34,
    0x64,0xbe,0x0e,0x67,0x64,0x1c,0xad,0x9c,0x42,0x22,0x85,0x71,0xc8,0xb9,0x77,0xa9,
    0x5c,0xb0,0xbc,0x3c,0xb3,0x94,0x74,0xae,0x92,0x54,0xb3,0x91,0xf8,0x65,0x8b,0xcf,
    0x87,0xb3,0xa3,0xd8,0xbc,0x4a,0x99,0x61,0xf1,0x03,0xa0,0x43,0xea,0x72,0x55,0xb9,
    0xac,0xd7,0x48,0xa8,0xcd,0x7c,0x30,0x27,0xa8,0xf5,0xd9,0x79,0xb5,0x3a,0xaf,0x46,
    0x0e,0x25,0xf6,0x94,0xdb,0xe3,0x41,0xc0,0x48,0x17,0x2e,0x95,0xf4,0x0f,0x51,0x30,
    0xe1,0x5d,0xe7,0x18,0x72,0x3e,0xcf,0x9c,0x9e,0x22,0x97,0x6d,0x34,0x46,0xa7,0x99,
    0x2e,0x81,0x94,0x59,0x45,0x00,0x9b,0x3e,0xaf,0x92,0xfe,0x33,0xbb,0xa7,0xd5,0x5b,
    0x79,0x3d,0xf6,0xa2,0xd4,0xb4,0x63,0xf6,0x48,0xfe,0xea,0x8d,0xcc,0x2e,0x6d,0xf7,
    0x31,0x0d,0x8d,0x1e,0x99,0xfc,0xb8,0xa9,0x14,0x04,0xbd,0x3b,0xb2,0x6c,0xdb,0x1a,
    0x75,0x35,0xf6,0x04,0xa6,0xc0,0x5a,0x9a,0xdd,0x16,0x72,0x26,0x9d,0xba,0x75,0x3b,
    0x8d,0x6e,0x72,0xcc,0x18,0x99,0xb5,0x7a,0xab,0x95,0xdf,0xa3,0x3a,0x96,0x9c,0x56,
    0x1a,0xf5,0x7a,0xbd,0x55,0x37,0x8c,0xf4,0xd9,0x54,0x55,0x7d,0xae,0xdc,0xc8,0xb3,
    0x7a,0x79,0x70,0xa5,0x3e,0x57,0x2b,0xd5,0xac,0xc0,0xc5,0xc9,0xf4,0x44,0x7a,0xc8,
    0x30,0x9b,0x7a,0xb2,0x18,0x99,0x5b,0xe9,0x52,0x30,0x75,0x9e,0xbd,0x97,0x33,0xd4,
    0xff,0x48,0x0c,0xa3,0xcc,0x86,0xab,0xe9,0xe6,0x8f,0x9c,0x8a,0x44,0xb1,0xde,0x1a,
    0xd8,0x83,0x44,0x6c,0x6b,0xf0,0x8f,0xc5,0xba,0x8c,0x36,0x68,0xd6,0x73,0xc1,0x2c,
    0x15,0xac,0x29,0x3d,0x44,0x5b,0x32,0x1f,0xcc,0x3e,0xfc,0xb6,0xdc,0x0c,0xb5,0x9c,
    0x0e,0xb0,0xe4,0x9a,0x87,0x34,0xdb,0xcd,0xe6,0x4b,0xcf,0x6c,0xe9,0x24,0x2d,0xb3,
    0x5b,0x6f,0x36,0x9b,0xb4,0x22,0xd7,0x48,0x7f,0xa7,0x9f,0x9a,0xef,0x25,0x39,0x75,
    0x69,0xdd,0x6c,0xd6,0xbb,0xd9,0x40,0xbd,0x97,0x66,0xb3,0xdd,0xc8,0xf3,0xa9,0xad,
    0x64,0xea,0xcf,0x2f,0x2f,0x9a,0xf6,0xc2,0xb6,0xde,0xfd,0xbe,0x98,0xc6,0x73,0x43,
    0xd3,0xb4,0xbf,0xea,0xba,0x99,0x27,0x5c,0xa1,0x58,0x2a,0x49,0x02,0x32,0x33,0x53,
    0xc3,0x43,0xda,0xd7,0x56,0xd9,0xec,0x75,0xc0,0x6a,0x52,0x49,0xd4,0x5e,0xeb,0x25,
    0x26,0x92,0xf6,0xfa,0x8c,0x5f,0x35,0x4d,0x7b,0xd6,0x52,0xc1,0x27,0x1c,0x93,0x05,
    0xa5,0x83,0xd6,0x22,0xce,0x6a,0xaf,0xca,0x13,0x16,0x7b,0xff,0x32,0x90,0x43,0xb7,
    0x30,0x9c,0xa1,0x2b,0x42,0x9e,0x3b,0x13,0x4a,0x85,0xc1,0x14,0xcf,0xd2,0xa6,0x23,
    0xfd,0xa5,0xa5,0xe0,0xe3,0x72,0x6a,0x69,0xa2,0xa8,0xd8,0xcb,0x01,0xd5,0x53,0xc8,
    0xa8,0xa7,0x53,0x13,0x19,0x5d,0x67,0x3a,0xed,0x4e,0x53,0x31,0xa7,0xd3,0x37,0xa1,
    0x5a,0xe6,0xad,0x91,0x0d,0x36,0xa6,0x4b,0xbb,0x6c,0x9a,0x35,0x6b,0x69,0x29,0xad,
    0x46,0x97,0x8d,0x67,0xb5,0x4a,0x62,0x5e,0x98,0x9f,0x84,0xc1,0x72,0x8b,0xc7,0x87,
    0x61,0x3c,0x6a,0x34,0x46,0x4b,0x3c,0xe0,0xdc,0x47,0xb6,0x22,0x2a,0xd3,0x28,0x05,
    0xbd,0x3c,0x11,0x68,0x23,0xf6,0x7a,0xcb,0x38,0x3e,0x7a,0xc7,0x38,0x5e,0x9a,0xde,
    0xe9,0x4d,0x28,0x58,0xf1,0x36,0x8e,0xb7,0x27,0x1b,0xb5,0xce,0x74,0x1d,0x8f,0x8c,
    0xa6,0x62,0x85,0x96,0x56,0xef,0x9e,0x37,0x78,0x9e,0x18,0xa2,0xf2,0xc9,0x6d,0xdd,
    0x14,0xf0,0xb6,0xb6,0xa9,0x1b,0xd6,0xcc,0xa8,0x77,0xa6,0xeb,0x69,0xd7,0xb4,0xbc,
    0x78,0x20,0x28,0xd3,0x53,0x02,0xfa,0x75,0x1a,0x73,0xd0,0xcb,0x4e,0xc7,0x34,0xb0,
    0xdf,0xc0,0x30,0x3b,0x9d,0x25,0x8e,0xe9,0xcb,0x8d,0x6d,0x1a,0x96,0x8f,0x8f,0xda,
    0x74,0xb3,0x9e,0x6a,0x2d,0xc5,0x3a,0x5a,0x0a,0x0d,0x62,0xea,0x1d,0x6b,0xda,0xca,
    0x71,0x05,0x12,0xa0,0x58,0x54,0xd1,0xba,0xfa,0x00,0x3a,0xac,0x35,0xb4,0x69,0x68,
    0x15,0xef,0x82,0x46,0x2b,0x0b,0x0a,0x44,0x54,0x27,0xd0,0x83,0xe3,0x52,0x17,0x70,
    0x75,0xa1,0x05,0xd0,0x87,0xe5,0x61,0xa4,0x28,0xd6,0xc9,0xd2,0x7b,0xd4,0x13,0xa2,
    0xe4,0xba,0x0d,0x61,0xba,0xb7,0x3b,0x0a,0x6a,0x8b,0xba,0x69,0x85,0x53,0x32,0x6c,
    0x2b,0x1a,0xdf,0xd7,0x74,0x45,0x2d,0x5b,0x91,0x55,0x51,0x2b,0x1d,0x0e,0x7a,0xaa,
    0x54,0x54,0x11,0x47,0x70,0xb7,0x02,0x67,0x3b,0xed,0xa0,0xa7,0x6d,0x36,0x70,0xe7,
    0xd8,0x78,0x52,0x7e,0xa8,0x9f,0xca,0xd6,0x71,0x0b,0xb7,0x19,0x28,0xc2,0x0f,0x2b,
    0x9a,0x76,0x08,0xf4,0x7e,0x6a,0x94,0xef,0x6a,0xba,0x5a,0x00,0x44,0x1c,0xce,0x40,
    0x63,0xbd,0x81,0x8e,0x74,0xa7,0xc7,0xde,0x34,0xb2,0x08,0x74,0x0d,0x37,0x63,0x43,
    0xe3,0x69,0x39,0xb2,0x07,0xc8,0xcd,0x5e,0xc6,0xa7,0x28,0x86,0x0d,0x0e,0x4e,0xa9,
    0xa6,0x8d,0xee,0xff,0x05,0x34,0x5a,0xc7,0xa6,0x19,0x2f,0x0d,0x98,0x87,0x50,0x55,
    0xac,0x29,0xc6,0x3b,0x4d,0x3b,0x62,0x8e,0xba,0x16,0x0b,0xba,0x39,0xb0,0x3d,0xa0,
    0x01,0x0c,0xa3,0xce,0x40,0x2b,0x66,0xc6,0x1e,0x7f,0x03,0xba,0x6c,0xef,0x1d,0x03,
    0x6d,0x06,0x8a,0x1d,0xda,0x36,0xce,0xca,0x12,0xb6,0x34,0x0d,0xec,0xe5,0xd6,0x2e,
    0xcb,0xf9,0x61,0xe6,0x0f,0x29,0x60,0x1a,0x05,0x68,0xba,0xd7,0x79,0x01,0x71,0xd9,
    0x9a,0x06,0x4d,0xcb,0x2a,0x36,0x7f,0x49,0x1a,0x40,0x8b,0x75,0x19,0xa0,0xf5,0x77,
    0xa0,0xa3,0xa9,0x22,0x63,0xb5,0x11,0x0c,0xa9,0x03,0x74,0xcd,0x5c,0x2e,0x9d,0x7d,
    0xfa,0x98,0xb6,0x80,0x4b,0xca,0xd3,0x40,0x06,0x1d,0x78,0x4d,0x81,0x40,0xbf,0xc5,
    0xde,0xc0,0xd0,0x3b,0xd3,0xbd,0x05,0x18,0x47,0x8b,0x9f,0x56,0xf0,0x67,0x0d,0x17,
    0x15,0x02,0xc4,0x0d,0xe8,0x6e,0xbc,0xc4,0x6a,0x48,0x07,0xec,0xc1,0x41,0xeb,0xce,
    0x7e,0x19,0x5a,0x05,0x61,0x60,0xd2,0x23,0x22,0x76,0x1f,0x72,0x7b,0x7e,0x31,0x38,
    0x2f,0x91,0x92,0x5a,0x1e,0x40,0x82,0xb2,0x97,0x96,0x3d,0x5d,0x2f,0xcd,0xba,0x36,
    0xf5,0xb1,0x0b,0x0c,0x62,0x0a,0xd6,0x09,0x1f,0xc9,0x7c,0x0a,0x4f,0xf2,0x05,0xf4,
    0x1b,0xc2,0xe0,0x7a,0x69,0x59,0xf8,0x61,0xd4,0x01,0xda,0xd4,0xbb,0xfa,0x74,0xed,
    0x5b,0x82,0x79,0x5e,0x22,0x3d,0xb5,0x96,0xa1,0x5d,0xc8,0xcb,0x3c,0x2a,0xa2,0x39,
    0x85,0x17,0xc6,0x51,0x6c,0x6b,0x12,0xd4,0x16,0x45,0x61,0xb8,0x1c,0x35,0x5a,0xda,
    0x34,0xb2,0xe3,0x28,0x3a,0x81,0x96,0x35,0x3b,0x0e,0xc9,0xb1,0x50,0x9f,0xcb,0x05,
    0xfb,0x94,0x69,0x5a,0x30,0xa7,0xe1,0x3e,0xda,0x2f,0x07,0xca,0x8b,0xbe,0x3c,0x8d,
    0xcc,0xba,0x30,0x88,0x23,0xbb,0xf0,0x86,0xf1,0x4e,0x31,0xda,0xeb,0x62,0x29,0x3f,
    0x93,0xd6,0x06,0xd6,0x78,0x4c,0x94,0x57,0x04,0x69,0xd3,0xc7,0x11,0xb6,0xe5,0x69,
    0x96,0x3d,0xb2,0x2c,0xc7,0xb2,0x10,0x28,0x14,0x34,0xa0,0xf3,0x25,0xe4,0xdf,0xc8,
    0xed,0x71,0x77,0x5a,0x96,0x8d,0x87,0x2f,0x82,0x6e,0x39,0x63,0xcb,0x54,0xf0,0x38,
    0xdc,0x72,0x46,0xe5,0x42,0x49,0x41,0xf3,0x82,0xa0,0x74,0xd9,0x78,0x79,0x5a,0x34,
    0x8b,0x01,0x85,0x32,0x8b,0x33,0xf4,0x19,0x1f,0x0b,0xb4,0x8a,0xa3,0x28,0x0d,0x45,
    0x61,0x15,0x58,0x89,0x0e,0x16,0xee,0x24,0x6d,0x58,0x08,0x36,0xd0,0x8c,0x6a,0x58,
    0xbd,0x9b,0xd5,0x56,0x25,0xe1,0xb3,0xf6,0x7f,0x94,0xa9,0x2b,0xb4,0xda,0xc9,0xeb,
    0xef,0x12,0xd6,0x4e,0xab,0x7c,0xbb,0x0a,0x2b,0x4e,0xd9,0x16,0x48,0xd6,0x20,0xd9,
    0x41,0x2a,0xc9,0x8c,0x7c,0x65,0xb9,0xc8,0xb7,0x6d,0xb6,0x92,0xba,0x5b,0x65,0xb6,
    0x50,0xa1,0xb3,0x85,0x4a,0xf5,0xd2,0x5e,0xf8,0x4a,0x9b,0x7a,0x93,0xad,0xa6,0x5f,
    0xe8,0xa5,0xa8,0xaf,0xbf,0x15,0xf9,0xbf,0x65,0xd3,0xf7,0x63,0x6e,0xaf,0x7f,0xcc,
    0x17,0x19,0x1e,0xf3,0x95,0x91,0xc7,0x7c,0x39,0xe7,0x31,0x5f,0x83,0x7a,0xcc,0x17,
    0xce,0x1e,0xf3,0xd5,0xbe,0xc7,0x7c,0x89,0xf2,0x31,0x5f,0x57,0x7d,0xcc,0x17,0x83,
    0x1f,0xf3,0x15,0xec,0xc7,0x7c,0xd9,0xfd,0x31,0xbf,0x56,0xe0,0x31,0xbf,0xc0,0xe1,
    0x31,0xbf,0x2a,0xe3,0x31,0xbf,0x94,0xe4,0x31,0xbf,0xfe,0xe5,0x31,0xbf,0x68,0xe7,
    0x31,0xbf,0xd2,0xe8,0x31,0xbf,0x3c,0xea,0x31,0xbf,0xa6,0xeb,0x31,0xbf,0x10,0xed,
    0x5b,0xbe,0xe5,0x5b,0xfe,0x3f,0x0a,0xdf,0xac,0xff,0x48,0x80,0x65,0xbe,0x1b,0x80,
    0x6d,0x13,0xfe,0x54,0x24,0x92,0xdf,0x9e,0x42,0x92,0xfe,0x51,0xff,0x0f,0xc2,0x1e,
    0x31,0xa9,0x4d,0x35,0xfb,0xfc,0x00,0x22,0x09,0x6a,0x7f,0x3c,0x99,0xcc,0x27,0x93,
    0x71,0x5f,0xfd,0xf4,0x21,0xbb,0x24,0x0c,0xc7,0x90,0xb6,0x50,0xfc,0xad,0x29,0xda,
    0x63,0x26,0xc3,0x3f,0xa4,0x15,0x59,0xe8,0x4f,0x0e,0x51,0x18,0x04,0x41,0x18,0x1d,
    0x26,0xfd,0xcf,0x46,0x95,0x85,0xf1,0x29,0x3c,0x1c,0xda,0xbf,0xb5,0x75,0xa0,0x2a,
    0x0c,0xa9,0x73,0x78,0x9a,0x54,0xc5,0x3f,0x83,0x79,0x18,0x85,0xde,0xc2,0xf5,0x7c,
    0xcf,0x5d,0x78,0x61,0x34,0x14,0x9e,0x3e,0x03,0x7d,0x70,0x3d,0xef,0xf7,0x40,0x17,
    0x85,0x49,0xe8,0x2e,0x16,0xde,0x22,0xe8,0xff,0x09,0x55,0x43,0xcf,0x91,0xe7,0xf9,
    0x41,0x14,0xec,0xf0,0xdf,0xf7,0xfc,0xe8,0x6a,0x58,0xf1,0x8a,0x52,0x00,0x3a,0x5a,
    0xf8,0xfe,0x05,0xf4,0x7d,0xba,0xb9,0x7b,0x10,0xd6,0xe1,0xed,0xfc,0x7d,0xe0,0x2f,
    0xa0,0x93,0xea,0xff,0xaa,0xcf,0xaf,0xc8,0x4c,0x14,0xd5,0x49,0xe0,0xf9,0xe1,0x64,
    0xd8,0x6f,0xf7,0x87,0xf3,0xd0,0xf3,0xc2,0x49,0xb5,0x98,0x7c,0xd9,0x09,0x03,0x2f,
    0x25,0x7f,0xde,0x82,0x4e,0x8e,0x66,0x74,0xc3,0xc8,0x41,0x14,0xe8,0x4f,0xf1,0xae,
    0x75,0x44,0x8b,0xc0,0xc5,0x4c,0xc1,0x44,0xcd,0x1a,0x50,0x97,0x22,0xf6,0xad,0xb2,
    0xcd,0xab,0xc5,0x9b,0x71,0xa4,0x0a,0x9b,0x43,0xfc,0x54,0xd1,0x21,0xf4,0xec,0xb6,
    0xd9,0x5f,0xed,0x49,0xe0,0xfb,0xc1,0x1b,0xc3,0xc5,0xfe,0x37,0x9b,0xd5,0x94,0x51,
    0xae,0x41,0xb3,0x6d,0x1a,0x09,0xdd,0x48,0xb7,0x4f,0x49,0x9b,0xea,0x3d,0xeb,0x10,
    0x27,0xe1,0x22,0x1a,0x0f,0x8f,0xb0,0xbf,0x77,0xf6,0x41,0x60,0x59,0x1f,0xf9,0x76,
    0x1c,0x99,0x1d,0x94,0x3e,0xf3,0x90,0x68,0x81,0xf1,0x04,0x55,0x96,0xb0,0xc9,0x61,
    0x18,0xb9,0x8b,0x43,0x9f,0xee,0xa0,0x2c,0x54,0x41,0x29,0xf3,0xf9,0x64,0x3c,0x6c,
    0xb2,0x01,0xaf,0x40,0x43,0x01,0xed,0x61,0x42,0x37,0x32,0x0d,0xfc,0x24,0xf4,0x39,
    0xb3,0x0c,0x71,0xb0,0xff,0x61,0x2a,0x58,0x87,0xef,0x2f,0x8e,0xc3,0x76,0xe8,0xd3,
    0x54,0x4f,0x57,0x7c,0x32,0x14,0x9a,0x43,0x9a,0x65,0xd8,0xa6,0x49,0x92,0x71,0xfa,
    0x38,0x37,0xa1,0x99,0xab,0xf7,0x51,0x03,0xf4,0x71,0xb1,0x38,0x8c,0x05,0x6c,0xdb,
    0x10,0x45,0x49,0x6d,0x43,0x9a,0x40,0x45,0x94,0x82,0x27,0xf7,0xfb,0x7d,0x70,0x88,
    0x38,0xa3,0x5c,0x40,0x8b,0x82,0x3a,0xf6,0x41,0x37,0xbb,0x7d,0x18,0x45,0x13,0xba,
    0x0a,0x46,0x0e,0xe1,0xe9,0x0d,0x3f,0x83,0x63,0xff,0xc3,0xbe,0x35,0x36,0x89,0x17,
    0xf5,0x8b,0x73,0xd8,0xc7,0xbc,0xc9,0xec,0x43,0x16,0xde,0x8e,0x87,0x30,0x1e,0x36,
    0x27,0x47,0xe2,0xad,0xa3,0x4b,0x93,0x24,0xe3,0xf4,0xfb,0x8b,0x88,0x1d,0x9c,0xdc,
    0x77,0xfb,0xaa,0xd0,0x3f,0x78,0xbe,0xef,0xf5,0x99,0x05,0x54,0xae,0xac,0xc6,0x0b,
    0xe1,0xed,0x41,0xe8,0xbb,0x8b,0x80,0x31,0x4a,0x06,0x5a,0x96,0xd4,0xf1,0xd1,0x77,
    0x41,0x34,0x87,0x3d,0xae,0x17,0x96,0x25,0x61,0x32,0x30,0xcb,0x62,0x08,0x12,0x5a,
    0xf8,0x1f,0x41,0x8b,0xd2,0x24,0xf4,0xf6,0x68,0x38,0x3e,0x2c,0xbc,0x43,0x5f,0xe4,
    0x26,0xf7,0x06,0xba,0x3a,0x0c,0xc7,0x47,0x2f,0xd8,0x13,0xad,0x00,0x82,0xcc,0xc6,
    0x59,0xf8,0xc3,0x49,0xb8,0x83,0xd3,0x7a,0x6e,0x34,0x69,0xde,0x8b,0xa1,0xa2,0xd0,
    0xa6,0x17,0x67,0xf7,0xe1,0xb8,0xdf,0x26,0xf3,0x82,0xc2,0xc9,0x0f,0xc4,0xb6,0x1b,
    0x2c,0xfc,0x20,0x98,0x4c,0x5c,0xf8,0xa6,0x47,0x8c,0x92,0x81,0x26,0xbd,0xb9,0x6e,
    0xe0,0xe1,0x3e,0xba,0x81,0x8f,0x81,0x45,0x89,0x19,0xd9,0x6e,0x31,0xdf,0x13,0x15,
    0x7f,0x00,0xcd,0xac,0x03,0x0e,0x2e,0xa7,0xa6,0xc8,0xad,0xed,0x0d,0xfe,0xb3,0x47,
    0x97,0xf9,0xc4,0x3d,0xc0,0x7a,0xc2,0x89,0x2a,0xb2,0x71,0xfc,0x9d,0x4b,0x13,0xfb,
    0xf0,0x35,0xf7,0x38,0xbe,0xcb,0x90,0x32,0xbb,0x75,0xbe,0x7f,0x88,0x16,0xb0,0xd0,
    0xb6,0x70,0xb1,0x5f,0x4c,0xd3,0xaf,0x16,0xdb,0x63,0xc6,0x28,0xaa,0x98,0x81,0x06,
    0xdf,0x84,0xe1,0x71,0xfe,0xc6,0x1d,0xd7,0xa3,0x43,0x2a,0x9b,0x0c,0x17,0x39,0xee,
    0xf7,0x31,0x46,0xb5,0xc2,0xdf,0x77,0x92,0x39,0xff,0x70,0x13,0xc4,0xed,0x22,0xe2,
    0xf3,0x02,0x97,0xd9,0x07,0x07,0xed,0x07,0xfe,0xb0,0xa9,0xb6,0x87,0x21,0x50,0x43,
    0x33,0xc9,0x38,0x98,0x58,0x55,0x69,0x62,0xff,0x9a,0x62,0xdf,0xa1,0x3e,0xc0,0x41,
    0xdc,0x05,0x2c,0xf4,0xe0,0x8e,0xc9,0x21,0xa0,0x1a,0x77,0x8f,0x61,0xb8,0xfd,0xe2,
    0xd6,0x92,0x73,0xaa,0x99,0x4d,0x57,0x9b,0x6f,0xfd,0x21,0x0e,0xa8,0x2a,0xbb,0xe0,
    0x43,0x36,0x59,0xd0,0xbf,0x4f,0xab,0x95,0x09,0x5d,0x38,0xee,0x16,0x99,0x09,0xc0,
    0x89,0xd5,0x14,0xb4,0x77,0x18,0xb2,0x26,0xe3,0x83,0xe7,0xc1,0xb1,0x92,0x71,0xb8,
    0xb1,0xb2,0x89,0xef,0xf2,0x3a,0xb3,0x90,0xfe,0x24,0x38,0x86,0x01,0x5e,0x9c,0xf1,
    0x82,0x68,0xc1,0x4c,0xab,0xcf,0xef,0x23,0xda,0xab,0x5c,0x4d,0xe3,0x2b,0xd0,0x57,
    0x1e,0xc1,0x8d,0x94,0x4f,0xe6,0x31,0x77,0x96,0xab,0x45,0x75,0x38,0x4e,0x05,0xfe,
    0x2f,0x32,0x78,0xe4,0x80,0x92,0x2a,0xb1,0x8b,0x1c,0x33,0x4a,0xe3,0x47,0x27,0x60,
    0x2d,0xb1,0x2a,0x91,0xfd,0x04,0x73,0x55,0xcc,0xc6,0x41,0xfc,0x48,0x3f,0x4b,0x9f,
    0xed,0x41,0x79,0x1b,0xc2,0x78,0x8f,0xd0,0xc7,0x62,0x7f,0x4c,0x30,0xe0,0x22,0x8b,
    0x00,0x0d,0xf3,0x66,0x06,0x89,0x0b,0xc8,0x40,0xc3,0x66,0x55,0x84,0xa2,0xe1,0x78,
    0xc2,0x94,0x75,0xe9,0x00,0xe3,0x86,0x0e,0x9a,0xde,0x31,0x62,0x72,0x38,0xba,0xaa,
    0x20,0x66,0xac,0x4a,0xf6,0x44,0x76,0x4c,0xae,0xcb,0x41,0x27,0x06,0x2e,0x0a,0x55,
    0x68,0x75,0x87,0x14,0xe1,0x76,0x62,0x8f,0x7b,0xc2,0x9d,0x10,0x03,0xd3,0x53,0xe9,
    0x62,0x80,0x62,0x12,0xd2,0xc5,0xbb,0xf0,0x48,0x66,0xd2,0x8c,0xe9,0x8a,0x42,0xd3,
    0xa5,0x40,0x86,0xe8,0x94,0x82,0x96,0x88,0xa5,0x3d,0xca,0x7f,0xc2,0xfd,0x2d,0x68,
    0x81,0x83,0x9e,0x78,0xfc,0x7d,0x29,0xd7,0x43,0xfc,0x2b,0x8a,0x55,0x66,0x60,0x43,
    0x62,0x53,0x72,0x02,0x10,0x36,0xda,0x71,0xd0,0xc7,0x21,0xa3,0x12,0x89,0xec,0x83,
    0xa2,0x1a,0xd7,0x6e,0x9f,0x5f,0x09,0x9f,0x58,0x15,0x3e,0x8b,0xe7,0x32,0x0b,0xa3,
    0xd5,0xbe,0xeb,0xb3,0xf4,0x00,0xf8,0xbc,0x04,0x34,0xfa,0xce,0x59,0x5f,0xf1,0xc2,
    0x1e,0x7d,0xf7,0x18,0x82,0x70,0x0e,0x07,0x0f,0xa6,0x7f,0x07,0xb4,0x1b,0x85,0x5c,
    0x8e,0x73,0x95,0x9a,0x93,0xc7,0xf9,0x2e,0x12,0x26,0x62,0x44,0x76,0xdb,0x8a,0x19,
    0x68,0x66,0xb2,0xb2,0x48,0x46,0x8d,0x09,0x19,0xe8,0x9b,0x89,0x13,0x5a,0xbf,0xb5,
    0x67,0xb5,0x4f,0x0e,0x8f,0xec,0xa4,0x28,0x57,0x91,0x32,0x1f,0x3c,0x8a,0x34,0x4c,
    0xd3,0x20,0xd4,0x0a,0x69,0x1a,0x4e,0x89,0x79,0xa0,0xdf,0x04,0x74,0xb1,0x39,0x0f,
    0xc1,0xd2,0xf3,0xf1,0x70,0xd8,0xbe,0x76,0xc4,0xcc,0x69,0xaa,0xfd,0x4c,0xde,0x24,
    0x1e,0x73,0x01,0x95,0x7d,0xa3,0x80,0xef,0xd1,0x6f,0x4a,0x15,0x33,0xf3,0x60,0xa9,
    0x07,0x92,0x40,0xcf,0xdf,0x27,0x9a,0xe6,0x9e,0xca,0x27,0xbe,0xa7,0x69,0xdc,0xe9,
    0xe0,0x10,0x9c,0x60,0xb1,0x74,0xa6,0xaa,0xbe,0x85,0xcc,0x8b,0x99,0xf3,0x71,0x2d,
    0xc8,0x52,0x9f,0x1f,0xcb,0x6c,0x9a,0x68,0xda,0x0b,0xc7,0x4d,0xea,0x7f,0x17,0xf4,
    0x7b,0xb5,0x10,0x2f,0xee,0x0e,0x4c,0xc2,0x43,0xb0,0xe3,0xfa,0xe5,0x8e,0x08,0xa5,
    0xb3,0x54,0x86,0x2c,0x81,0x6c,0x9d,0x8d,0x73,0x3d,0x31,0x1a,0x88,0xf7,0x86,0x44,
    0xe8,0x21,0x8e,0x91,0x69,0x47,0x54,0xea,0x32,0xbc,0xbd,0x28,0x57,0x65,0x22,0x0d,
    0xc6,0x3c,0x19,0x7b,0xd0,0x6f,0x22,0x5b,0xfa,0xca,0xa5,0xfb,0xa0,0xe5,0x8b,0x24,
    0x19,0xd9,0x6e,0x91,0xf2,0xc9,0x84,0x41,0x95,0x8a,0x09,0x4f,0xfb,0x6f,0xc8,0x7a,
    0x54,0x62,0x2b,0xf2,0xb9,0x0a,0xd3,0x34,0x69,0x17,0xbb,0x41,0xf9,0xc4,0xe3,0x3b,
    0x8a,0xa8,0x30,0x28,0x81,0xdb,0xa7,0x9c,0xaa,0x39,0xf4,0xb9,0x4d,0x3f,0x11,0x47,
    0x26,0x1c,0xda,0xf7,0xf9,0x0d,0xad,0xbe,0x07,0x4d,0xce,0x3b,0x0f,0xfc,0xbf,0xd1,
    0x34,0xba,0x71,0x9a,0x4b,0x84,0x87,0x14,0xbc,0x2e,0xc0,0x79,0x3a,0x9c,0x34,0xf9,
    0x38,0x89,0x62,0xc8,0x94,0xd0,0x9a,0x98,0xa1,0x9f,0x50,0x53,0xe5,0x4e,0x68,0x21,
    0x46,0xf3,0x83,0x70,0x02,0x2d,0xcc,0x0f,0x44,0x97,0x6e,0x13,0x26,0xd5,0x3f,0x52,
    0x64,0x42,0x80,0x1b,0x93,0x0f,0x51,0x34,0x95,0xaf,0xcc,0x03,0x6a,0x19,0xb7,0xd5,
    0x26,0xca,0x34,0xff,0x6f,0x40,0x73,0xeb,0xc0,0x1d,0x07,0x49,0x43,0xb0,0x3b,0x2e,
    0x09,0x19,0x2a,0x87,0xbf,0x47,0x26,0x3f,0x1c,0xba,0x4c,0xe9,0x97,0xc8,0x4a,0x07,
    0xc7,0xfc,0x9a,0xd4,0x7b,0xc5,0x00,0x85,0x71,0x32,0x10,0x64,0x73,0x87,0x80,0x78,
    0x9a,0x55,0x2e,0x32,0xd3,0x8f,0x87,0x63,0x91,0x8f,0x39,0x28,0x6f,0xb9,0x80,0x6e,
    0xa3,0x9d,0x1f,0xba,0x93,0x79,0x10,0xcd,0xc1,0xae,0xbf,0x04,0xcd,0xad,0x83,0x05,
    0xd7,0x4b,0x2a,0x4c,0x85,0x86,0x94,0x04,0x17,0x37,0x3c,0x44,0xd1,0x9e,0x14,0x33,
    0x64,0x09,0x13,0xe6,0xd8,0x23,0xef,0x00,0xcb,0xfb,0x34,0xf3,0x27,0xd5,0x19,0x50,
    0x7b,0xa8,0xb3,0x16,0xa0,0x55,0xd8,0x09,0xa5,0x88,0xcc,0x35,0xd4,0x71,0x84,0x0c,
    0x8f,0xb8,0x36,0xe0,0x19,0x62,0x56,0x23,0x52,0x2a,0x81,0xf0,0xe9,0x05,0x01,0x92,
    0x56,0x5c,0x66,0xc8,0x40,0x1f,0x5c,0xa4,0x6c,0x1f,0x41,0x3f,0x51,0xaf,0x05,0x23,
    0xb9,0x64,0xb6,0x37,0x52,0x0d,0x05,0x7c,0x4e,0x79,0x50,0x19,0x26,0xf6,0xf6,0xc7,
    0x71,0x55,0xe6,0x71,0x08,0xec,0x31,0x3e,0x2d,0xe8,0x20,0xa5,0x97,0xf2,0xe7,0xe5,
    0xfd,0x24,0x88,0xa2,0xe3,0x31,0x0a,0x92,0x64,0x9c,0xd5,0x4c,0x08,0xee,0x74,0x0c,
    0xb7,0x4a,0xe5,0x71,0x77,0x7c,0xc2,0x8b,0xc1,0x94,0x9a,0x22,0xd7,0x87,0x2a,0x0e,
    0xc1,0xb8,0xfd,0x76,0xa4,0xf4,0x97,0x40,0x9f,0x82,0xe0,0xf4,0x11,0x34,0x27,0xed,
    0xeb,0x33,0x08,0x35,0x48,0x19,0x4e,0x63,0x21,0xa5,0xbc,0xa1,0x7b,0x38,0x1e,0x0f,
    0xee,0x90,0x22,0x5f,0x12,0x3c,0xfb,0xf2,0xd8,0x8b,0x68,0xe6,0x5f,0x54,0xc1,0x38,
    0xa1,0x22,0x05,0x1a,0x0e,0xfb,0x6f,0x6a,0x56,0xf6,0x88,0x45,0x16,0x24,0x71,0x50,
    0xe5,0x55,0x0f,0x92,0x58,0x62,0x5e,0x95,0x95,0x48,0x4d,0x3a,0x03,0x5a,0x6d,0xd2,
    0xa1,0x26,0x5d,0x37,0xa3,0xfb,0x0f,0x6b,0x22,0x49,0x1c,0xe8,0x5f,0x22,0x44,0x32,
    0xcc,0x9b,0x94,0x80,0xe6,0x63,0xa1,0x01,0xb5,0xa8,0xa6,0x66,0xc6,0xa6,0xfe,0xc5,
    0x22,0x0c,0xe9,0xfa,0xe2,0xa0,0x15,0xe9,0x76,0xdd,0x89,0xef,0x45,0xbe,0xbb,0x24,
    0x95,0x94,0x8a,0xbf,0xbd,0x0a,0x70,0x15,0x5c,0x58,0x50,0xbc,0x14,0x7f,0x43,0x51,
    0x7d,0x07,0xe1,0x93,0xc5,0x3c,0xf6,0x2d,0xa9,0x92,0x78,0x5b,0x70,0xd0,0xb1,0xcb,
    0xf2,0x9e,0x94,0x7c,0x27,0x40,0xda,0x9e,0x6c,0x3f,0x3d,0x54,0xa4,0x0f,0xc5,0xbb,
    0x23,0x5f,0x7a,0x5d,0x0d,0x73,0x89,0x88,0x72,0x11,0xaf,0x89,0x26,0x5d,0x33,0x4d,
    0x57,0xa5,0x9b,0x99,0xbf,0x8a,0x5c,0x67,0x79,0xef,0xcb,0xec,0x61,0x6e,0x5f,0xb3,
    0xf8,0xcf,0x41,0x53,0x8d,0xf8,0x01,0xf4,0x27,0x2c,0xf4,0x65,0x40,0x87,0xc1,0xfe,
    0xf8,0x01,0xf4,0x71,0xbf,0x3f,0x7e,0x61,0xd0,0xed,0x09,0x64,0xf8,0x0e,0x74,0x9f,
    0x0e,0xf6,0xbf,0x2a,0x68,0x38,0x24,0x8a,0x4c,0xf5,0x3d,0xba,0x2a,0x1d,0x7c,0x94,
    0x35,0xf2,0x47,0x7a,0xca,0xf3,0x31,0x1b,0x12,0x1f,0xee,0xd9,0xcf,0xb7,0x7c,0xcb,
    0xb7,0x7c,0xcb,0xb7,0x7c,0xcb,0xb7,0xfc,0x97,0xca,0xff,0x00,0x72,0xcc,0x4f,0x46,
    0x37,0x70,0xf6,0x6f,0x00,0x00,0x00,0x00,0x49,0x45,0x4e,0x44,0xae,0x42,0x60,0x82,
};

// Icon für den Home-Bildschirm (iOS) – ohne Login abrufbar, 1 Tag cachebar
static esp_err_t apple_icon_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, (const char *)APPLE_ICON, sizeof(APPLE_ICON));
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
    config.max_uri_handlers = 24;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Webserver konnte nicht gestartet werden!");
        return;
    }

    httpd_uri_t uris[] = {
        { .uri = "/",                 .method = HTTP_GET,  .handler = root_get_handler },
        { .uri = "/set",              .method = HTTP_POST, .handler = set_post_handler },
        { .uri = "/options",          .method = HTTP_GET,  .handler = options_get_handler },
        { .uri = "/info",             .method = HTTP_GET,  .handler = info_get_handler },
        { .uri = "/chart",            .method = HTTP_GET,  .handler = chart_get_handler },
        { .uri = "/api/status",       .method = HTTP_GET,  .handler = api_status_handler },
        { .uri = "/api/info",         .method = HTTP_GET,  .handler = api_info_handler },
        { .uri = "/api/history",      .method = HTTP_GET,  .handler = api_history_handler },
        { .uri = "/config/ip",        .method = HTTP_POST, .handler = config_ip_post_handler },
        { .uri = "/config/pass",      .method = HTTP_POST, .handler = config_pass_post_handler },
        { .uri = "/config/poll",      .method = HTTP_POST, .handler = config_poll_post_handler },
        { .uri = "/config/wallbox",   .method = HTTP_POST, .handler = config_wallbox_post_handler },
        { .uri = "/config/soclimits", .method = HTTP_POST, .handler = config_soclimits_post_handler },
        { .uri = "/config/calc",      .method = HTTP_POST, .handler = config_calc_post_handler },
        { .uri = "/ota",              .method = HTTP_POST, .handler = ota_post_handler },
        { .uri = "/favicon.ico",      .method = HTTP_GET,  .handler = favicon_get_handler },
        { .uri = "/apple-touch-icon.png", .method = HTTP_GET, .handler = apple_icon_get_handler },
        { .uri = "/apple-touch-icon-precomposed.png", .method = HTTP_GET, .handler = apple_icon_get_handler },
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
        s_eth_link = false;
        ESP_LOGW(TAG, "Ethernet-Link getrennt");
    } else if (id == ETHERNET_EVENT_CONNECTED) {
        s_eth_link = true;
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
    cache_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Ethernet starten (kein festes Warten: Modbus-Task wartet auf das GOT_IP-Event)
    s_eth_netif = init_ethernet();
    ESP_LOGI(TAG, "Ethernet-Init abgeschlossen, starte WLAN...");

    // WLAN-Hotspot parallel starten
    init_wifi_softap();
    ESP_LOGI(TAG, "WLAN-Init abgeschlossen, starte Modbus-Task...");

    // Modbus-Task: wartet auf IP, initialisiert Modbus, pollt zyklisch
    xTaskCreate(modbus_task, "modbus_task", 8192, NULL, 5, &s_modbus_task);

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
