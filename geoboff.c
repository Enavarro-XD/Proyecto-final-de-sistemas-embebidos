// lora_rx_geofence.c - Receptor LoRa con Geofence
// Milk-V Duo 256M - Auto-detección SPI + Validación Geográfica

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <math.h>

// ==================== CONFIGURACIÓN GEOFENCE ====================
#define BASE_LAT    6.25184      // CAMBIAR: Tu latitud base
#define BASE_LON    -75.56359    // CAMBIAR: Tu longitud base
#define RADIO_MAX   500.0        // CAMBIAR: Radio máximo en metros

// ==================== REGISTROS SX1278 ====================
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_LNA                 0x0C
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_SNR_VALUE       0x19
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_MODEM_CONFIG_3      0x26
#define REG_SYNC_WORD           0x39
#define REG_VERSION             0x42

// Modos
#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_RX_CONTINUOUS      0x05

// IRQ
#define IRQ_RX_DONE             0x40
#define IRQ_CRC_ERROR           0x20

static int spi_fd = -1;
static volatile int running = 1;
static uint32_t working_speed = 0;
static uint8_t working_mode = 0;

// ==================== FUNCIONES GEOFENCE ====================

double toRad(double degree) {
    return degree * M_PI / 180.0;
}

double calcularDistancia(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371000.0; // Radio de la Tierra en metros
    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);
    
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(toRad(lat1)) * cos(toRad(lat2)) *
               sin(dLon/2) * sin(dLon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

void verificarGeofence(const char *id, double lat, double lon) {
    double distancia = calcularDistancia(BASE_LAT, BASE_LON, lat, lon);
    
    printf("┌─────────────────────────────────────────────┐\n");
    printf("│ ANÁLISIS GEOFENCE                           │\n");
    printf("├─────────────────────────────────────────────┤\n");
    printf("│ ID Animal: %-32s │\n", id);
    printf("│ Lat: %12.6f   Lon: %12.6f │\n", lat, lon);
    printf("│ Distancia: %10.2f metros              │\n", distancia);
    printf("│ Radio Max: %10.0f metros              │\n", RADIO_MAX);
    printf("├─────────────────────────────────────────────┤\n");
    
    if (distancia > RADIO_MAX) {
        printf("│ Estado: ⚠️  ALERTA - FUERA DE ZONA          │\n");
        printf("│ Exceso: %10.2f metros              │\n", distancia - RADIO_MAX);
        // Aquí puedes agregar: envío de SMS, Email, Telegram, etc.
    } else {
        printf("│ Estado: ✓  OK - DENTRO DE ZONA             │\n");
        printf("│ Margen: %10.2f metros              │\n", RADIO_MAX - distancia);
    }
    
    printf("└─────────────────────────────────────────────┘\n\n");
}

// ==================== SIGNAL HANDLER ====================

void signal_handler(int sig) {
    (void)sig;
    running = 0;
    printf("\n[INFO] Saliendo...\n");
}

// ==================== SPI BÁSICO ====================

int spi_open_with_config(const char *device, uint8_t mode, uint32_t speed) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    uint8_t bits = 8;
    uint8_t lsb = 0;  // MSB first

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        close(fd);
        return -1;
    }

    ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb);
    return fd;
}

uint8_t spi_read_register(uint8_t reg, uint32_t speed) {
    usleep(200);
    
    uint8_t tx[2] = {reg & 0x7F, 0x00};
    uint8_t rx[2] = {0, 0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = speed,
        .bits_per_word = 8,
        .delay_usecs = 100,
        .cs_change = 0,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        return 0;
    }

    usleep(200);
    return rx[1];
}

void spi_write_register(uint8_t reg, uint8_t value, uint32_t speed) {
    usleep(200);
    
    uint8_t tx[2] = {reg | 0x80, value};
    uint8_t rx[2] = {0, 0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = speed,
        .bits_per_word = 8,
        .delay_usecs = 100,
        .cs_change = 0,
    };

    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    usleep(200);
}

// ==================== AUTO-DETECCIÓN ====================

int auto_detect_spi_config(void) {
    printf("\n[INFO] Buscando configuración correcta de SPI...\n");
    printf("       Esto puede tardar unos segundos...\n\n");

    uint32_t speeds[] = {50000, 100000, 200000, 500000};
    uint8_t modes[] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};

    for (int m = 0; m < 4; m++) {
        for (int s = 0; s < 4; s++) {
            if (spi_fd >= 0) {
                close(spi_fd);
            }

            spi_fd = spi_open_with_config("/dev/spidev0.0", modes[m], speeds[s]);
            if (spi_fd < 0) {
                continue;
            }

            uint8_t version = spi_read_register(REG_VERSION, speeds[s]);

            if (version == 0x12 || version == 0x11 || version == 0x24) {
                spi_write_register(REG_SYNC_WORD, 0x55, speeds[s]);
                usleep(1000);
                uint8_t test_read = spi_read_register(REG_SYNC_WORD, speeds[s]);

                if (test_read == 0x55) {
                    printf("[OK] ¡Configuración encontrada!\n");
                    printf("     Modo SPI: %d\n", modes[m]);
                    printf("     Velocidad: %u Hz\n", speeds[s]);
                    printf("     Versión chip: 0x%02X\n", version);
                    
                    working_mode = modes[m];
                    working_speed = speeds[s];
                    
                    spi_write_register(REG_SYNC_WORD, 0x12, speeds[s]);
                    return 0;
                }
            }
        }
    }

    printf("[ERROR] No se encontró configuración válida\n");
    return -1;
}

// ==================== LORA ====================

void lora_set_mode(uint8_t mode) {
    spi_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode, working_speed);
    usleep(10000);
}

void lora_set_frequency(long frequency) {
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    
    spi_write_register(REG_FRF_MSB, (uint8_t)(frf >> 16), working_speed);
    spi_write_register(REG_FRF_MID, (uint8_t)(frf >> 8), working_speed);
    spi_write_register(REG_FRF_LSB, (uint8_t)(frf >> 0), working_speed);
    
    printf("[OK] Frecuencia: %ld Hz\n", frequency);
}

void lora_set_spreading_factor(int sf) {
    if (sf < 6 || sf > 12) sf = 7;
    
    uint8_t config2 = spi_read_register(REG_MODEM_CONFIG_2, working_speed);
    config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
    spi_write_register(REG_MODEM_CONFIG_2, config2, working_speed);
    
    printf("[OK] SF: %d\n", sf);
}

void lora_set_bandwidth(long bw) {
    int bw_val;
    
    if (bw <= 7800) bw_val = 0;
    else if (bw <= 10400) bw_val = 1;
    else if (bw <= 15600) bw_val = 2;
    else if (bw <= 20800) bw_val = 3;
    else if (bw <= 31250) bw_val = 4;
    else if (bw <= 41700) bw_val = 5;
    else if (bw <= 62500) bw_val = 6;
    else if (bw <= 125000) bw_val = 7;
    else if (bw <= 250000) bw_val = 8;
    else bw_val = 9;
    
    uint8_t config1 = spi_read_register(REG_MODEM_CONFIG_1, working_speed);
    config1 = (config1 & 0x0F) | (bw_val << 4);
    spi_write_register(REG_MODEM_CONFIG_1, config1, working_speed);
    
    printf("[OK] BW: %ld Hz\n", bw);
}

void lora_set_coding_rate(int cr) {
    if (cr < 5) cr = 5;
    if (cr > 8) cr = 8;
    
    int cr_val = cr - 4;
    
    uint8_t config1 = spi_read_register(REG_MODEM_CONFIG_1, working_speed);
    config1 = (config1 & 0xF1) | (cr_val << 1);
    spi_write_register(REG_MODEM_CONFIG_1, config1, working_speed);
    
    printf("[OK] CR: 4/%d\n", cr);
}

int lora_init(void) {
    printf("\n[INFO] Inicializando LoRa...\n");
    
    lora_set_mode(MODE_SLEEP);
    
    // Debe coincidir con la ESP32
    lora_set_frequency(433000000);
    lora_set_spreading_factor(12);
    lora_set_bandwidth(62500);
    lora_set_coding_rate(8);
    
    // MODEM_CONFIG_3: AGC + LowDataRateOptimize
    spi_write_register(REG_MODEM_CONFIG_3, 0x0C, working_speed);
    printf("[OK] MODEM_CONFIG_3: AGC + LowDataRateOptimize\n");

    // CRC OFF (debe coincidir con LoRa.disableCrc() en la ESP32)
    uint8_t config2 = spi_read_register(REG_MODEM_CONFIG_2, working_speed);
    config2 &= ~0x04;
    spi_write_register(REG_MODEM_CONFIG_2, config2, working_speed);
    printf("[OK] CRC: OFF\n");
    
    // Sync word igual al de la ESP32
    spi_write_register(REG_SYNC_WORD, 0x12, working_speed);
    
    uint8_t lna = spi_read_register(REG_LNA, working_speed);
    lna |= 0x03;
    spi_write_register(REG_LNA, lna, working_speed);
    
    spi_write_register(REG_FIFO_TX_BASE_ADDR, 0x00, working_speed);
    spi_write_register(REG_FIFO_RX_BASE_ADDR, 0x00, working_speed);
    
    lora_set_mode(MODE_STDBY);
    
    printf("[OK] LoRa inicializado\n");
    printf("     SF12 + BW 62.5kHz + CR 4/8 + CRC OFF + LDRO + SYNC 0x12\n\n");
    return 0;
}

int lora_receive_packet(uint8_t *buf, int max_len) {
    spi_write_register(REG_IRQ_FLAGS, 0xFF, working_speed);
    lora_set_mode(MODE_RX_CONTINUOUS);
    
    for (int i = 0; i < 50 && running; i++) {
        uint8_t irq = spi_read_register(REG_IRQ_FLAGS, working_speed);
        
        if (irq & IRQ_RX_DONE) {
            if (irq & IRQ_CRC_ERROR) {
                printf("[WARN] CRC error - descartando paquete\n");
                spi_write_register(REG_IRQ_FLAGS, 0xFF, working_speed);
                return -1;
            }
            
            int len = spi_read_register(REG_RX_NB_BYTES, working_speed);
            uint8_t fifo_addr = spi_read_register(REG_FIFO_RX_CURRENT, working_speed);
            
            if (len > max_len) {
                printf("[WARN] Paquete demasiado grande: %d bytes (max: %d)\n", len, max_len);
                len = max_len;
            }
            
            spi_write_register(REG_FIFO_ADDR_PTR, fifo_addr, working_speed);
            usleep(1000);
            
            usleep(500);
            uint8_t tx[256] = {REG_FIFO & 0x7F};
            uint8_t rx[256] = {0};
            
            struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = len + 1,
                .speed_hz = working_speed,
                .bits_per_word = 8,
                .delay_usecs = 100,
                .cs_change = 0,
            };
            
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
                printf("[ERROR] Fallo al leer FIFO\n");
                spi_write_register(REG_IRQ_FLAGS, 0xFF, working_speed);
                return -1;
            }
            
            memcpy(buf, &rx[1], len);

            int rssi = spi_read_register(REG_PKT_RSSI_VALUE, working_speed) - 164;
            int snr = (int8_t)spi_read_register(REG_PKT_SNR_VALUE, working_speed) / 4;
            
            printf("[RX] %d bytes | RSSI: %d dBm | SNR: %d dB\n", len, rssi, snr);
            
            spi_write_register(REG_IRQ_FLAGS, 0xFF, working_speed);
            
            return len;
        }
        
        usleep(100000);
    }
    
    return 0;
}

// ==================== MAIN ====================

int main(void) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("=================================================\n");
    printf("  Gateway LoRa con Geofence\n");
    printf("  Milk-V Duo 256M\n");
    printf("=================================================\n");
    printf("  Base: Lat %.5f, Lon %.5f\n", BASE_LAT, BASE_LON);
    printf("  Radio: %.0f metros\n", RADIO_MAX);
    printf("=================================================\n");
    
    if (auto_detect_spi_config() != 0) {
        fprintf(stderr, "\n[ERROR] No se pudo detectar el chip LoRa\n");
        fprintf(stderr, "[AYUDA] Verifica:\n");
        fprintf(stderr, "  1. Conexiones físicas correctas\n");
        fprintf(stderr, "  2. Alimentación 3.3V en Ra-02\n");
        fprintf(stderr, "  3. Prueba intercambiar MISO/MOSI físicamente\n\n");
        return 1;
    }
    
    if (lora_init() != 0) {
        close(spi_fd);
        return 1;
    }
    
    printf("[INFO] Esperando paquetes GPS... (Ctrl+C para salir)\n");
    printf("[INFO] Formato esperado: ANIMAL_ID,LAT,LON\n");
    printf("[INFO] Ejemplo: VACA_01,6.250000,-75.560000\n\n");
    
    uint8_t buffer[256];
    int count = 0;
    
    while (running) {
        int len = lora_receive_packet(buffer, sizeof(buffer) - 1);
        
        if (len > 0) {
            // Asegurar terminación en '\0'
            if (len >= (int)sizeof(buffer)) {
                len = sizeof(buffer) - 1;
            }
            buffer[len] = '\0';

            // Sanitizar: cortar en \r o \n si vienen de ESP32 con println
            for (int i = 0; i < len; i++) {
                if (buffer[i] == '\r' || buffer[i] == '\n') {
                    buffer[i] = '\0';
                    len = i;
                    break;
                }
            }

            count++;
            
            // Mostrar datos como cadena directa
            printf("[RAW ] %s\n", (char*)buffer);

            // Mostrar datos ASCII (solo imprimibles)
            printf("[DATO] ");
            for (int i = 0; i < len; i++) {
                if (buffer[i] >= 32 && buffer[i] <= 126) {
                    printf("%c", buffer[i]);
                } else {
                    printf(".");
                }
            }
            printf("\n");

            // Mostrar datos HEX para depuración
            printf("[HEX ] ");
            for (int i = 0; i < len; i++) {
                printf("%02X ", buffer[i]);
            }
            printf("\n");

            // ==================== PARSEAR Y VERIFICAR GEOFENCE ====================
            char id[32] = {0};
            double lat = 0.0, lon = 0.0;
            
            // Formato esperado: "VACA_01,6.250000,-75.560000"
            // %31[^,] para evitar overflow en id
            if (sscanf((char*)buffer, "%31[^,],%lf,%lf", id, &lat, &lon) == 3) {
                printf("[PARSE] ID='%s' LAT=%.6f LON=%.6f\n", id, lat, lon);
                verificarGeofence(id, lat, lon);
            } else {
                printf("[ERROR] Formato inválido\n");
                printf("[INFO] Esperado: ID,LATITUD,LONGITUD\n");
                printf("[INFO] Ejemplo: VACA_01,6.250000,-75.560000\n\n");
            }
            
            printf("[TOTAL] %d paquetes procesados\n\n", count);
        }
        
        static int status = 0;
        if (++status >= 10) {
            printf("[STATUS] Escuchando... (%d recibidos)\n", count);
            status = 0;
        }
    }
    
    printf("\n[INFO] Cerrando...\n");
    lora_set_mode(MODE_SLEEP);
    close(spi_fd);
    printf("[OK] Gateway cerrado\n");
    
    return 0;
}
