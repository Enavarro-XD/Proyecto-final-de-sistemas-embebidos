#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ==================== CONFIGURACIÓN DE PINES ====================
// ESP32 DevKit V4 -> RA-01 (433 MHz)
#define SS_PIN    5    // Chip Select
#define RST_PIN   14   // Reset
#define DIO0_PIN  26   // Interrupción

// ESP32 DevKit V4 -> GPS NEO-6M
#define RX_PIN    16   // Conectar al TX del GPS
#define TX_PIN    17   // Conectar al RX del GPS
#define GPS_BAUD  9600

// ==================== CONFIGURACIÓN LORA (Igual al Gateway) ====================
#define BAND      433E6    // 433 MHz
#define ID_ANIMAL "VACA_01" // ID que espera el Gateway

// Objetos globales
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART 2

unsigned long lastSendTime = 0;
const int interval = 10000; // Enviar cada 10 segundos

void setup() {
  // 1. Iniciar Monitor Serial (USB)
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("\n=== INICIANDO COLLAR DE GANADO ===");

  // 2. Iniciar GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("[INFO] GPS iniciado en pines 16(RX) y 17(TX)");

  // 3. Iniciar LoRa
  Serial.println("[INFO] Iniciando LoRa...");
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(BAND)) {
    Serial.println("[ERROR] No se detecta el módulo LoRa.");
    Serial.println("   Revisa: VCC=3.3V, GND, MOSI=23, MISO=19, SCK=18, CS=5");
    while (1);
  }

  // --- CONFIGURACIÓN ESPEJO DEL CÓDIGO C (GATEWAY) ---
  // IMPORTANTE: No cambiar nada aquí o perderás conexión
  
  LoRa.setSpreadingFactor(12);       // SF12 (Igual que C)
  LoRa.setSignalBandwidth(62.5E3);   // 62.5 kHz (Igual que C)
  LoRa.setCodingRate4(8);            // 4/8 (Igual que C)
  LoRa.setSyncWord(0x12);            // 0x12 (Igual que C)
  LoRa.disableCrc();                 // CRC OFF (Vital: Tu código C tiene CRC OFF)
  
  // La librería LoRa activa LowDataRateOptimize (LDRO) automáticamente
  // cuando detecta SF11/12 y BW bajo, así que coincide con tu C.

  Serial.println("[OK] LoRa Configurado: SF12 | BW 62.5k | CR 4/8 | CRC OFF");
}

void loop() {
  // 1. Alimentar la librería GPS siempre que haya datos
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Temporizador para enviar datos
  if (millis() - lastSendTime > interval) {
    
    double lat = 0.0;
    double lon = 0.0;
    bool gpsValido = false;

    // Verificar si tenemos señal GPS válida
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      gpsValido = true;
      Serial.print("[GPS] Ubicación real: ");
    } else {
      // SI NO HAY SEÑAL GPS (ej. dentro de casa):
      // Enviamos coordenadas "falsas" o 0.0 para probar que el Gateway recibe.
      // Así puedes probar la conexión LoRa sin salir a la calle.
      lat = 6.250000;   // Coordenada de prueba
      lon = -75.560000; // Coordenada de prueba
      Serial.print("[GPS] Sin señal (Usando prueba): ");
    }
    
    Serial.print(lat, 6); Serial.print(", "); Serial.println(lon, 6);

    // Enviar el paquete
    enviarPaquete(lat, lon);
    
    lastSendTime = millis();
  }
}

void enviarPaquete(double latitud, double longitud) {
  // Construir mensaje: "ID,LAT,LON"
  // Ejemplo: "VACA_01,6.251234,-75.561234"
  String mensaje = String(ID_ANIMAL) + "," + String(latitud, 6) + "," + String(longitud, 6);

  Serial.print("[LORA] Enviando: ");
  Serial.println(mensaje);

  // Transmisión
  LoRa.beginPacket();
  LoRa.print(mensaje);
  LoRa.endPacket();
  
  Serial.println("[LORA] Enviado.");
}