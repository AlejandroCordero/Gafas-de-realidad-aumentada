#include <TFT_eSPI.h>
#include <SPI.h>
#include "EscudoUMA.h"
#include <BLEDevice.h>
#include <Wire.h>

#define MAXBUFF 10
#define SCREEN_WIDTH 240 // ANCHO OLED,  en pixels
#define SCREEN_HEIGHT 135 // ALTURA OLED, en pixels
#define RX 12         // RX pin
#define TX 13         // TX pin
HardwareSerial RVoz(1);

// Crea un objeto de tipo display TFT con los pines de User_Setup.h
TFT_eSPI tft = TFT_eSPI();

//String serverUUID = "FC:58:FA:88:56:59";
// Servicio al que se desea conectar
static BLEUUID serviceUUID("fff0");
// La característica buscada del servicio anterior
static BLEUUID    charUUID("fff4");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

byte unsigned XORv[20] = {0x41, 0x21, 0x73, 0x55, 0xA2, 0xC1, 0x32,
                          0x71, 0x66, 0xAA, 0x3B, 0xD0, 0xE2, 0xA8,
                          0x33, 0x14, 0x20, 0x21, 0xAA, 0xBB
                         };                                               // Array para descifrar dato recibido
const byte maskVDC = 0x60;                                                // Máscara detección lectura VDC
const byte maskVAC = 0xA0;                                                // Máscara detección lectura VAC
enum Lectura {Auto, C, Fahr, VDC, VAC, nF, uF, kOhm, MOhm, Ohm, DIODO};   // Máquina de estados
Lectura EstadoActual;                                                     //
Lectura EstadoAnterior;                                                   // Variables de máquina de estados
const byte num[10] = {0b10111110, 0b10100000, 0b11011010, 0b11111000, 0b11100100,
                      0b01111100, 0b01111110, 0b10101000, 0b11111110, 0b11111100
                     };                                                   // Array para identificar dígito
byte buffPalabra[MAXBUFF] = {NULL};                                       // Array dato descifrado
uint8_t Comando[5][10] = {
  {0xea, 0xec, 0x70, 0xed, 0xa2, 0xc1, 0x32, 0x71, 0x64, 0x99},           // AUTO
  {0xea, 0xec, 0x70, 0xe3, 0xa2, 0xc1, 0x32, 0x71, 0x64, 0x9b},           // Celsius
  {0xea, 0xec, 0x70, 0xe2, 0xa2, 0xc1, 0x32, 0x71, 0x64, 0x98},           // Fahr
  {0xea, 0xec, 0x70, 0xe5, 0xa2, 0xc1, 0x32, 0x71, 0x64, 0x81},           // Capacidad
  {0xea, 0xec, 0x70, 0xe4, 0xa2, 0xc1, 0x32, 0x71, 0x64, 0x86}            // Diodo
};                                                                        // Array comandos cambiar modo
byte Voz = 0x00;                                                          //
byte VozAnterior = 0x00;                                                  // Variables control por voz
char IniciarRVoz[2] = {0x21, 0xAA};                                       // Valores binarios que inicializan el set de comandos de voz.

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  for (int i = 0; i < length; i++) {
    buffPalabra[i] = *(pData + i);
  }

  ConvierteAPalabra(buffPalabra, length);
  tft.fillScreen(TFT_BLUE);
  SeleccionaDigitos(buffPalabra);
  SeleccionaTipoLectura(buffPalabra, EstadoActual);
  if (EstadoAnterior != EstadoActual) {
    EstadoAnterior = EstadoActual;
  }
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      tft.fillScreen(TFT_BLUE);
      tft.setCursor(0, SCREEN_HEIGHT / 2 - 30);
      tft.setTextFont(4);
      tft.setTextSize(2);
      tft.setTextColor(TFT_GOLD);
      tft.print("Scanning");
    }
};

bool connectToServer() {

  BLEClient*  pClient  = BLEDevice::createClient();

  pClient->setClientCallbacks(new MyClientCallback());

  // Conectar con el servidor BLE remoto.
  pClient->connect(myDevice);
  pClient->setMTU(517); //Solicita el MTU al servidor

  // Obtiene una referencia del servicio que buscamos
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Obtiene una referencia de la característica que buscamos
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Lee el valor de la característica.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}
/**
   Escanea servidores BLE y busca uno que ofrezca el servicio deseado
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Se llama por cada servidor mostrado
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {

      // Se comprueba si ese servidor ofrece el servicio que buscamos
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;

      } // Encontramos nuestro servidor
    }
};

void setup() {
  RVoz.begin(38400, SERIAL_8N1, TX, RX);   // baud, protocolo, txpin, rxpin
  Serial.begin(115200);
  BLEDevice::init("");

  // Se comienza el escaneo
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  EstadoActual = Auto;
  EstadoAnterior = Auto;

  // Inicializa la pantalla
  tft.init();
  // Rotation = 1 simboliza rota la lectura de la imagen
  tft.setRotation(1);
  // La pantalla llena con el color azul
  tft.fillScreen(TFT_WHITE);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 145, 135, EscudoUMA);
  tft.setCursor(146, SCREEN_HEIGHT / 2 - 50);
  tft.setTextFont(4);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.println("HUD"); //
  tft.setCursor(146, SCREEN_HEIGHT / 2 - 20);
  tft.println("Glass");
  tft.setCursor(146, SCREEN_HEIGHT / 2 + 10);
  tft.println("2022");
  delay(2000);                 // Muestra el texto durante un tiempo
  RVoz.write(IniciarRVoz, 2);
  RVoz.write(IniciarRVoz, 2);
  RVoz.write(IniciarRVoz, 2);
  RVoz.flush();
  delay(5);
  tft.fillScreen(TFT_BLUE);
  tft.setCursor(0, SCREEN_HEIGHT / 2 - 30);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GOLD);
  tft.print("Scanning");
} // Fin del setup


void loop() {
  // Si la flag "doConnect" se ha realizado de manera correcta la búsqueda y conexión al servicio
  if (doConnect == true) {
    if (connectToServer()) {
    } else {
    }
    doConnect = false;
  }

  // Si el dispositivo está conectado, permite enviar datos al multímetro
  if (connected) {
    if (RVoz.available() > 0) {
      CambiarTipoLectura();
    }
  } else {
    BLEDevice::getScan()->start(0);  // Tras la desconexión comienza a bustar otra vez las características
  }
  delay(100);
} // Fin del loop


void SeleccionaDigitos(byte *ptrPalabra) {  // Algoritmo para pintar digitos
  tft.setTextFont(7);
  tft.setTextSize(1);
  tft.setCursor(10, SCREEN_HEIGHT / 2 - 50);

  int i = 3;                      // Primer 7 segmentos tiene primer bit en posición 3
  int j = 4;
  int k = 0;
  int l = 0;
  bool siguiente = true;          // Controla si se pasa al siguiente dígito
  byte Digitos[4];                // Array que almacena el dígito a dibujar
  byte DigitosMsk[4];             // Máscara del dígito a dibujar

  while ((i < 8) && (l < 4)) {                 // 'i' recorre todo el array del código 7 segmentos recibido.
    siguiente = false;
    k = 0;
    while (!(siguiente == true && j == 4)) {
      // Se lee el valor de ptrPalabra para la posición j, y se escribe su valor en Digitos[l]
      bitWrite(Digitos[l], k, bitRead(*(ptrPalabra + i), j));
      k++;
      j++;
      if (j == 8) {
        j = 0;
        i++;
        siguiente = true;
      }
    }
    DigitosMsk[l] = Digitos[l];         // Para comparar con "num[]"
    l++;
  }
  i = 0;
  j = 0;
  bool dibujado = false;
  // Comprueba los 4 dígitos para dibujarlos
  while (j < 4) {
    DigitosMsk[j] &= 0b11111110;        // Preparación del comparador

    while ((i < 10) && (!dibujado)) {

      // Repasa si el 7 segmentos coincide con algún num[i]
      if (DigitosMsk[j] == num[i]) {
        dibujado = true;

        // Si coincide, se comprueba si el msb es un 0, o 1 ('.' o '-')
        if (bitRead(Digitos[j], 0) == 1) {
          if (j == 0) {
            tft.print("-");
          } else {
            tft.print(".");
          }
        }
        tft.print(i);                 // "i" corresponde con la posición del número al que representa en el array num[i].
      }
      i++;
    }
    i = 0;
    dibujado = false;
    j++;
  }
}

void SeleccionaTipoLectura(byte* ptrPalabra, Lectura & Estado) {    // Selecciona el modo de lectura
  tft.setTextFont(4);
  tft.setTextSize(2);
  tft.setCursor(10, SCREEN_HEIGHT / 2);
  if (*(ptrPalabra + 9) == 0x88) {
    tft.print("C");
    Estado = C;
  } else if (*(ptrPalabra + 9) == 0x84) {
    tft.print("FAHR");
    Estado = Fahr;
  } else if (*(ptrPalabra + 9) == 0x80) {
    if (*(ptrPalabra + 8) == 0x00) {
      if ((*(ptrPalabra + 7)&maskVDC) == maskVDC) {
        tft.print("V DC");
        Estado = VDC;
      } else if ((*(ptrPalabra + 7)&maskVAC) == maskVAC) {
        tft.print("V AC TRMS");
        Estado = VAC;
      }
      if (*(ptrPalabra + 7) == 0x07) {
        tft.fillScreen(TFT_BLUE);
        Estado = Auto;
      }
    } if (*(ptrPalabra + 8) == 0x02) {
      tft.print("V DIOD");
      Estado = DIODO;

    }
    if (*(ptrPalabra + 8) == 0x01) {
      tft.print("nF");
      Estado = nF;
    }
    if (*(ptrPalabra + 8) == 0x09) {
      tft.print("uF");
      Estado = uF;
    }
    if (*(ptrPalabra + 8) == 0x10) {
      tft.print("Ohm");
      Estado = Ohm;
    }
    if (*(ptrPalabra + 8) == 0x30) {
      tft.print("kOhm");
      Estado = kOhm;
    }
    if ((*(ptrPalabra + 8) == 0x90) && (*(ptrPalabra + 7) != 0x07)) {
      tft.print("MOhm");
      Estado = MOhm;
    }
  } else {
    tft.fillScreen(TFT_BLUE);
  }
}

void ConvierteAPalabra(byte* Dato, size_t length) {        // Descifra el dato recibido
  int i = 0;
  while (i < length) {
    *(Dato + i) ^= XORv[i];
    i++;
  }
}

void CambiarTipoLectura() {                               // Controla el tipo de lectura mediante voz
  Voz = RVoz.read();
  if ((Voz != VozAnterior) && (Voz > 0x10) && (Voz < 0x16)) {
    switch (Voz) {
      case 0x11:
        pRemoteCharacteristic->writeValue(Comando[0], sizeof(Comando[0]));
        break;

      case 0x12:
        pRemoteCharacteristic->writeValue(Comando[1], sizeof(Comando[1]));
        break;

      case 0x13:
        pRemoteCharacteristic->writeValue(Comando[2], sizeof(Comando[2]));
        break;

      case 0x14:
        pRemoteCharacteristic->writeValue(Comando[3], sizeof(Comando[3]));
        break;

      case 0x15:
        pRemoteCharacteristic->writeValue(Comando[4], sizeof(Comando[4]));
        break;

      default:
        break;
    }
    VozAnterior = Voz;
  }
}
