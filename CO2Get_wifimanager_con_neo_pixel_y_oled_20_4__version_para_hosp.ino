#include <ESPEssentials.h>
#include <OTA.h>
#include <WebServer.h>
#include <Wifi.h>

// upDated: 06/01/21
//
// Proyecto para medición de CO2 ambiental:
// WiFi
// MLX
// On/Off con GPIO al MLX
// Con ruta de envío de datos de temperatura a server
// String url = "http://45.55.129.9/apiesp/?data=" + String(variable);
// http://demoadox.com/ambientecontroladoco2/services/Services.php?acc=AD&id=xx&co2=xxxx&temp=xx.x&bateria=xxxx&wifi=xxx



// Con medición de CO2 conectando por puerto serial al sensor mh-z19c



// con A/D para los 3,3 de la "batería"

// Queda pendiente sumar el tema del OTA + HTTPS como
// está en el post que tengo anotado.

// Se suma la el RSSI (potencia de señal de wifi) como 4to parámetro

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include "MHZ19.h"                                        
#include <SoftwareSerial.h>  

//AGREGADO OLED Y NEOPIXEL
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET LED_BUILTIN  //4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };


  // Which pin on the ESP8266 is connected to the NeoPixels?
#define PIN            0

// How many NeoPixels are attached to the ESP8266?
#define NUMPIXELS      8

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second
int cont_color_verde = 200;
int flag = 1;

int cont_amarillo = 0;
int cont_display = 0;
int pantalla = 0;
int estado_CO2 = 0;

//FIN AGREGADO OLED Y NEOPIXEL



#define RX_PIN 13                                          // D7 - Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 15                                          // D8 - Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

#define LED_D6  12

//#define SENSOR_ID String(ESP.getChipId())  //Esto lo vamos hardcodeando por ahora sensor a sensor así lo podemos identificar en el software de ambiente controlado.
const String SensorID = String(ESP.getChipId(),HEX);


// WiFi credentials.
//const char *WIFI_SSID = "ADOXWIFI"; //"Fibertel WiFi663"; //"Desarrollos"; //"Moto 9586";
//const char *WIFI_PASS = "adoxwifi"; //"0043185996";     //"adox1225";//"titi2011"; 

HTTPClient http;
WiFiClient client;

const int RSSI_MAX = -50;  // define maximum strength of signal in dBm
const int RSSI_MIN = -100; // define minimum strength of signal in dBm

uint32_t PAUSA = 6;   // 5 min o 6 seg por ahora;


MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial


int CO2;
int get_dev = 0; 


ADC_MODE(ADC_VCC); //este modo sirve para habilitar el divisor interno y
			    //y poder medir correctamente el BUS de 3.3v

uint32_t calculateCRC32(const uint8_t *data, size_t length);

// helper function to dump memory contents as hex
void printMemory();


//unsigned long getDataTimer = 0;

long previousMillis = 0;        // will store last time LED was updated
long interval = 30000;           // interval at which to blink (milliseconds)

void connect();

// Structure which will be stored in RTC memory.
// First field is CRC32, which is calculated based on the
// rest of structure contents.
// Any fields can go after CRC32.
// We use byte array as an example.
struct
{
	uint32_t crc32;
	byte data[508];
} rtcData;

//WiFiClientSecure wifiClient;

//Lee una posicion de memoria volatil que se resetea cuando quitamos la bateria, con deepsleep persiste
int leer_memoria(int posicion)
{
	int valor = 0;

	// Read struct from RTC memory
	if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData)))
	{
		uint32_t crcOfData = calculateCRC32((uint8_t *)&rtcData.data[0], sizeof(rtcData.data));
		if (crcOfData != rtcData.crc32)
		{
			//Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
			valor = 0;
		}
		else
		{
			//Serial.println("CRC32 check ok, data is probably valid.");
			valor = rtcData.data[posicion];
		}
	}
	return valor;
}

//Escribe una posicion de memoria volatil que se resetea cuando quitamos la bateria, con deepsleep persiste
void escribir_memoria(int posicion, int valor)
{
	rtcData.data[posicion] = valor;

	// Update CRC32 of data
	rtcData.crc32 = calculateCRC32((uint8_t *)&rtcData.data[0], sizeof(rtcData.data));
	// Write struct to RTC memory
	if (ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData)))
	{
		Serial.println("Escritura ok");
	}
}

void setup()
{

//AGREGADO OLED Y NEOPIXEL
 Serial.begin(9600);

    pinMode (LED_D6, OUTPUT);


  // put your setup code here, to run once:
  pixels.begin(); // This initializes the NeoPixel library.


  

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
//  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//    for(;;); // Don't proceed, loop forever
//  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
//  display.display();

 
// display.clearDisplay();
  





  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
//  display.clearDisplay();

  // Draw a single pixel in white
  //display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
//  display.display();
  //delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...



//  testdrawstyles();    // Draw 'stylized' characters


delay(2000);

//FIN AGREGADO OLED Y NEOPIXEL

  
  initESPEssentials("Sensor_"+SensorID);

  WebServer.on("/reset_wifi", HTTP_GET, [&]() {
    WebServer.send(200, "text/plain", "Wifi settings reset.");
    Wifi.resetSettings();
  });

  delay(3000);

  


  //	Serial.begin(115200);
  //	Serial.setTimeout(500);

	// Wait for serial to initialize.
  //	while (!Serial)
  //	{
  //	}

  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

  myMHZ19.autoCalibration(true);                              // Turn auto calibration ON (OFF autoCalibration(false))
 
	////////////int a = connect();
  // connect();

}


void loop()
{
  unsigned long currentMillis = millis();
  handleESPEssentials();

  pantalla = 1;
  CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
 //AGREGADO OLED Y NEOPIXEL
 // display.invertDisplay(true);
 // delay(50);
 // display.invertDisplay(false);
  //delay(100);

   // testdrawstyles();
          if (CO2 == 0)
          {
                 

             
                     pixels.setPixelColor(0, pixels.Color(255,0,0)); // Moderately bright green color.
                     pixels.show(); // This sends the updated pixel color to the hardware.  
               
              
          
          
          }
          else 
          {

            //luces de estado de conexion
                    if (WiFi.status()== 0 )
                  {
                       pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
                     
                     pixels.show(); // This sends the updated pixel color to the hardware.  
                    delay (200);
              
              
                      pixels.setPixelColor(0, pixels.Color(0,255,0)); // Moderately bright green color.
                      pixels.show(); // This sends the updated pixel color to the hardware.  

                      delay (200);
             
                   }else if (WiFi.status()== 3&& get_dev == 1)
                    {
                    pixels.setPixelColor(0, pixels.Color(0,255,0)); // Moderately bright green color.
          
                     pixels.show(); // This sends the updated pixel color to the hardware.  
                  }else

                  {
                     pixels.setPixelColor(0, pixels.Color(0,0,255)); // Moderately bright green color.
                     pixels.show(); // This sends the updated pixel color to the hardware.  
                  }
                  
               /*

              //anulando luces en version de hospital
               pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
                     
               pixels.show(); // This sends the updated pixel color to the hardware.  
          */
          }
         

//FIN AGREGADO OLED Y NEOPIXEL


  
  if ((WiFi.status() == WL_CONNECTED) && (currentMillis - previousMillis > interval))
  {
    previousMillis = currentMillis;
    connect();
  }
}


void connect()
{
//	// Connect to Wifi.
//  Serial.println();
//  Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(WIFI_SSID);
//	
//	WiFi.begin(WIFI_SSID, WIFI_PASS);
//
//	WiFi.persistent(false);
//	WiFi.mode(WIFI_OFF);
//	WiFi.mode(WIFI_STA);
//	WiFi.begin(WIFI_SSID, WIFI_PASS);
//
//	unsigned long wifiConnectStart = millis();
//
//	while (WiFi.status() != WL_CONNECTED)
//	{
//		// Check to see if
//		if (WiFi.status() == WL_CONNECT_FAILED)
//		{
//			Serial.println("Failed to connect to WiFi. Please verify credentials: ");
//			delay(10000);
//		}
//
//		delay(500);
//		Serial.println(".");
//		// Only try for 5 seconds.
//		if (millis() - wifiConnectStart > 15000)
//		{
//			Serial.println("Failed to connect to WiFi");
//			return;
//		}
//	}
//
//	Serial.println("WiFi connected");


  Serial.print("[HTTP] begin...\n");
  
  Serial.print("SensorID:\n");
  Serial.print(SensorID + "\n");

	int variable = leer_memoria(1);
 // int CO2; 
  int8_t Temp;

	//Sensores
  // CO2
  // if (millis() - getDataTimer >= 2000)
  // {

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
    if below background CO2 levels or above range (useful to validate sensor). You can use the 
    usual documented command with getCO2(false) */

    

  //CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
    
  Serial.print("CO2 (ppm): ");                      
  Serial.println(CO2);                                

    
  Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
  Serial.print("Temperature (C): ");                  
  Serial.println(Temp);                               

  //   getDataTimer = millis();
  // }
	
	long rssi = WiFi.RSSI();
	Serial.print("RSSI:");
	Serial.println(rssi);

	
	int quality = dBmtoPercentage(rssi);
	Serial.println(quality);


  // Armo string para mandarle a la web
	//String url_post = "http://45.55.129.9/apiesp/post.php";
	//String data_post = String(variable) + " " + " " + String(ESP.getVcc()) + " " + String(quality) + " " + String(CO2); //+ String(rssi)+ " " 

  
  
  //String url = "https://demoadox.com/ambientecontroladoco2/services/Services.php?acc=AD&id=" + String(SENSOR_ID) + "&co2=" + String(CO2) + "&temp=" + String(Temp) + "&bateria=" + String(ESP.getVcc()) + "&wifi=" + String(quality);
  //String url = "http://159.203.150.67/ProAccessFace/services/Services.php?acc=TEMP&TUNO=" + String(mlx.readAmbientTempC()) + "&TDOS=" + String(mlx.readObjectTempC()) + "&ESTADO=OK";
  //String url = "http://159.203.150.67/ProAccessFace/services/Services.php?acc=AD&id=" + String(SENSOR_ID) + "&co2=" + String(CO2) + "&temp=" + String(Temp) + "&bateria=" + String(ESP.getVcc()) + "&wifi=" + String(quality);

  String url = "http://159.203.150.67/calidaddelaireadox/services/Services.php?acc=AD&id=" + SensorID + "&co2=" + String(CO2) + "&temp=" + String(Temp) + "&bateria=" + String(ESP.getVcc()) + "&wifi=" + String(quality);
  

	if (variable != 1)
	{
		//Serial.println("-------------------------------------");
		//Serial.println("Parece que fue un booteo");
		//Serial.println("-------------------------------------");
		escribir_memoria(1, 1);
	}
	else
	{
		//Serial.println("Ciclo normal");
	}

	//Serial.println("URL: " + url_post);
	//Serial.println("DATA: " + data_post);
  Serial.println("URL: " + url);


  ///HASTA ACÄ LLEGUE PROLIJO CON EL TEMA DE PONER EL GET Y NO EL POST. HAY QUE SEGUIR COPIANDO DESDE EL 
  //PROYECTO aug27a !!!!

	if (http.begin(client, url))
	{
	  // HTTP
		//http.addHeader("Content-Type", "text/plain");  //Specify content-type header

    Serial.print("[HTTP] GET...\n");
		// start connection and send HTTP header
		int httpCode = http.GET();

		// httpCode will be negative on error
		if (httpCode > 0)
		{

        get_dev = 1;
		  // HTTP header has been send and Server response header has been handled
			Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      
      //---------------------------------------------------------------------------------------------
			// file found at server 
    
      // ESTO LO DEJO POR SI QUEREMOS MANDAR ALGÚN DATO A LA PLACA NODE DESDE EL SERVER
      //---------------------------------------------------------------------------------------------
      
      //			if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      //			{
      //				String payload = http.getString();
      //				Serial.println(payload);
      //				//este payload trae el tiempo de sleep
      //				//PAUSA = (payload.toInt());
      //			}
      //		}
      //		else
      //		{
      //			Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      //		}
		  //Serial.println("Voy a tirar el primer http end");
		  http.end();
		  //Serial.println("Volví del http end 1");
	  }
	  else
	  {
		  Serial.printf("[HTTP] Unable to connect\n");
      get_dev = 0;
    //	http.end();
	  }
	}
	//Serial.println("Voy a tirar el segundo http end");
	//Serial.println("Volví del http end 2");

	//return;
	//Serial.println("salió del connect?");
}

uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
	uint32_t crc = 0xffffffff;
	while (length--)
	{
		uint8_t c = *data++;
		for (uint32_t i = 0x80; i > 0; i >>= 1)
		{
			bool bit = crc & 0x80000000;
			if (c & i)
			{
				bit = !bit;
			}
			crc <<= 1;
			if (bit)
			{
				crc ^= 0x04c11db7;
			}
		}
	}
	return crc;
}

//prints all rtcData, including the leading crc32
void printMemory()
{
	char buf[3];
	uint8_t *ptr = (uint8_t *)&rtcData;
	for (size_t i = 0; i < sizeof(rtcData); i++)
	{
		sprintf(buf, "%02X", ptr[i]);
		//Serial.print(buf);
		if ((i + 1) % 32 == 0)
		{
			//Serial.println();
		}
		else
		{
			//Serial.print(" ");
		}
	}
}

/*
 * Written by Ahmad Shamshiri
  * with lots of research, this sources was used:
 * https://support.randomsolutions.nl/827069-Best-dBm-Values-for-Wifi 
 * This is approximate percentage calculation of RSSI
 * Wifi Signal Strength Calculation
 * Written Aug 08, 2019 at 21:45 in Ajax, Ontario, Canada
 */

int dBmtoPercentage(long dBm)
{
	int quality;
	if (dBm <= RSSI_MIN)
	{
		quality = 0;
	}
	else if (dBm >= RSSI_MAX)
	{
		quality = 100;
	}
	else
	{
		quality = 2 * (dBm + 100);
	}

	return quality;
} //dBmtoPercentage
