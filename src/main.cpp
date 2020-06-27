/* --------------Bibliotecas------------- */
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <ESP32Servo.h>

/* --------------Pinos------------- */
namespace Pins
{
    const uint8_t LDR{36};
    const uint8_t SERVO{23};
}
/* --------------Variáveis ESP-Client------------- */
namespace Wifi
{
    const char* ssid{ "WIFI_TESTE_GIO" };
    const char* password{ "abcd@1234" };
    const uint8_t ip[4] {192, 168, 1, 210};
    const uint8_t geteway[4] {192, 168, 1, 1};
    const uint8_t netmask[4] {255, 255, 255, 0};
}
namespace Mqtt
{
    const uint8_t server[4] {192, 168, 1, 206};
    const uint16_t port {1883};
    const char* user {"sensor"};
    const char* password {"sensor"};
    const char* clientId{"esp32_teste"};

}

WiFiClient espClient;
PubSubClient client{ espClient };
BME280I2C bme{};
Servo servo{};

/*-------------Variáveis Sensores-----------*/
float pressao {0.0};
float umidade {0.0};
float temperatura{0.0};
float luminosidade {0.0};

/*-------------Protótipos-----------*/
void inicializaPerifericos();
void inicializaMqtt();
void inicializaWifi();

void processaEnvio();
void processaMqtt();
void processaRecebimento( char* topic, byte* payload, unsigned int length );

void sensorLuminosidade();
void sensorBME280();
void moveServo( bool acao );

//----------------------------------------------------------------------------------------------------
void setup()
{
    delay( 1000 );
    Serial.begin( 115200 );
    Serial.setDebugOutput( true );
    Serial.printf( "inicio\r\n" );

    inicializaPerifericos();
    inicializaMqtt();
    inicializaWifi();
}
//----------------------------------------------------------------------------------------------------
void loop()
{
    processaMqtt();
    processaEnvio();
}
//----------------------------------------------------------------------------------------------------
void processaMqtt()
{
    const uint32_t agora{ millis() };
    if ( not client.connected() )
    {
        static uint32_t timerReconexao{0};
        if ( agora - timerReconexao > 5000 )
        {
            timerReconexao = agora;

            if ( client.connect( Mqtt::clientId, Mqtt::user, Mqtt::password ) )
            {
                client.subscribe( "action" );
                timerReconexao = 0;
            }
        }
    }
    else
    {
        client.loop();
    }
}
//----------------------------------------------------------------------------------------------------
void processaEnvio()
{
    if( client.connected() )
    {
        static uint32_t timerEnvio{0};
        if( timerEnvio == 0 or millis() - timerEnvio > 10000 )
        {
            timerEnvio = millis();

            sensorBME280();
            sensorLuminosidade();

            char mensagem[128];
            sprintf( mensagem, "%s;%f", Mqtt::clientId, temperatura );
            client.publish( "temperature", mensagem, true );
            Serial.printf( "Enviado [%s = %s]\r\n", "temperature", mensagem );

            sprintf( mensagem, "%s;%f", Mqtt::clientId, luminosidade );
            client.publish( "luminosity", mensagem, true );
            Serial.printf( "Enviado [%s = %s]\r\n", "luminosity", mensagem );
        }
    }
}
//----------------------------------------------------------------------------------------------------
void processaRecebimento( char* topic, byte* payload, unsigned int length )
{
    Serial.printf( "Recebido [%s = %.*s]\r\n", topic, length, reinterpret_cast<const char*>( payload ) );

    if( strcmp( topic, "action" ) == 0 )
    {
        if( length == 8 and strncmp( reinterpret_cast<const char*>( payload ), "activate", length ) == 0 )
        {
            moveServo( true );
        }
        else if( length == 10 and strncmp( reinterpret_cast<const char*>( payload ), "inactivate", length ) == 0 )
        {
            moveServo( false );
        }
    }
}
//----------------------------------------------------------------------------------------------------
void moveServo( bool acao )
{
    const auto rotacao{acao ? 0 : 55};
    servo.write( 180 - constrain( rotacao, 0, 55 ) );
}
//----------------------------------------------------------------------------------------------------
void sensorLuminosidade()
{
    const auto leitura {analogRead( Pins::LDR )}; // Leitura Analogica
    const auto tensao{( leitura / 4095.0 ) * 3.3}; // Calcula a Tensão
    const auto resistencia{tensao * 10000.0 / ( 3.3 - tensao )}; // Calcula resistencia
    const auto lux{pow( 10.0, 6.5 - 1.25 * log10( resistencia ) )}; // Calcula Lux
    luminosidade = lux;
}
//----------------------------------------------------------------------------------------------------
void sensorBME280()
{
    bme.read( pressao, temperatura, umidade, BME280::TempUnit_Celsius, BME280::PresUnit_Pa );
}
//----------------------------------------------------------------------------------------------------
void inicializaWifi()
{
    Serial.printf( "inicializa wifi\r\n" );

    WiFi.mode( WIFI_MODE_STA );

    WiFi.persistent( false );
    WiFi.setAutoConnect( false );
    WiFi.setAutoReconnect( true );

    WiFi.config( Wifi::ip, Wifi::geteway, Wifi::netmask );
    WiFi.begin( Wifi::ssid, Wifi::password );
}
//----------------------------------------------------------------------------------------------------
void inicializaMqtt()
{
    Serial.printf( "inicializa mqtt\r\n" );

    client.setServer( Mqtt::server, Mqtt::port );
    client.setCallback( processaRecebimento );
}
//----------------------------------------------------------------------------------------------------
void inicializaPerifericos()
{
    Serial.printf( "inicializa perifericos\r\n" );

    pinMode( Pins::LDR, INPUT );

    Wire.begin();
    if(!bme.begin()){
        Serial.printf( "erro ao inicializar bme\r\n" );
    }

    pinMode( Pins::SERVO, OUTPUT );
    servo.attach( Pins::SERVO );
    servo.write( 180 );
}
//----------------------------------------------------------------------------------------------------