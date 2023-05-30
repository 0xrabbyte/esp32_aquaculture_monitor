#include <Arduino.h>
#include <OneWire.h>
#include <Wire.h>
#define TdsSensorPin A1
#define NTHPin A0
#define WaterDepthPin A2
#define pHPin A3
#define tempPin 10
#define pumpPin 3
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
#define PH_MAX_SAMPLES 256
OneWire ds(tempPin); // DS18B20
#include <Servo.h>

Servo myservo;

const float adc_resolution = 1024.0;
const float ph_offset = -1.5;

float pH_measuring = 0;
float TDS_measuring = 0;
int samples = 0, tds_samples = 0;

struct _aqua_data {
    float TDS_value;
    float pH_value;
    float NTH_value;
    float water_level_value;
    float water_temperature;
    bool is_pump;
} aqua_data;

void trigger_servo() {
    myservo.write(90);
    delay(200);
    myservo.write(0);
    delay(1000);
    myservo.write(90);
    delay(200);
    Serial.println("Servo Triggerd!");
}

void sent_aqua_data() {
    Wire.write((byte *)&aqua_data, (size_t)sizeof(struct _aqua_data));
}

void recv_aqua_data(int len) {
    union myUnion {
      bool data[2];
      char buffer[2];
    } mine;
    int i = 0;
    while(Wire.available())
    {
      mine.buffer[i++] = Wire.read();
    }
    aqua_data.is_pump = mine.data[0];
    Serial.print("Set Pump Status to: ");
    if (aqua_data.is_pump)
        digitalWrite(pumpPin, HIGH);
    else
        digitalWrite(pumpPin, LOW);
    Serial.println(aqua_data.is_pump ? "Running" : "Stopped");
    bool servo_triggered = mine.data[1];
    if (servo_triggered) {
        trigger_servo();
    }
}

void setup(void) {
    Serial.begin(115200);
    Wire.begin(8);
    Wire.onRequest(sent_aqua_data);
    Wire.onReceive(recv_aqua_data);
    pinMode(TdsSensorPin, INPUT);
    pinMode(NTHPin, INPUT);
    pinMode(WaterDepthPin, INPUT);
    pinMode(pHPin, INPUT);
    pinMode(pumpPin, OUTPUT);
    aqua_data.is_pump = false;
    digitalWrite(pumpPin, LOW);
    myservo.attach(9);
}

float ph(float voltage) {
    return 3.5 * voltage + ph_offset;
}

void loop(void) {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    float celsius, fahrenheit;

    if (!ds.search(addr)) {
        // Serial.println("No more addresses.");
        // Serial.println();
        ds.reset_search();
        delay(250);
        return;
    }

    Serial.print("ROM =");
    for (i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    // switch (addr[0])
    // {
    //   case 0x10:
    //     Serial.println("  Chip = DS18S20");  // or old DS1820
    //     type_s = 1;
    //     break;
    //   case 0x28:
    //     Serial.println("  Chip = DS18B20");
    //     type_s = 0;
    //     break;
    //   case 0x22:
    //     Serial.println("  Chip = DS1822");
    //     type_s = 0;
    //     break;
    //   default:
    //     Serial.println("Device is not a DS18x20 family device.");
    //     return;
    // }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    delay(1000); // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    // Serial.print("  Data = ");
    // Serial.print(present, HEX);
    // Serial.print(" ");
    for (i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds.read();
        // Serial.print(data[i], HEX);
        // Serial.print(" ");
    }
    // Serial.print(" CRC=");
    // Serial.print(OneWire::crc8(data, 8), HEX);
    // Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
                            //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    Serial.print("  Temperature = ");
    Serial.print(celsius);
    aqua_data.water_temperature = celsius;
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");

    float voltage = analogRead(TdsSensorPin) * (float)VREF / 1024.0, tdsValue = 0, temperature = celsius;                                                                                           // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);  
    tds_samples++; TDS_measuring += voltage; 
    if (tds_samples > PH_MAX_SAMPLES) {
        TDS_measuring = TDS_measuring / tds_samples * (PH_MAX_SAMPLES / 16);
        tds_samples = PH_MAX_SAMPLES / 16;
    }                                                                                                           // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = (TDS_measuring / tds_samples) / compensationCoefficient / 15;                                                                                                            // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
    // Serial.print("voltage:");
    // Serial.print(averageVoltage,2);
    // Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    aqua_data.TDS_value = tdsValue;
    int NTHValue = analogRead(NTHPin);
    NTHValue = (NTHValue * 100 / 1024);
    Serial.print("turbidity value is ");
    Serial.println(NTHValue);
    aqua_data.NTH_value = NTHValue;
    double depth = analogRead(WaterDepthPin) / 160;

    Serial.print("the depth is:");
    Serial.print(depth);
    Serial.println("cm");
    aqua_data.water_level_value = depth;

    pH_measuring += analogRead(pHPin);
    samples++;

    if (samples > PH_MAX_SAMPLES) {
        pH_measuring /= samples * (PH_MAX_SAMPLES / 16);
        samples = PH_MAX_SAMPLES / 16;
    }

    voltage = pH_measuring * 5 / adc_resolution / samples / 2;
    Serial.print("pH= ");
    Serial.println(ph(voltage));
    aqua_data.pH_value = ph(voltage);
}