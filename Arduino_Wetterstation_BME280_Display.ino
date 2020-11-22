#include <Wire.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); //I2C Displayadresse 0x27 -> 20 chars und 4 Zeilen Display
Adafruit_BME280 bme;

//Globale Variablen
int counter = 0;
int _LoopDelay = 1000;
int DisplayCounter = 0; //Umschalten zwischen Aktuellen Werten und Min/Max Werten

float LastTemp = 0;
float LastPressure = 0;
float LastHumidity = 0;
float LastWasserdampfgehalt = 0;

float MinTemp = 1000;
float MinPressure = 10000;
float MinHumidity = 1000;
float MinWasserdampfgehalt = 1000;

float MaxTemp = -1000;
float MaxPressure = -10000;
float MaxHumidity = -1000;
float MaxWasserdampfgehalt = -1000;

//Prototyp
void BlinkBacklight(int count = 3);

void setup()
{
  bool status = false;
  pinMode(LED_BUILTIN, OUTPUT); //integrierte LED initialisieren
  Serial.begin(9600);

  status = bme.begin(0x76); //Adresse I2C Temp, Feuchte, Druck Sensor

  if (!status)
  {

    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
    }
    //while (1); //Zwangsstopp --> Reset durchführen
  }

  lcd.begin(); //LCD Initalisieren
  lcd.backlight();  //Hintergrundbeleuchtung einschalten
}
void loop()
{
  // lcd.print(0x30+val/100,BYTE);
  // lcd.print(0x30+(val0)/10,BYTE);
  // lcd.print(0x30+val,BYTE);
  ReadSensorData();
  CalcWaterValue();
  CalcMinMaxValues();
  SerialLoopAusgabe();

  if (DisplayCounter == 30)
  {
    lcd.clear();
    PrintMinMaxValuesOnLCD();
    DisplayCounter = 0;
    delay(5000);
    lcd.clear();
  }
  else
  {
    PrintLastValuesOnLCD();
    if (CheckForAlarm()) //Alarmwerte abfragen z.B Feuchte >= 60%rF
      BlinkBacklight(1);
    delay(_LoopDelay);
  }

  DisplayCounter++;
}

void PrintLastValuesOnLCD()
{
  //Temp, Feuchte, Druck Sensor (Bosch BME280)
  lcd.setCursor(0, 0);
  lcd.print(LastTemp);  //°C
  lcd.setCursor(17, 0);
  lcd.print("\337C");

  lcd.setCursor(0, 1);
  lcd.print(LastHumidity); //%rF
  lcd.setCursor(17, 1);
  lcd.print("%rF");

  lcd.setCursor(0, 2);
  lcd.print(LastPressure); //hPa
  lcd.setCursor(17, 2);
  lcd.print("hPa");

  lcd.setCursor(0, 3);
  lcd.print(LastWasserdampfgehalt); //g/m^3
  lcd.setCursor(16, 3);
  lcd.print("g/m3");
}

void PrintMinMaxValuesOnLCD()
{
  //Temp, Feuchte, Druck Sensor (Bosch BME280)
  lcd.setCursor(0, 0);
  lcd.print("Min");
  lcd.print(MinTemp);  //°C
  lcd.setCursor(8, 0);
  lcd.print("Max:");
  lcd.print(MaxTemp);  //°C
  lcd.setCursor(17, 0);
  lcd.print("\337C");//°C

  lcd.setCursor(0, 1);
  lcd.print("Min");
  lcd.print(MinHumidity);  //%rF
  lcd.setCursor(8, 1);
  lcd.print("Max:");
  lcd.print(MaxHumidity);  //%rF
  lcd.setCursor(17, 1);
  lcd.print("%rF");

  lcd.setCursor(0, 2);
  lcd.print("Min");
  lcd.print((int)MinPressure);  //hPa
  lcd.setCursor(8, 2);
  lcd.print("Max:");
  lcd.print((int)MaxPressure);  //hPa
  lcd.setCursor(17, 2);
  lcd.print("hPa");

  lcd.setCursor(0, 3);
  lcd.print("Min");
  lcd.print(MinWasserdampfgehalt);  //g/m³
  lcd.setCursor(8, 3);
  lcd.print("Max:");
  lcd.print(MaxWasserdampfgehalt);  //g/m³
  lcd.setCursor(17, 3);
  lcd.print("g/m");
}

void BlinkBacklight(int count)
{
  lcd.noBacklight(); //Hintergrundbeleuchtung ausschalten
  delay(_LoopDelay / 2);
  lcd.backlight();  //Hintergrundbeleuchtung einschalten
}

bool CheckForAlarm()
{
  if (LastHumidity >= 60)
    return true;
  else
    return false;
}

void ResetMinMax24h()
{
  
}

void ReadSensorData()
{
  //Temp, Feuchte, Druck Sensor (Bosch BME280)
  LastTemp = bme.readTemperature();
  LastPressure = bme.readPressure() / 100.0f;
  LastHumidity = bme.readHumidity();
}

void CalcMinMaxValues()
{
  CalcMinValues();
  CalcMaxValues();
}

void CalcMinValues()
{
  if (MinTemp >= LastTemp)
    MinTemp = LastTemp;
  if (MinPressure >= LastPressure  )
    MinPressure = LastPressure;
  if (MinHumidity >= LastHumidity )
    MinHumidity = LastHumidity;
  if (MinWasserdampfgehalt >= LastWasserdampfgehalt)
    MinWasserdampfgehalt = LastWasserdampfgehalt;
}

void CalcMaxValues()
{
  if (MaxTemp <= LastTemp)
    MaxTemp = LastTemp;
  if (MaxPressure <= LastPressure)
    MaxPressure = LastPressure;
  if (MaxHumidity <= LastHumidity )
    MaxHumidity = LastHumidity;
  if (MaxWasserdampfgehalt <= LastWasserdampfgehalt)
    MaxWasserdampfgehalt = LastWasserdampfgehalt;
}

void CalcWaterValue()
{
  float Saettingsungsdampfdruck = 0;
  float TatsaechlicherDampfdruck = 0;
  float MaxWasserdampfgehalt = 0;
  float TatsaechlicherWasserdampfgehalt = 0;

  Saettingsungsdampfdruck = 6.112 * exp((17.62 * LastTemp) / (243.12 + LastTemp));
  TatsaechlicherDampfdruck = Saettingsungsdampfdruck / 100 * LastHumidity;
  MaxWasserdampfgehalt = (Saettingsungsdampfdruck / (461.5 * (273.15 + LastTemp))) * 100000;
  TatsaechlicherWasserdampfgehalt = (TatsaechlicherDampfdruck / (461.5 * (273.15 + LastTemp))) * 100000;

  LastWasserdampfgehalt = TatsaechlicherWasserdampfgehalt;
}

void SerialLoopAusgabe()
{
  //Loop Ausgabe
  String ausgabe = "Loop: ";
  Serial.println(ausgabe + counter);
  Serial.println(LastTemp);
  Serial.println(MinTemp);
  Serial.println(MaxTemp);

  Serial.println(LastPressure);
  Serial.println(MinPressure);
  Serial.println(MaxPressure);

  Serial.println(LastHumidity);
  Serial.println(MinHumidity);
  Serial.println(MaxHumidity);
  counter++;
}
