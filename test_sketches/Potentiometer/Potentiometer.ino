#define ADCPIN A0

int adcValue;
float voltValue;
void setup()
{  
  Serial.begin(115200);
}
void loop()
{
  adcValue = analogRead(ADCPIN);
  voltValue = ((adcValue * 3.3) / 4095);
  Serial.print("ADC Value = ");
  Serial.print(adcValue);
  //delay(1000);
  Serial.print("  ");
  Serial.print("Voltage = ");
  Serial.print(voltValue);
  Serial.println(" V");
}