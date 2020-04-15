/*
 AmboVent-UVG
 funciones_LCD.ino
 Para verificar las funciones básicas del LCD
 Based on the original code for the AmboVent (April 12, 2020)
*/

#include <Wire.h>    // Used for I2C
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display
int cont = 0;

void setup()
{
  lcd.begin();      // initialize the LCD
  lcd.backlight();  // Turn on the blacklight and print a message.
  lcd.setCursor(0, 0);  lcd.print("AmboVent-UVG   ");
  lcd.setCursor(0, 1);  lcd.print("1690.108       ");
}

void loop()
{
  display_text_2_lines("Prueba LCD", "Esperando...");
  delay(1000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Contador: ");
  lcd.setCursor(0, 1);
  lcd.print(cont);
  lcd.print(" ua");
  delay(1000);

  cont = (cont + 1)%1024;
}

void display_text_2_lines(char *message1, char *message2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message1);
  lcd.setCursor(0, 1);
  lcd.print(message2);
}
