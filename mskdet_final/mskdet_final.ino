#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define blue 9
#define pink 10
#define yellow 11
#define orange 12

#define HALFSTEP 8
#define FULLSTEP 4

AccelStepper stepper1(HALFSTEP, blue, yellow, pink, orange);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

int ledPin = 4; // Define the pin for the LED

void setup()
{
    Serial.begin(9600);      // Initialize serial communication
    pinMode(ledPin, OUTPUT); // Set the LED pin as an output
    lcd.init();              // Initialize the LCD
    lcd.backlight();         // Turn on the backlight
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(100.0);
}

void loop()
{
    if (Serial.available() > 0)
    {
        // Read the input from serial
        String input = Serial.readStringUntil('\n');

        // Extract X coordinate and mask detection status
        int commaIndex = input.indexOf(',');
        int x = input.substring(0, commaIndex).toInt();
        String maskStatus = input.substring(commaIndex + 1);

        // Convert X coordinate to angle
        int angle = map(x, 0, 1200, -180, 180); // Assuming X coordinate ranges from 0 to 1200 pixels and mapping it to -180 to 180 degrees

        // Move the stepper motor
        stepper1.moveTo(angle);

        // Display mask detection status on LCD and control LED
        if (maskStatus == "M")
        {                               // Mask detected
            lcd.clear();                // Clear the LCD
            lcd.setCursor(0, 0);        // Set cursor to first line
            lcd.print("Mask Detected"); // Print message on LCD
            digitalWrite(ledPin, LOW);  // Turn off the LED
        }
        else if (maskStatus == "N")
        {                                  // No mask detected
            lcd.clear();                   // Clear the LCD
            lcd.setCursor(0, 0);           // Set cursor to first line
            lcd.print("No Mask Detected"); // Print message on LCD
            digitalWrite(ledPin, HIGH);    // Turn on the LED
        }
    }

    stepper1.run();
}