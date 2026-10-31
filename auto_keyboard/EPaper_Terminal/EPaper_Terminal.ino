#include <SPI.h>
#include "epd4in2_V2.h"
#include "epdpaint.h"

#define COLORED     0
#define UNCOLORED   1

unsigned char image[1200];
Paint paint(image, 400, 24);
Epd epd;

String inputString = "";

void setup() {
  Serial.begin(115200);

  if (epd.Init() != 0) {
    Serial.println("e-Paper initialization failed.");
    return;
  }

  // Clear to pure white on startup and stay awake
  epd.Clear();
}

void loop() {
  bool screenNeedsUpdate = false;
  
  // As soon as data arrives, read it immediately
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Backspace
    if (inChar == '\b' || inChar == 127) {
      if (inputString.length() > 0) {
        inputString.remove(inputString.length() - 1);
        screenNeedsUpdate = true;
      }
    }
    // Enter / Newline clears the screen for a new sentence
    else if (inChar == '\n' || inChar == '\r') {
      inputString = "";
      screenNeedsUpdate = true;
    }
    // Any standard typed character
    else if (inChar >= 32 && inChar <= 126) {
      inputString += inChar;
      screenNeedsUpdate = true;
    }
    
    // A tiny 2ms delay groups characters if they arrive perfectly simultaneously
    delay(2); 
  }

  // Fire the screen update instantly
  if (screenNeedsUpdate) {
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 0, inputString.c_str(), &Font24, COLORED);
    epd.Display_Partial(paint.GetImage(), 0, 138, paint.GetWidth(), 138 + paint.GetHeight());
  }
}
