#include <SPI.h>
#include "epd4in2_V2.h"
#include "epdpaint.h"

#define COLORED     0
#define UNCOLORED   1

// Create a buffer for a single line of text (400px wide, 24px high)
unsigned char image[1200];
Paint paint(image, 400, 24);
Epd epd;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);

  if (epd.Init() != 0) {
    Serial.println("e-Paper initialization failed.");
    return;
  }

  // Clear the display to pure white on startup
  epd.Clear();
  Serial.println("Screen cleared. Type a message and press Enter...");
  
  // Note: We deliberately do NOT put the display to sleep here 
  // so it retains its memory state for clean partial updates.
}

void loop() {
  // Read Serial input character by character
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Check for newline (Enter key)
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }

  // When Enter is pressed, update the e-Paper
  if (stringComplete) {
    Serial.print("Updating display with: ");
    Serial.println(inputString);

    // 1. Wipe our local 24px memory buffer clean
    paint.Clear(UNCOLORED);
    
    // 2. Draw the new text into the buffer
    paint.DrawStringAt(0, 0, inputString.c_str(), &Font24, COLORED);
    
    // 3. Push the buffer to the display. 
    // Because it stayed awake, it automatically erases the old text!
    epd.Display_Partial(paint.GetImage(), 0, 138, paint.GetWidth(), 138 + paint.GetHeight());

    // Reset the string for the next message
    inputString = "";
    stringComplete = false;
    Serial.println("Update complete. Ready for next message.");
  }
}