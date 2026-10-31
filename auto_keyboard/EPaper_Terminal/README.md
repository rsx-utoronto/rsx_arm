# e-Paper Live Typing Terminal

This code contains the Arduino sketch for a live-typing serial terminal using an Arduino UNO and a Waveshare 4.2-inch e-Paper V2 display. 

Because the standard Arduino IDE Serial Monitor buffers text and only transmits data after pressing "Enter", achieving true character-by-character live typing requires a terminal emulator that sends raw USB serial data instantly on keydown. This guide explains how to upload the code and configure PuTTY on Windows to interface with the display.

## Prerequisites

* **Hardware:** Arduino UNO connected via USB, wired to a Waveshare 4.2-inch e-Paper V2.
* **Operating System:** Windows 10 / Windows 11.
* **Terminal Emulator:** [PuTTY](https://www.putty.org/) (Download the 64-bit x86 installer or standalone `putty.exe`).

## 1. Upload the Firmware
1. Open `EPaper_Terminal.ino` in the Arduino IDE. 
2. Ensure you have the `epd4in2_V2.h`/`.cpp` files (and the other dependencies) in the same sketch folder.
3. Go to **Tools > Board** and select **Arduino Uno**.
4. Go to **Tools > Port** and select your Arduino's COM port (e.g., `COM3`). Note this number down.
5. Click the **Upload** button to flash the code to the Arduino. 
6. **Crucial:** Once the upload is complete, ensure the Arduino IDE Serial Monitor is **CLOSED**. Only one program can control a serial port at a time; PuTTY will fail to connect if the IDE is still monitoring the port.

## 2. Configure PuTTY for Serial Communication
1. Launch PuTTY.
2. In the main configuration window, locate the **Connection type** radio buttons and select **Serial**.
3. In the **Serial line** field, enter your exact Arduino port from Step 1 (e.g., `COM3`).
4. In the **Speed** field, enter `115200` to match the `Serial.begin(115200)` baud rate in the firmware.

## 3. Start Live Typing
1. Click the **Open** button at the bottom of the PuTTY window.
2. A blank black terminal interface will appear.
3. Click inside the terminal window to ensure it has keyboard focus and begin typing.
4. Standard alphanumeric keys, Backspace, and Enter are transmitted instantly to the UNO and rendered sequentially on the e-Paper display.
