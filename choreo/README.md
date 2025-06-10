### ELRS

Using the `Jumper Aion ELRS 2.4G TX Nano`:

- USB does not seem to be powerful enough to power the module
- Attach it to TX (tested with RadioMaster Zorro)
- Go into the currently selected model menu and disable the internal RF, enable external CRSF
- Go into the ELRS menu
- Bind with the model using the Menu
- Enable Wifi in the menu
- Go to `10.0.0.1/hardware.html` change pins: RX: 3  TX: 1 (note this will prohibit communication with the TX handset because it will listen on its own USB serial for CRSF communication)
- Test using DeckTX (921600 Baud)
