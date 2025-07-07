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



### PX4
These might not be necessary conditions but this set of settings has been found to be sufficient in aggregate:

1. ELRS Receiver: Disable telemetry transmission on the ELRS receiver
2. DroneBridge
  - Lower the max packet size to 16
  - Set Baudrate to 921600
  - Don't make contact with the carbon fiber frame
3. PX4
  - Set `MAV_0_MODE` to External Vision
  - Set `MAV_0_RATE` to 500 B/s
  - Set `SER_TEL1_BAUD` to 921600
  - Use the `SanDisk 32GB Extreme U3` SD card to prevent logging dropouts
  - Configure just a few topics in the `etc/logging/logger_topics.txt`
4. Laptop
  - Run forwarding script with `sudo nice -n-20`
  - Disconnect from Wifi
  - `sudo -E nice -n-20 ./.venv/bin/python choreo/individual_px4.py`
4. Vicon: 
  - 100Hz
  - Enable tracking
5. GL.inet MT300N-v2
  - Wifi running on `Mode: Client  Channel: 8 (2.447 GHz)  HT Mode: HT20` (though Channel 1 might be better with ELRS in the same room)