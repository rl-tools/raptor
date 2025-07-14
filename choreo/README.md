### Serial Device Registry
```
sudo mkdir /dev/serial/by-name/
sudo ln -s /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_F4\:12\:FA\:DC\:98\:F8-if00 /dev/serial/by-name/m5stamp-forwarder
sudo ln -s /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_48\:CA\:43\:B6\:44\:48-if00 /dev/serial/by-name/m5stamp-uav
sudo ln -s /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 /dev/serial/by-name/elrs-transmitter1
sudo ln -s /dev/serial/by-id/usb-1a86_USB_Single_Serial_5969026050-if00 /dev/serial/by-name/elrs-transmitter2
```


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
- Disable Model Match (bind to normal transmitter with the ELRS Lua script, and disable Model match there).
- Make sure the propeller directions (CW/CCW) are not inverted (front-left: CW, front-right: CCW, back-left: CCW, back-right: CW)
- Use the LED colors if available to check if the Mocap feedback works. Set to mode Color and then modulate by e.g. "Roll" which is the x displacement in our Mocap over ELRS conventions
- Disable Telemetry on the Drone to reduce RF load



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
  - Check `listener rl_tools_policy_status` for the `visual_odometry_dt_std` and `visual_odometry_dt_max` 3000 and ~30000 (uS) are decent values respectively (when feeding Mocap at 100Hz)
4. Laptop
  - Run forwarding script with `sudo nice -n-20`
  - Disconnect from Wifi
  - `sudo -E nice -n-20 ./.venv/bin/python choreo/individual_px4.py`
4. Vicon: 
  - 100Hz
  - Enable tracking
  - Make `Tracker.exe` Real-time priority
5. GL.inet MT300N-v2
  - Wifi running on `Mode: Client  Channel: 8 (2.447 GHz)  HT Mode: HT20` (though Channel 1 might be better with ELRS in the same room)


  ### Deadman Switch
  Ikkegol footpedal:

  ```
  sudo gpasswd -a $USER input
  ```