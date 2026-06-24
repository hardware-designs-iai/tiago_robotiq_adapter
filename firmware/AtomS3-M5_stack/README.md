## Master flashing guide

1. Insert AtomS3U-M5 stack into pc. Get the port address by:
```bash
ls /dev/ttyACM*
```
2. Activate ESP-IDF terminal and, go to:
```bash
cd tiago_robotiq_adapter/firmware/AtomS3-M5_stack
```
3. Note down Master's MAC address, we'll need it while flashing firmware for Grippers (Slave)
```python
python -m esptool -p /dev/ttyACM0 read-mac
```
4. Replace MAC address of both slaves in `tusb_serial_device_main.c`.
5. Change CDC count
```bash
idf.py menuconfig
```
`Component config → TinyUSB Stack → Communication Device Class (CDC) → CDC Channel Count → 2`

Save the configuration.

6. Flash the firmware in ESP-IDF terminal using this command:
```bash
idf.py -p /dev/ttyACM[x] flash
``` 
replace `x` with actual hardware address of Master device.

---
    
**Note: If you get device not found error, press EN/RST button on AtomS3U device until it blinks green.**  