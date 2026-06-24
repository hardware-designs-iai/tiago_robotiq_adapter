## Slave flashing guide

1. Connect Gripper's pcb via USB-C to pc. Get the port address by:
```bash
ls /dev/ttyACM*
```
2. Activate ESP-IDF terminal and, go to:
```bash
cd tiago_robotiq_adapter/firmware/gripper
```
3. Note down Slave's MAC address, we'll need it while flashing firmware for AtomS3U (Master)
```python
python -m esptool -p /dev/ttyACM[x] read-mac
```
4. Replace MAC address of Master device in `espnow_gripper_main.c`.


5. Flash the firmware in ESP-IDF terminal using this command in both the grippers:
```bash
idf.py -p /dev/ttyACM[x] flash
``` 
replace `x` with actual hardware address of Master device.
    