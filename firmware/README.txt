1. Install ESP-IDF(version 5.1) 
   Documentation:https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html

2. To build code open ESP-IDF5.1.CMD  command prompt.
3. Navigate to the corresponding folder tiago or gripper.
4. Set the target  "idf.py set-target ESP32S3".
5. Build project "idf.py -p build".
6. Flash project "idf.py -p COMx flash".
7. Monitor com port "idf.py -p COMx monitor".

To change the number of  virtual com ports in tiago.
1. Navigate to the Tiago folder.
2. Open menuconfig "idf.py menuconfig". 
3. Navigate to "Example Configuration".
4. Change the number of "channel".
5. Save the configuration[S].
6. Build the project.
7. To flash press the Boot button and press reset button.
8. Flash the code.
