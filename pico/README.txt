If using a new Raspberry PI Pico, hold down the BOOTSEL button on the Pico while plugging it in.
Then, copy the .uf2 file to the drive that appears. The drive will close once it recognizes the
file and the code will now be running on the Pico.

This will only work with two REV Color Sensor V3's. If you want more, you will have to make that
code yourself. See the linked GitHub repo.

The .uf2 file was provided by this GitHub repo: https://github.com/ThadHouse/picocolorsensor

Pinout of Pico:
  https://www.raspberrypi-spy.co.uk/wp-content/uploads/2021/01/raspberry_pi_pico_pinout.png

===============================

HOOKING UP THE PICO TO THE ROBORIO:
- MXP (roboRIO) to Pico:
  - Pin 1 MXP to pin 39 Pico
  - Pin 8 MXP to pin 38 Pico
  - Pin 10 MXP to pin 1 Pico
- RoboRIO to REV Sensors
  - Both REV sensors ground to pin 8 Pico
  - Both REV sensors power to pin 36 Pico
  - REV SDA (white) to pins 6 and 9 respectively Pico
  - REV SCL (blue) to pins 7 and 10 respectively Pico
