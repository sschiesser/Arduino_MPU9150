<<<<<<< HEAD
# Important note

Pansenti, LLC is no longer in business and this repo is no longer actively supported. Please checkout www.richards-tech.com for the latest IMU-related software developments.

# MPU9150Lib

MPU9150Lib is an implementation of 9-axis data fusion on an Arduino using the InvenSense MPU-9150 IMU. See MPU9150Lib.pdf for details.

July 2, 2013
------------
Problems have been reported with the latest IDE (1.5) and USB Arduinos due to the code size being too large when taking into account the much large boot code. There is now a new #define:

//#define MPU_MAXIMAL

in inv_mpu.h that, when commented out (as it is by default), takes out some unused parts of the MotionDriver library. However, this is still insufficient so the example sketches will not currently fit on Leonardo and USB LilyPads.

May 7, 2013
-----------
MPU9150Lib now supports the Arduino Due.

There is a new sketch, Due9150, and new versions of the libraries that allow the Arduino Due to be used with one or two MPU-9150s. The Due does not have EEPROM so calibration data is stored in flash. This means that it is lost every time that new code is uploaded as the Arduio IDE completely clears flash on every upload. Consequently, calibration software has been added to the main Due9150 sketch as switching between sketches is impossible. Check Due9150.ino for information on how to run the calibration code.

*** Update **

Please note that it is possible for a processor restart to interrupt an I2C transfer and this can leave the I2C bus in a hung state due to the MPU-9150 permanently pulling the SDA line low. Due9150 will display an error message if this occurs. The only known way to recover from this situation is to power cycle the system.


May 2, 2013
-----------
MPU9150Lib now supports two IMU chips on a single Arduino at addresses 0x68 and 0x69.

Arduino9150 has a #define called DEVICE_TO_USE. Set this to 0 for the default of 0x68, 1 for the alternate IMU address of 0x69.

Added a new sketch ArduinoDual9150 to demonstrate how to use two devices in a system simultaneously.

Modified calibration code so that two different IMUs can be calibrated. Set DEVICE_TO_CALIBRATE to 0 (the default) in the sketch to calibrate the MPU-9150 at address 0x68 or 1 to calibrate the MPU-9150 at address 0x69. In a dual MPU-9150 system, AccelCal9150 and MagCal9150 should be run with both settings in order to calibrate both devices. Each IMU has its own independent area in EEPROM for calibration data. 

April 30, 2013
--------------
Moved mpu_set_lpf call so that it is the last call made in init().

Adjusted the way that calibrated accels is calculated.

April 29, 2013
--------------
Added two parameters to the library init() function. The first controls the magnetometer update rate separately from the DMP update rate. The second controls the low pass filter setting. See MPU9150Lib.h
for more details.

The calibrated accelerometer data should now reflect the correct offsets. The DMP only puts raw accel data in the FIFO so the library now adjusts the accel values based on the calibration data.

March 30, 2013
--------------
Added Accel9150 to demonstrate how to calculate residual accelerations in the body frame.

Added magMix parameter to MPU6150Lib::init(). This controls how much influence the magnetometer has on the yaw information:
  
  0 = Just use MPU-9150 gyros and ignore the magnetometer
  1 = Just use magnetometer and ignore gyro yaw information
  2-n = Fuse gyro and magnetometer information. The higher the value, the less influence the magnetometer has.

The value that's best will depend on the application and the environment. See MPU9150Lib.h for more.

March 26, 2012
--------------
This is the initial version of the MPU9150Lib software.

For more information, see www.pansenti.com,  pansenti.wordpress.com and twitter.com/Pansenti.

Pansenti can be contacted by emailing to info@pansenti.com.
=======
# Arduino_MPU9150
>>>>>>> cfc52719d685002e44e4fc3a21521d67f939c71d
