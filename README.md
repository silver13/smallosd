# smallosd
*micro minimOSD standalone firmware*

To compile use the Arduino IDE. The upload procedure is similar to that for an arduino mini pro , 
only the selected board should be Arduino nano 328.

The firmware supports 2 voltages right now, which can be connected to any of the 4 inputs, however
since BATT1 / BATT2 have 1/16 dividers they are not recommended for 1S range voltages.

The CURR and RSSI inputs have no dividers so they should not exceed 5V by a large margin. The board can also display VCC as one of the voltages.

The board is powered by 5V. At approx 4.5V the OSD will stop functioning, and lower it will lose video. The OSD recovers from mild undervoltages ( >3v ) but an exclamation mark is shown since some brownouts are too short to be noticed or can be hard to identify.
