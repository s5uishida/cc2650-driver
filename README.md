# cc2650-driver
cc2650-driver is a java library for using [TI SensorTag CC2650](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide) by [bluez-dbus](https://github.com/hypfvieh/bluez-dbus) with [BlueZ](http://www.bluez.org/) version 5.50 on linux OS. I releases this in the form of the Eclipse plug-in project.
You need Java 8 or higher.

CC2650 is powered by a button battery **CR2032**. From CC2650, cc2650-driver get battery level and the following sensor information.

- System information  
  - Firmware version
  - Battery level  
    The battery level is available for firmware 1.30 or higher.

- Sensors  
  - IR Temperature (Object / Ambience)
  - Relative humidity
  - Barometric pressure
  - Optical
  - Movement (Gyroscope / Accelerometer / Magnetometer)

There are two ways to acquire data from various sensors of CC2650.
- Direct reading
- Notification  
  The data acquisition interval that can be specified by notification is 2550 (msec) at maximum.

In addition, for `Movement`, when CC2650 detects a shake using Wake-On-Motion function, it may notify ` Movement` data at a time interval specified by notification for 10 seconds.

I do not know how long CC2650 battery is effective. In addition, it seems that the sensors of `Movement` has relatively large power consumption. When the battery level reached approximately 60%, empirically CC2650 could not work properly.

I have confirmed that it works in Raspberry Pi 3B ([Raspbian Buster Lite OS](https://www.raspberrypi.org/downloads/raspbian/) (2019-07-10)).

## Install Raspbian Buster Lite OS (2019-07-10)
The reason for using this version is that it is the latest as of July 2019 and [BlueZ](http://www.bluez.org/) 5.50 is included from the beginning.

## Install jdk11 on Raspberry Pi 3B
For example, [jdk11 apt-install](https://apt.bell-sw.com/) at [BELLSOFT](https://bell-sw.com/) is shown below.
```
# wget -q -O - https://download.bell-sw.com/pki/GPG-KEY-bellsoft | apt-key add -
# echo "deb [arch=armhf] https://apt.bell-sw.com/ stable main" | tee /etc/apt/sources.list.d/bellsoft.list
# apt-get update
# apt-get install bellsoft-java11
```

## Install git
If git is not included, please install it.
```
# apt-get install git
```

## Use this with the following bundles
- [SLF4J 1.7.26](https://www.slf4j.org/)
- [bluez-dbus-osgi 0.1.2-SNAPSHOT](https://github.com/hypfvieh/bluez-dbus)

I would like to thank the authors of these very useful codes, and all the contributors.

## How to use
The following sample codes included in CC2650Driver.java will be helpful.
- CC2650Driver#testRead()
- CC2650Driver#testNotification()
- CC2650Driver#testWakeOnMotion()
