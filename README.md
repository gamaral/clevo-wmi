 CLEVO/Notebook P65 WMI Driver
===============================

**Work in Progress**

Tested on my **P65** CLEVO P670RE (SAGER NP8677-S).

 Working
---------

* Hotkey Support

    - Fn + 2
    - Fn + Backspace
    - Fn + ESC
    - Fn + F2
    - Fn + F11

* Headphone Amplifier Support

    - Configurable via sysfs.

      /sys/devices/platform/clevo/headphone_amp

* Misc Power Support

    - Turn Bluetooth On/Off via sysfs.

      /sys/devices/platform/clevo/bluetooth_power

    - Turn LCD On/Off via sysfs.

      /sys/devices/platform/clevo/lcd_power

    - Turn Webcam On/Off via sysfs.

      /sys/devices/platform/clevo/webcam_power

* Airplane Mode LED

    - Exposed via sysfs.

      /sys/devices/platform/clevo/leds/clevo:green:airplane_mode

    - Automatically toggled by RFKill.

 TRACKPAD
----------

    Toggleing FN+F2 will generate Touchpad key events in:

        /dev/input/by-path/platform-clevo-event

    Since X11 doesn't see keycodes over 255, they won't be usable in keybinders
    like *xbindkeys*.  You have two choices, use something like **evbind** or
    change the keycodes returned by the driver.

    I've also made evbind available under my GitHub account.

 DSDT
------

* TPM configuration patch added for those interested.

 Errata
========

