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
    - Fn + F2 (via DSDT)
    - Fn + F11 (Work in Progress)

* Headphone Amplifier Support

    - Configurable via sysfs.

      /sys/devices/platform/clevo/headphone_amp

* Misc Power Support

    - Turn Bluetooth On/Off via sysfs.

      /sys/devices/platform/clevo/bluetooth_power

    - Turn LCD On/Off via sysfs.

      /sys/devices/platform/clevo/lcd_power

    - Turn Trackpad On/Off via sysfs.

      /sys/devices/platform/clevo/trackpad_power

    - Turn Webcam via sysfs.

      /sys/devices/platform/clevo/webcam_power

* Airplane Mode LED

    - Exposed via sysfs.
    - Automatically toggled by RFKill.

 DSDT
------

* Patch to fix Trackpad via DSDT

    - Precompiled DSDT for the CLEVO P670RE.

