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

    - Turn Webcam On/Off via sysfs.

      /sys/devices/platform/clevo/webcam_power

* Airplane Mode LED

    - Exposed via sysfs.

      /sys/devices/platform/clevo/leds/clevo:green:airplane_mode

    - Automatically toggled by RFKill.

 DSDT
------

* Patch to fix Trackpad via DSDT

    - Precompiled DSDT for the CLEVO P670RE.

* TPM configuration patch added for those interested.

 Errata
========

 Dual Booting
--------------

Windows driver causes the trackpad button to send scan codes and disables
trackpad power control via WMI.

I've added 'trackpad-keycodes.sh' which will make the button send KEY_PROG3
(XF86Launch3 in X11) key codes. This can be used to toggle the trackpad via
synclient.

**NOTICE** Rebooting will not fix the issue. You will have to power off and
disconnect the charger for a few minutes.
