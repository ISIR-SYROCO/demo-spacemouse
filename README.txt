Space mouse Demo

XDE must be called with some argument to configure corba:

XDE python-script.py -ORBInitRef NameService=corbaloc:iiop:127.0.0.1:2809/NameService



Troubleshooting:
================

* if libusb cannot access /dev/usb:
-----------------------------------
    Create a new group called "usb" ("sudo groupadd usb")
    Add yourself in this group ( maybe a better way, but we propose: "sudo gedit /etc/group" and append your user name after "usb" - for instance "usb:x:1001:yourname")
    Add a udev rule to allow the group usb to access usb devices:
        cd /etc/udev/rules.d/
        sudo echo "SUBSYSTEM==\"usb\", GROUP==\"usb\"" > usb-permission.rules   #you can choose another name of file
    Then, restart udev, or restart computer



