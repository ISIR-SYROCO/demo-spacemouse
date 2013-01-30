Space mouse Demo:
=================

XDE must be called with some argument to configure corba:

XDE python-script.py -ORBInitRef NameService=corbaloc:iiop:127.0.0.1:2809/NameService

Twin robot demo:
----------------

In this demo, you interact with the master robot in gravity compensation mode through the spacemouse.
The slave robot uses a PD controller to follow the master robot joint position.

Run the master program:

`$XDE python/main_gravitycomp.py -ORBInitRef NameService=corbaloc:iiop:127.0.0.1:2809/NameService`

Then in a new terminal run the slave program:

`$XDE python/test_slave.py -ORBInitRef NameService=corbaloc:iiop:127.0.0.1:2809/NameService`

Troubleshooting:
----------------

### if libusb cannot access /dev/usb:

Create a new group called "usb" ("sudo groupadd usb")
Add yourself in this group ( maybe a better way, but we propose: "sudo gedit /etc/group" and append your user name after "usb" - for instance "usb:x:1001:yourname")
Add a udev rule to allow the group usb to access usb devices:

	cd /etc/udev/rules.d/
	sudo echo "SUBSYSTEM==\"usb\", GROUP==\"usb\"" > usb-permission.rules

You can choose another name of file
Then, restart udev, or restart computer

