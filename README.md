# DCF77 for RaspberryPi

DCF77 receiving daemon for the RaspberryPi
This program/daemon receive the DCF77 signal directly over the GPIO pins
and can push the date and time to NTP over SHM (shared memory).
It make use of the build in pull up resistors in the RaspberryPi.
No need to wire extra pull up resistors.

Caution: the GPIOs of the RaspberryPi can only handle up to 3.3V!
The most DCF77 modules start to work at 1.2V or 2.5V.
So, there shouldn't be a problem with any module.

This program need the ‚wiringPi‘ library.
# aptitude install wiringpi

The library ‚rt‘ (‚realtime‘) is allready included in the Raspian OS.

compile with:
$ gcc -Wall -pedantic -std=c99 -lrt -lwiringPi -o dcf77_clock dcf77_clock.c

To start, you need at least the ‚-g‘ parameter with the pin number where the module is wired.
If you have a receiver module with two outputs (normal and inverted),
you can give the parameter ‚-g‘ two times.
This program use the numbering from the ‚wiringPi‘ library.
  https://pinout.xyz/pinout/wiringpi

The parameter ‚-u‘ is the ‚Shared Memory Unit‘ from the NTPD,
where the program should push the data.
You can configure the NTPD in the ntp.conf wirg the following lines:

server 127.127.28.0 minpoll 6 maxpoll 6
fudge 127.127.28.0 time1 0.030 refid DCF

The pseudo IP 127.127.28.x configure a SHM where the NTPD should look for data.
The last number represent the unit number.
The units 0 and 1 are only writable by root.
Unit 2 and above can also be written by unprivileged users.

The falling and rising edge of the signal should have a time delay
of 100ms (bit value 0) or 200ms (bit value 1).
External influence or bad signal lead to different time delays.
You can set a time tolerance with the parameter ‚-t‘ (default is 25).
