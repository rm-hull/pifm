Turning the Raspberry Pi Into an FM Transmitter
===============================================

(Created by Oliver Mattos and Oskar Weigl. Code is GPL)

Steps to play sound:
--------------------
Download the code here, and compile:

    make clean all
    
Now connect a 20cm or so plain wire to GPIO 4 to act as an antenna, and tune an FM radio to 100.0Mhz

    sudo python
    >>> import PiFm
    >>> PiFm.play_sound("sound.wav")

Just run the above code in the same folder. The antenna is optional, but range
is reduced from ~100 meters to ~10cm without the antenna. The sound file must
be 16 bit mono wav format. 

How to change the broadcast frequency
-------------------------------------
In `pifm.c`, see line 106. The "5a" number here is a hardware specific password -
ignore it (see the [datasheet](http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf)
for specifics). The "0x5000" is the carrier frequency, it should be interpreted
as 5.000 (in hex), and m is the added audio modulation. This chooses how the
500Mhz system clock is divided down to produce the 100Mhz FM carrier. Hence if
you wanted a 99.0Mhz carrier, you would divide by 5.050505 (decimal), which in
hex is 5.0CF. Hence if you changed the number on that line to 0x50CF you should
be able to get a 99.0Mhz FM signal. Most radio receivers want a signal to be a
multiple of 0.1 MHz to work properly.  The details of how it works

Below is some code that was hacked together over a few hours at the [Code Club
pihack](http://blog.codeclub.org.uk/blog/brief/). It uses the hardware on the 
raspberry pi that is actually meant to generate spread-spectrum clock signals
on the GPIO pins to output FM Radio energy. This means that all you need to do
to turn the Raspberry-Pi into a (ridiculously powerful) FM Transmitter is to
plug in a wire as the antenna (as little as 20cm will do) into GPIO pin 4 and
run the code posted below. It transmits on 100.0 MHz.  When testing, the signal
only started to break up after we went through several conference rooms with
heavy walls, at least 50m away, and crouched behind a heavy metal cabinet. The
sound quality is ok, but not amazing, as it currently plays some clicks when
the CPU gets switched away to do anything else than play the music. The plan
was to make a kernel mode driver that would be able to use the DMA controller
to offload the CPU and play smooth music without loading the CPU, but we ran
out of time.  If you're v.  smart, you might be able to get stereo going!

Accessing Hardware
------------------
The python library calls a C program (provided both precompiled and in source
form). The C program maps the Peripheral Bus (0x20000000) in physical memory
into virtual address space using /dev/mem and mmap. To do this it needs root
access, hence the sudo. Next it sets the clock generator module to enabled and
sets it to output on GPIO4 (no other accessible pins can be used). It also sets
the frequency to 100.0Mhz (provided from PLLD@500Mhz, divided by 5), which
provides a carrier. At this point, radios will stop making a "fuzz" noise, and
become silent.  Modulation is done by adjusting the frequency using the
fractional divider between 100.025Mhz and 99.975Mhz, which makes the audio
signal. The fractional divider doesn't have enough resolution to produce more
than ~6 bit audio, but since the PI is very fast, we can do oversampling to
provide about 9.5 bit audio by using 128 subsamples per real audio sample.

Notes
-----
This is a copy of the original documentation and code from 
http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter, 
all rights of the original authors reserved.

References
----------
* http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter

* http://blog.makezine.com/2012/12/10/raspberry-pi-as-an-fm-transmitter/?parent=Electronics

* http://www.youtube.com/v/ekcdAX53-S8#! 
