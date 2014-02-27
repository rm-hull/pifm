## UPDATE

~~### https://github.com/richardghirst/PiBits/blob/master/PiFmDma/PiFmDma.c presents a rewrite using DMA, which uses much less CPU than this version. Please use that version in preference.~~

This has been updated by the original authors to use DMA, as well as allow tuning and stereo.  Richard Hirst is no longer maintaining PiBits, and refers users to this solution.

##Turning the Raspberry Pi Into an FM Transmitter

###Steps to play sound:

*(Created by Oliver Mattos and Oskar Weigl. Code is GPL)*

```
sudo python
>>> import PiFm
>>> PiFm.play_sound("sound.wav")
```

Now connect a 20cm or so plain wire to GPIO 4 (which is pin 7 on [header P1](http://elinux.org/RPi_Low-level_peripherals#General_Purpose_Input.2FOutput_.28GPIO.29)) to act as an antenna, and tune an FM radio to 103.3Mhz

from a [post on MAKE](http://blog.makezine.com/2012/12/10/raspberry-pi-as-an-fm-transmitter/?parent=Electronics) by Matt Richardson

The antenna is optional, but range is reduced from ~100 meters to ~10cm without the antenna. The sound file must be 16 bit ~~mono~~ wav format.

###New! Now with stereo

```
sudo ./pifm left_right.wav 103.3 22050 stereo

# Example command lines
# play an MP3
ffmpeg -i input.mp3 -f s16le -ar 22.05k -ac 1 - | sudo ./pifm -

# Broadcast from a usb microphone (see arecord manual page for config)
arecord -d0 -c2 -f S16_LE -r 22050 -twav -D copy | sudo ./pifm -
```

###How to change the broadcast frequency

Run the ./pifm binary with no command line arguments to find usage.

The second command line argument is the frequency to transmit on, as a number in Mhz. Eg. This will transmit on 100.0

> sudo ./pifm sound.wav 100.0

It will work from about 1Mhz up to 250Mhz, although the useful FM band is 88 Mhz to 108 Mhz in most countries.

Most radio receivers want a signal to be an odd multiple of 0.1 MHz to work properly.

###The details of how it works

Below is some code that was hacked together over a few hours at the [Code Club pihack](http://blog.codeclub.org.uk/blog/brief/). It uses the hardware on the raspberry pi that is actually meant to generate spread-spectrum clock signals on the GPIO pins to output FM Radio energy. This means that all you need to do to turn the Raspberry-Pi into a (ridiculously powerful) FM Transmitter is to plug in a wire as the antenna (as little as 20cm will do) into GPIO pin 4 and run the code posted below. It transmits on 100.0 MHz.

When testing, the signal only started to break up after we went through several conference rooms with heavy walls, at least 50m away, and crouched behind a heavy metal cabinet. The sound quality is ok, but not amazing, as it currently plays some clicks when the CPU gets switched away to do anything else than play the music. The plan was to make a kernel mode driver that would be able to use the ~~DMA controller to offload the CPU and play smooth music without loading the CPU, but we ran out of time.~~ Now Done and working, DMA from userspace is awesome and awful at the same time!

~~If you're v. smart, you might be able to get stereo going!~~ Done!

###Accessing Hardware

The python library calls a C program. The C program maps the Peripheral Bus (0x20000000) in physical memory into virtual address space using /dev/mem and mmap. To do this it needs root access, hence the sudo. Next it sets the clock generator module to enabled and sets it to output on GPIO4 (no other accessible pins can be used). It also sets the frequency to ~~100.0Mhz (provided from PLLD@500Mhz, divided by 5)~~ 103.3, which provides a carrier. At this point, radios will stop making a "fuzz" noise, and become silent.

Modulation is done by adjusting the frequency using the fractional divider between 103.325Mhz and 103.275Mhz, which makes the audio signal. ~~The fractional divider doesn't have enough resolution to produce more than ~6 bit audio, but since the PI is very fast, we can do oversampling to provide about 9.5 bit audio by using 128 subsamples per real audio sample.~~ We were being naive with our subsampling algorithm - you can now get full 16 bit quality sound, and it even does FM pre-emphasis so that the result doesn't sound bass-heavy. 

###Notes

This is a copy of the updated documentation and code from 
http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter, 

The only changes are removal of the download link (since the source can be downloaded here), and formatting in Github Markdown.

All rights of the original authors reserved.

###References

* http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter

* http://blog.makezine.com/2012/12/10/raspberry-pi-as-an-fm-transmitter/?parent=Electronics

* http://www.youtube.com/v/ekcdAX53-S8#! 

* https://github.com/richardghirst/PiBits/pull/18
