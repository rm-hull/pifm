#!/usr/bin/python

from subprocess import call


def play_sound( filename, carrier_freq=99.9, sample_rate=22050, stereo=False):
    """Broadcast wav file `filename` using pifm for Raspberry Pi.

    Radio is tuned to `carrier_freq` (MHz)
    """
    if stereo:
        call(["pifm", filename, "{}".format(carrier_freq), "{}".format(sample_rate), "stereo"])
    else:
        call(["pifm", filename, "{}".format(carrier_freq), "{}".format(sample_rate)])
    return
