#!/usr/bin/python

from subprocess import call


def play_sound( filename, carrier_freq=99.9, sample_rate=22050, stereo=False):
    if stereo:
        call(["./pifm", filename, carrier_freq, sample_rate, "stereo"])
    else:
        call(["./pifm", filename, carrier_freq, sample_rate])
    return
