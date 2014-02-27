#!/usr/bin/python

from subprocess import call


def play_sound( filename ):
   call(["./pifm", filename])
   return