#!/usr/bin/env python

from subprocess import call


def play_sound( filename ):
   call(["./pifm", filename])
   return
