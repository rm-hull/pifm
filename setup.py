from distutils.core import setup
from distutils.command.install import install as DistutilsInstall

import subprocess
import os

class MyInstall(DistutilsInstall):
    def run(self):
        run_makefile()
        DistutilsInstall.run(self)
        do_post_install()

def run_makefile():
    subprocess.call(["make", "all"])
    
def do_post_install():
    subprocess.call(["make", "clean"])

setup (name = "PiFm",
	version = '1.0',
	description = "Turn your Rasberry Pi into an FM Transmitter",
	author = ["Oliver Mattos", "Oskar Weigl"],
	license = "GPLv2",
	url = "http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter",
    cmdclass={'install': MyInstall},
    data_files = [('bin', ['pifm'])],
)

