from distutils.core import setup
from distutils.command.install import install as DistutilsInstall
from distutils.command.build import build as DistutilsBuild

import subprocess
import os

import PiFM

class MyInstall(DistutilsInstall):
    def run(self):
        DistutilsInstall.run(self)
        do_post_install()

class MyBuild(DistutilsBuild):
    def run(self):
        run_makefile()
        DistutilsBuild.run(self)

def run_makefile():
    subprocess.call(["make", "all"])
    
def do_post_install():
    subprocess.call(["make", "clean"])

setup (name = "PiFM",
    version = PiFM.__version__,
    description = PiFM.__doc__,
    author = PiFM.__author__,
    author_email =  PiFM.__author_email__,
    license = PiFM.__license__,
    url = PiFM.__url__,
    keywords = ['RaspberryPi', 'FM', 'Radio', 'FM Transmitter'],
    packages = ['PiFM'],
    cmdclass={'install': MyInstall, 'build': MyBuild},
    data_files = [('bin', ['pifm'])],
)

