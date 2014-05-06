"""Turn your Rasberry Pi into an FM Transmitter"""
__author__ = ["Oliver Mattos", "Oskar Weigl", "Richard Hirst"]
__author_email__ = ["omattos@gmail.com"]
__license__ = "GPLv2"
__version__ = "1.0"
__url__ = "http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter",

from radio import play_sound

__all__ = [play_sound]
