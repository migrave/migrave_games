#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "migrave_game_imitation",
        "migrave_game_emotions",
    ],
    package_dir={
        "migrave_game_imitation": "ros/src/migrave_game_imitation",
        "migrave_game_emotions": "ros/src/migrave_game_emotions",
    },
)

setup(**d)
