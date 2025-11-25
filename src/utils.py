#!/usr/bin/env python3
"""
MiRo utility constants and configurations.
"""
import numpy as np

# Audio constants
BLOCK_SAMPLES = 500
BLOCK_RATE = 40
MICS = 4
TONE_FREQUENCY = {'min': 50, 'max': 2000}
TONE_VOLUME = {'min': 0, 'max': 255}

# Visual salience map dimensions
PRI = {'width': 178, 'height': 100}
PRIW = {'width': 256, 'height': 1}

# Color definitions (BGR tuples and ARGB words)
BGR_TUPLE = {
    'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0),
    'yellow': (0, 255, 255), 'cyan': (255, 255, 0), 'magenta': (255, 0, 255),
    'white': (255, 255, 255), 'black': (0, 0, 0),
}

ARGB_WORD = {
    'red': 0xFFFF0000, 'green': 0xFF00FF00, 'blue': 0xFF0000FF,
    'yellow': 0xFFFFFF00, 'cyan': 0xFF00FFFF, 'magenta': 0xFFFF00FF,
    'white': 0xFFFFFFFF, 'black': 0x00000000, 'off': 0x00000000,
}

# Camera calibration parameters
MTX = np.array([
    [1.04358065e+03, 0, 3.29969935e+02],
    [0, 1.03845278e+03, 1.68243114e+02],
    [0, 0, 1]
])
DIST = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, 
                   -7.48630198e-04, -3.20700124e+01]])
FOCAL_LENGTH = 330
