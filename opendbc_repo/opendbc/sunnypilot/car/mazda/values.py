"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class MazdaSafetyFlagsSP:
  DEFAULT = 0
  TORQUE_INTERCEPTOR = 1


class MazdaFlagsSP(IntFlag):
  """
    Flags for Mazda specific quirks within sunnypilot.
  """
  TORQUE_INTERCEPTOR = 1


class TI_STATE:
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3


class TICarControllerParams:
  """Steer torque limits for the torque interceptor channel (CAM_LKAS2, bus 1)."""
  STEER_MAX = 600                  # theoretical max_steer 2047
  STEER_DELTA_UP = 6               # torque increase per refresh
  STEER_DELTA_DOWN = 15            # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15      # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 40     # weight driver torque
  STEER_DRIVER_FACTOR = 1          # from dbc


class TI_LKAS_LIMITS:
  STEER_THRESHOLD = 6  # TI torque sensor units (~±85 full scale)
