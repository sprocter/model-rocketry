"""State estimation / filtering for a model rocket.

Kalman filter implementation from vikas m at geeksforgeeks.com

--------------------------------------------------------------------------------
Copyright (C) 2025-2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from ulab import numpy as np
from fusion import Fusion
from orientate import orientate

class StateEstimator:

    def __init__(self, period: float, accel_err: float, alti_err: float):
        """Initialize the estimator

        This initializes a Kalman filter using the supplied timestep and errors (as standard deviations)

        :param float period: The time between readings in milliseconds
        :param float alti_err: The standard deviation of altimeter readings in meters
        :param float accel_err: The standard deviation of accelerometer readings in meters per second per second
        """
        self.KF = KalmanFilter(period / 1000, accel_err, alti_err)
        # self.transpose = (1, 2, 0)  # Y -> Z, Z -> X, X -> Y
        # self.transpose = (0, 1, 2)  # No changes
        # self.invert = (True, False, False)  # Invert X (I think?)
        self.acceleration = [0.0, 0.0, 0.0]
        self.gyroscope = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.fuse = Fusion()

    @property
    def altitude(self) -> float:
        return self.KF.x[0][0]

    @property
    def velocity(self) -> float:
        return self.KF.x[1][0]

    @property
    def heading(self) -> float:
        return self.fuse.heading

    @property
    def pitch(self) -> float:
        return self.fuse.pitch

    @property
    def roll(self) -> float:
        return self.fuse.roll

    @property
    def acceleration(self) -> float:
        return self._acceleration  # Just returns the cached sensor reading

    @property
    def gyroscope(self) -> float:
        return self._gyro  # Just returns the cached sensor reading
    
    @property
    def magnetometer(self) -> float:
        return self._magnetometer  # Just returns the cached sensor reading

    @altitude.setter
    def altitude(self, value) -> None:
        self.KF.predict(self.acceleration[1]) # TODO: Should we use more than the Y value? Adjust for pitch?
        self.KF.update(value)
        # TODO: This should be "orientated" once the mounting / payload position is known / established
        self.fuse.update(
            self.acceleration, 
            # The fusion module seems to want the Z axis to spin the other way
            orientate((0,1,2), (False, False, True), self.gyroscope)[0],
            self.magnetometer
        )

    @acceleration.setter
    def acceleration(self, value) -> None:
        self._acceleration = value

    @gyroscope.setter
    def gyroscope(self, value) -> None:
        self._gyro = value

    @magnetometer.setter
    def magnetometer(self, value) -> None:
        self._magnetometer = value



class KalmanFilter:
    # Adapted from https://www.geeksforgeeks.org/python/kalman-filter-in-python/
    def __init__(self, delta_t, accel_err_std_dev, alti_err_std_dev):
        F = np.array([[1, delta_t], [0, 1]])
        G = np.array([[0.5 * delta_t**2], [delta_t]])
        H = np.array([[1, 0]])
        Q = (
            np.array(
                [[(delta_t**4) / 4, (delta_t**3) / 2], [(delta_t**3) / 2, delta_t**2]]
            )
            * accel_err_std_dev**2
        )
        R = np.array([[alti_err_std_dev**2]])

        x0 = np.array([[0], [0]])
        P0 = np.array([[500, 0], [0, 500]])

        self.F = F
        self.G = G
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0

    @micropython.native
    def predict(self, u: float) -> None:
        # Change this if you're launching somewhere other than earth 😄
        u -= 9.80665  # https://en.wikipedia.org/wiki/Standard_gravity
        u = np.array([[u]])
        self.x = np.dot(self.F, self.x) + np.dot(self.G, u)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    @micropython.native
    def update(self, z: float) -> None:
        z = np.array([[z]])
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
