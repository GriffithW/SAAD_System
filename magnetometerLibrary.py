import smbus2
import math
import time

class QMC5883LCompass:
    def __init__(self, address=0x0D):
        self._ADDR = address
        self._magneticDeclinationDegrees = 0.0
        self._smoothUse = False
        self._smoothSteps = 5
        self._smoothAdvanced = False
        self._vRaw = [0, 0, 0]
        self._vCalibrated = [0, 0, 0]
        self._vSmooth = [0, 0, 0]
        self._vHistory = [[[0 for _ in range(3)] for _ in range(10)] for _ in range(3)]
        self._vTotals = [0, 0, 0]
        self._vScan = 0
        self._offset = [0.0, 0.0, 0.0]
        self._scale = [1.0, 1.0, 1.0]

        self.bus = smbus2.SMBus(1)

    def init(self):
        self._writeReg(0x0B, 0x01)
        self.setMode(0x01, 0x0C, 0x10, 0x00)

    def setADDR(self, address):
        self._ADDR = address

    def _writeReg(self, reg, value):
        try:
            self.bus.write_byte_data(self._ADDR, reg, value)
        except Exception as e:
            return
            
    def setMode(self, mode, odr, rng, osr):
        self._writeReg(0x09, mode | odr | rng | osr)

    def setMagneticDeclination(self, degrees, minutes):
        self._magneticDeclinationDegrees = degrees + minutes / 60.0

    def setReset(self):
        self._writeReg(0x0A, 0x80)

    def setSmoothing(self, steps, adv):
        self._smoothUse = True
        self._smoothSteps = min(steps, 10)
        self._smoothAdvanced = adv

    def calibrate(self):
        self.clearCalibration()
        calibrationData = [[65000, -65000], [65000, -65000], [65000, -65000]]
        x = calibrationData[0][0] = calibrationData[0][1] = self.getX()
        y = calibrationData[1][0] = calibrationData[1][1] = self.getY()
        z = calibrationData[2][0] = calibrationData[2][1] = self.getZ()

        startTime = time.time()
        while time.time() - startTime < 20:
            self.read()
            x = self.getX()
            y = self.getY()
            z = self.getZ()
            print("X: " + str(x), ", Y: " + str(y) + ", Z: " + str(z))

            calibrationData[0][0] = min(calibrationData[0][0], x)
            calibrationData[0][1] = max(calibrationData[0][1], x)
            calibrationData[1][0] = min(calibrationData[1][0], y)
            calibrationData[1][1] = max(calibrationData[1][1], y)
            calibrationData[2][0] = min(calibrationData[2][0], z)
            calibrationData[2][1] = max(calibrationData[2][1], z)
            time.sleep(0.1)

        self.setCalibration(
            calibrationData[0][0], calibrationData[0][1],
            calibrationData[1][0], calibrationData[1][1],
            calibrationData[2][0], calibrationData[2][1]
        )

        print("sensor.setCalibration(" + 
            str(calibrationData[0][0]) + ", " + str(calibrationData[0][1]) + ", \n" + 
            str(calibrationData[1][0]) + ", " + str(calibrationData[1][1]) + ", \n" + 
            str(calibrationData[2][0]) + ", " + str(calibrationData[2][1]) + ")" )

    def setCalibration(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.setCalibrationOffsets(
            (x_min + x_max) / 2.0,
            (y_min + y_max) / 2.0,
            (z_min + z_max) / 2.0
        )

        x_avg_delta = (x_max - x_min) / 2.0
        y_avg_delta = (y_max - y_min) / 2.0
        z_avg_delta = (z_max - z_min) / 2.0

        avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3.0

        self.setCalibrationScales(
            avg_delta / x_avg_delta,
            avg_delta / y_avg_delta,
            avg_delta / z_avg_delta
        )

    def setCalibrationOffsets(self, x_offset, y_offset, z_offset):
        self._offset = [x_offset, y_offset, z_offset]

    def setCalibrationScales(self, x_scale, y_scale, z_scale):
        self._scale = [x_scale, y_scale, z_scale]

    def getCalibrationOffset(self, index):
        return self._offset[index]

    def getCalibrationScale(self, index):
        return self._scale[index]

    def clearCalibration(self):
        self.setCalibrationOffsets(0.0, 0.0, 0.0)
        self.setCalibrationScales(1.0, 1.0, 1.0)

    def read(self):
        try:
            self.bus.write_byte_data(self._ADDR, 0x00, 0x01)
            data = self.bus.read_i2c_block_data(self._ADDR, 0x00, 6)
        except Exception as e:
            return
        self._vRaw[0] = (data[0] | (data[1] << 8))
        self._vRaw[1] = (data[2] | (data[3] << 8))
        self._vRaw[2] = (data[4] | (data[5] << 8))

        self._vRaw[0] = (self._vRaw[0] - 65536) if self._vRaw[0] > 32768 else self._vRaw[0]
        self._vRaw[1] = (self._vRaw[1] - 65536) if self._vRaw[1] > 32768 else self._vRaw[1]
        self._vRaw[2] = (self._vRaw[2] - 65536) if self._vRaw[2] > 32768 else self._vRaw[2]

        self._applyCalibration()

        if self._smoothUse:
            self._smoothing()

    def _applyCalibration(self):
        self._vCalibrated[0] = (self._vRaw[0] - self._offset[0]) * self._scale[0]
        self._vCalibrated[1] = (self._vRaw[1] - self._offset[1]) * self._scale[1]
        self._vCalibrated[2] = (self._vRaw[2] - self._offset[2]) * self._scale[2]
        # print("X0: " + str(self._vRaw[0]), ", Y0: " + str(self._vRaw[1]) + ", Z0: " + str(self._vRaw[2]))
        # print("X1: " + str(self._vCalibrated[0]), ", Y1: " + str(self._vCalibrated[1]) + ", Z1: " + str(self._vCalibrated[2]))


    def _smoothing(self):
        max_val = 0
        min_val = 0

        if self._vScan > self._smoothSteps - 1:
            self._vScan = 0

        for i in range(3):
            if self._vTotals[i] != 0:
                self._vTotals[i] -= self._vHistory[self._vScan][i]
            self._vHistory[self._vScan][i] = self._vCalibrated[i]
            self._vTotals[i] += self._vHistory[self._vScan][i]

            if self._smoothAdvanced:
                max_val = max(range(self._smoothSteps - 1), key=lambda j: self._vHistory[j][i])
                min_val = min(range(self._smoothSteps - 1), key=lambda k: self._vHistory[k][i])

                self._vSmooth[i] = (self._vTotals[i] - (self._vHistory[max_val][i] + self._vHistory[min_val][i])) / (self._smoothSteps - 2)
            else:
                self._vSmooth[i] = self._vTotals[i] / self._smoothSteps

        self._vScan += 1

    def getX(self):
        return self._get(0)

    def getY(self):
        return self._get(1)

    def getZ(self):
        return self._get(2)

    def _get(self, i):
        if self._smoothUse:
            return self._vSmooth[i]
        return self._vCalibrated[i]

    def getAzimuth(self):
        heading = math.atan2(self.getY(), self.getX()) * 180.0 / math.pi
        heading += self._magneticDeclinationDegrees
        return int(heading) % 360

    def getBearing(self, azimuth):
        a = azimuth / 22.5
        r = a - int(a)
        return int(math.ceil(a) if r >= 0.5 else math.floor(a))

    def getDirection(self, azimuth):
        bearings = [
            [' ', ' ', 'N'], ['N', 'N', 'E'], [' ', 'N', 'E'], ['E', 'N', 'E'],
            [' ', ' ', 'E'], ['E', 'S', 'E'], [' ', 'S', 'E'], ['S', 'S', 'E'],
            [' ', ' ', 'S'], ['S', 'S', 'W'], [' ', 'S', 'W'], ['W', 'S', 'W'],
            [' ', ' ', 'W'], ['W', 'N', 'W'], [' ', 'N', 'W'], ['N', 'N', 'W'],
            [' ', ' ', 'N']
        ]
        
        # Get the bearing direction index
        direction = self.getBearing(azimuth) % len(bearings)
        # Return the corresponding direction from the bearings array
        return ''.join(bearings[direction]).strip()

