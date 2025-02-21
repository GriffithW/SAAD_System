import RPi.GPIO as GPIO
from time import sleep
import numpy as np
import sys
import gps
import collections
import math
with open("/home/saad-sys/workspace/magnetometerLibrary.py")  as file:
  exec(file.read())



# Connect to the gpsd daemon
session = gps.gps(mode=gps.WATCH_ENABLE)

# Pin configuration
LEFT_MOTOR = 12
RIGHT_MOTOR = 13  # GPIO pin connected to the PWM device

# Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR, GPIO.OUT)

# Clock
CLOCK = 50

# Create PWM instance with a frequency of 50 Hz
pwm = GPIO.PWM(LEFT_MOTOR, CLOCK)
pwm2 = GPIO.PWM(RIGHT_MOTOR, CLOCK)

# Setup a circular buffer
pastGPSPositions = collections.deque(maxlen=5)
pastGPSPositions.append((32.231311667, -110.957048333))
pastGPSPositions.append((32.231322461931185, -110.95715212191602))
pastGPSPositions.append((32.23132366727646, -110.95729089480041))
pastGPSPositions.append((32.23132864818837, -110.9574352940373))
pastGPSPositions.append((32.231332086967264, -110.95781008036023))

targetCoordinates = (0, 0)

# Initialize sensor
sensor = QMC5883LCompass()
sensor.init()
sensor.setCalibration(
    -1070.0, 1766.0, 
    -2366.0, 495.0, 
    -1575.0, 1117.0)


def initializeThrusters():
  pwm.start(7.2)
  pwm2.start(7.2)

  sleep(3)

def cleanupThrusters():
  # Cleanup
  pwm.stop()
  pwm2.stop()
  GPIO.cleanup()

def cleanupGPS():
  ser.close()

def calibrateThrusters():
  raw_input("Unplug the ESCs... Then press enter...")
  pwm.start(10.0)
  pwm2.start(10.0)
  raw_input("Plug in the ESCs... Wait for a beep... Then press enter...")
  pwm.ChangeDutyCycle(7.2)
  pwm2.ChangeDutyCycle(7.2)
  raw_input("Wait for a beep... Then press enter...")
  pwm.ChangeDutyCycle(5.0)
  pwm2.ChangeDutyCycle(5.0)
  raw_input("Unplug the ESCs.. Then press enter...")
  sys.exit()


def send_command(gps_serial, command):
    """
    Send a command to the GPS module over serial.
    Args:
        gps_serial: Serial connection to the GPS module.
        command: Command string to send (NMEA command).
    """
    gps_serial.write(command.encode('ascii'))
    time.sleep(0.1)  # Give the GPS module time to process the command

def setGPSPollingRate():
  gps_serial = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
  print("Connected to GPS module.")

  # Command to set the update rate to 10 Hz (100 ms)
  update_rate_command = "$PMTK220,100*2F\r\n"

  # Command to set the position fix interval to 100 ms
  fix_rate_command = "$PMTK300,100,0,0,0,0*2C\r\n"

  print("Setting polling rate to 10 Hz...")
  send_command(gps_serial, update_rate_command)
  send_command(gps_serial, fix_rate_command)

  print("Polling rate maximized to 10 Hz. Verifying...")
  while True:
      # Read GPS output to confirm new rate
      data = gps_serial.readline().decode('ascii', errors='ignore')
      if data:
          print(data.strip())

def readGPS():
 # Fetch the next report from gpsd
  report = session.next()

  # Check if the report contains a position fix
  if report['class'] == 'TPV':
    # Latitude and longitude may be missing if there's no fix
    latitude = getattr(report, 'lat', 'N/A')
    longitude = getattr(report, 'lon', 'N/A')
    altitude = getattr(report, 'alt', 'N/A')
    speed = getattr(report, 'speed', 'N/A')

    # print("Latitude: ", latitude, "Longitude: ", longitude)
    # print("Altitude: ", altitude, " m, Speed: ", speed, " m/s")
    return (1, latitude, longitude)
  return (0, 0, 0)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two points.
    Arguments:
        lat1, lon1: Latitude and Longitude of the first point (in decimal degrees).
        lat2, lon2: Latitude and Longitude of the second point (in decimal degrees).
    Returns:
        Bearing in degrees (0 = North, 90 = East, etc.).
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    
    initial_bearing = math.atan2(x, y)
    
    return initial_bearing

def getSmoothAzimuth():
  total = 0
  for i in range(10):
    sensor.read()
    print("Azimuth: " + str(math.radians(sensor.getAzimuth())))
    total = total + (math.radians(sensor.getAzimuth()) / 10)
    sleep(0.01)
  return total

def getAzimuth():
  sensor.read()
  return math.radians(sensor.getAzimuth())

def thrusts(theta_diff):
  thrust1 = 7.2 + (2.5 * np.sin(theta_diff))
  thrust2 = (7.2 + (7.2 - thrust1))
  return thrust1, thrust2

def thruster_control(xd, yd, gains, x_in, y_in, theta_in):
    # Unpack g_in values
    # x_in, y_in, theta_in = g_in

    # fraction = gains[5]  # Gains fraction (6th element in MATLAB, 5th in Python)

    # # Moving target
    # o = np.array([xb, yb])  # Origin of the vector
    # r = np.array([xd, yd])  # Reference point of the vector
    # p = np.array([x_in, y_in])  # Point to project

    # v = r - o
    # w = p - o

    # t = np.dot(w, v) / np.dot(v, v)
    # pxy = o + t * v

    # moving_target = r * fraction + pxy * (1 - fraction)

    # xdmove, ydmove = moving_target

    theta_d = theta_in

    alpha = theta_d # theta_in - theta_d

    if alpha % (2.0 * np.pi) > np.pi:
        alphamag = 2.0 * np.pi - alpha % (2.0 * np.pi)
    else:
        alphamag = alpha % (2.0 * np.pi)

    if np.sin(alpha) >= 0:
        theta_des = theta_in - alphamag
    else:
        theta_des = theta_in + alphamag

    rdiff = np.sqrt((xd - x_in) ** 2.0 + (yd - y_in) ** 2.0)

    thetadiff = theta_des - theta_in

    theta_go = gains[4]
    if abs(thetadiff) > theta_go:
        thrust = 0.0
    else:
        thrust = theta_go - abs(thetadiff) + rdiff * gains[7]  # Gains index 8 in MATLAB -> 7 in Python

    f_u = np.dot(np.array([[gains[0], 0.0], [0.0, gains[1]]]), np.array([thrust, thetadiff]))
    print("f_u" + str(f_u))
    print("f_u" + str(f_u[0]) + " " + str(f_u[1]))
    g_u = np.array([0, 0])

    # g_u[0] = 1.0/2.0 * f_u[0] - 1.0/2.0 * f_u[1]
    # g_u[1] = 1.0/2.0 * f_u[0] + 1.0/2.0 * f_u[1]
    g_u = np.dot(np.array([[1.0 / 2.0, -1.0 / 2.0], [1.0 / 2.0, 1.0 / 2.0]]), f_u)
    print("g_u" + str(g_u))
    g_u[0] = np.sign(g_u[0]) * 10 if abs(g_u[0]) > 10 else g_u[0]
    g_u[1] = np.sign(g_u[1]) * 10 if abs(g_u[1]) > 10 else g_u[1]

    pwm = g_u * (9.7-7.2)/10.0 + np.array([7.2, 7.2])

    

    return pwm

#target: 32.23105811654229 -110.95725426345098

def main():
  initializeThrusters()
  print("Thrusters initialized.")

  if 1 == 0:
    calibrateThrusters()

  prevPWM = (7.2, 7.2)
  while True:
    # pastGPSPositions.append(readGPS())
    currentTheta = getSmoothAzimuth()
    # print("Current theta: " + str(math.degrees(currentTheta)))
    
    success, lat, lon = readGPS()
    if(success):
      # print("APPENDING SUCCESS")
      pastGPSPositions.append((lat, lon))

    # print("Current GPS: ", lat, lon)
    # print("Current: ", math.degrees(currentTheta))
    thetaToTarget = calculate_bearing(targetCoordinates[0], targetCoordinates[1], pastGPSPositions[-1][0], pastGPSPositions[-1][1])
    # print("To target: ", math.degrees(thetaToTarget))
    thetaDiff = thetaToTarget - currentTheta + math.radians(115)
    print("Difference: ", math.degrees(thetaDiff))
    
    
    # gains = [forcegain, torquegain, stopnum, dampgain, theta_go, fractiontarget, intgain, distgain]
    gains = [7.0, 5.0, 0.0, 0.0, math.radians(45.0), 0.0, 0.0, 0.5]
    pwmOutputs = thruster_control(targetCoordinates[0], targetCoordinates[1], gains, pastGPSPositions[-1][0], pastGPSPositions[-1][1], thetaDiff)
    # pwmOutputs = thrusts(thetaDiff)
    # print("ThetaDiff: " + str(thetaDiff) + ", LEFT: " + str(pwmOutputs[0]) + ", RIGHT: " + str(pwmOutputs[1]))
    print("PWM Outputs: " + str(pwmOutputs))

    for i in range(1, 20):
      # pwm.ChangeDutyCycle(pwmOutputs[0])
      # pwm2.ChangeDutyCycle(pwmOutputs[1])

      pwm.ChangeDutyCycle(pwmOutputs[0] * float(i) / 20.0 + prevPWM[0] * (20.0 - float(i)) / 20.0)
      pwm2.ChangeDutyCycle(pwmOutputs[1] * float(i) / 20.0 + prevPWM[1] * (20.0 - float(i)) / 20.0)
      sleep(0.05)

    prevPWM = (pwmOutputs[0], pwmOutputs[1])

  cleanupThrusters()
  cleanupGPS()

if __name__ == '__main__':

  n = len(sys.argv)
  print("Total arguments passed:", n)

  # Arguments passed
  print("\nName of Python script:", sys.argv[0])

  print("\nArguments passed:")
  print(sys.argv[1], sys.argv[2])

  targetCoordinates = (float(sys.argv[1]), float(sys.argv[2]))
  print(targetCoordinates[0], targetCoordinates[1])

  main()