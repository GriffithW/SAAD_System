import gps

def readGPS():
  
  session = gps.gps(mode=gps.WATCH_ENABLE)
 # Fetch the next report from gpsd
  report = session.next()
  #print('h')
  # Check if the report contains a position fix
  if report['class'] == 'TPV':
    # Latitude and longitude may be missing if there's no fix
    latitude = getattr(report, 'lat', 'N/A')
    longitude = getattr(report, 'lon', 'N/A')
    altitude = getattr(report, 'alt', 'N/A')
    speed = getattr(report, 'speed', 'N/A')

    print("Latitude: ", latitude, "Longitude: ", longitude)
    print("Altitude: ", altitude, " m, Speed: ", speed, " m/s")
    return (1, latitude, longitude)
  else:
    print("no")
  return (0, 0, 0)


if __name__ == '__main__':
  
  while True:
    readGPS()  
  main()