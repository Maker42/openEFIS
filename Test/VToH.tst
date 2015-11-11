steps
step
 condition True
 responses
  response sensors.SetSensor("Altitude", 10.0),
  response sensors.SetSensor("Heading", 0.0)
  response sensors.SetSensor("Roll", 0.0)
  response sensors.SetSensor("RollRate", 0.0)
  response sensors.SetSensor("Pitch", 0.0)
  response sensors.SetSensor("PitchRate", 0.0)
  response sensors.SetSensor("Yaw", 0.0)
  response sensors.SetSensor("AirSpeed", 0.0)
  response sensors.SetSensor("GroundSpeed", 0.0)
  response sensors.SetSensor("ClimbRate", 0.0)
  response sensors.SetSensor("Longitude", -107.0)
  response sensors.SetSensor("Latitude", 40.0)
  response sensors.SetSensor("MagneticDeclination", -10.0)
  response sensors.SetSensor("TrueHeading", 350.0)
  response sensors.SetSensor("GroundTrack", 0.0)
  response sensors.SetSensor("Battery", 100.0)
  response sensors.SetSensor("OuterEnginePosition", "upward")

step
 condition control["Throttle"] >= 0.8 and self.TimeSinceEntry() > 0.5
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 5.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)

step
 condition control["RightTilt"] >= 5.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 10.0)
  response sensors.SetSensor("Altitude", 1990.0)
  response sensors.SetSensor("ClimbRate", -100.0)


step
 condition control["RightTilt"] >= 10.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .8
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 20.0)
  response sensors.SetSensor("Altitude", 1995.0)
  response sensors.SetSensor("ClimbRate", 50.0)


step
 condition control["RightTilt"] >= 15.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 25.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 20.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 30.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 25.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 35.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 30.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 40.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 35.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 45.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 45.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 50.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 50.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 53.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 55.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
 responses
  response sensors.SetSensor("AirSpeed", 56.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 60.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 59.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 65.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 62.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 70.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 65.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 75.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 68.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 80.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 71.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] >= 85.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] > .7
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 73.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["RightTilt"] == 90.0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["ForwardEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 74.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)


step
 condition control["VTOLEngineReleaseControl"] == 1
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["Throttle"] <= .1
  assert control["ForwardEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 79.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)
  response sensors.SetSensor("OuterEnginePosition", "forward")

step
 condition control["VTOLEngineReleaseControl"] == 0
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["ForwardEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 89.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)

step
 condition control["Throttle"] == 1
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["ForwardEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("AirSpeed", 139.0)
  response sensors.SetSensor("Altitude", 2000.0)
  response sensors.SetSensor("ClimbRate", 0.0)

