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
  response sensors.SetSensor("Altitude", 600.0)
  response sensors.SetSensor("ClimbRate", 0.0)
  response sensors.SetSensor("GroundTrack", 180.0)
  response sensors.SetSensor("GroundSpeed", 1.0)

step
 condition self.TimeSinceEntry() > 1.5
 post_assertions
  assert abs(control["RightTilt"] - control["LeftTilt"]) <.01
  assert control["ForwardEngineReleaseControl"] == 0
  assert control["VTOLEngineReleaseControl"] == 0
 responses
  response sensors.SetSensor("Pitch", -1.0)
