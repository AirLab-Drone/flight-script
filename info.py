def connectCheck(vehicle):
    # Get some vehicle attributes (state)
    print("Get some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)    # settable

def NEDLocation(vehicle):
    print("local NED frame: %s" % vehicle.location.local_frame)

def speed(vehicle):
    print("Groundspeed: %s" % vehicle.groundspeed)    # settable
    print("Airspeed: %s" % vehicle.airspeed)    # settable