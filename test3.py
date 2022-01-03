# take off, do a circle, rtl

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import apriltag
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("--connect", default="127.0.0.1:14551")
args = parser.parse_args()
cap = cv2.VideoCapture(0)

# Connect to the Vehicle
print("Connecting to vehicle on: %s" % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)
print("Take off complete\n")

# time.sleep()

while True:
    _, img = cap.read()
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = apriltag.Detector().detect(img_gray)
    cv2.imshow("frame", img)

    @vehicle.on_message("MISSION_ITEM_REACHED")
    def listener(self, name, message):
        print(message.seq)
        if message.seq == 19 and vehicle.mode != "GUIDED":
            #vehicle.mode = VehicleMode("GUIDED")
            print(type(vehicle.mode))

    if res:
        tag = res[0][1]
        print(tag)

        if tag == 12 and (vehicle.mode != VehicleMode("AUTO")):
            print("Auto\n")
            vehicle.mode = VehicleMode("AUTO")

        elif tag == 42 and vehicle.mode != VehicleMode("RTL"):
            print("Return to Launch\n")
            vehicle.parameters["RTL_ALT"] = 0
            vehicle.mode = VehicleMode("RTL")
            break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        cap.release()

        print("Return to launch\n")
        vehicle.parameters["RTL_ALT"] = 0
        vehicle.mode = VehicleMode("RTL")
        break

cv2.destroyAllWindows()
cap.release()
# Close vehicle object
vehicle.close()
