from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Quaternion

# Use this script for empirical calculation of robot to camera transform
# for each pose sensor configuration using fixed target measured
# from robot and targetPose averages from PhotonVision.

# Steps:
# 1) Place the robot in a fixed position with the target camera facing
# a single AprilTag target with the center of robot precisely aligned
# with the center of the AprilTag (so that the Y translation below is zero).
# Best approach is to create a rectangle on the floor with gaffer tape
# with the robot frame parallel on one side and the AprilTag (mounted)
# on the other side to ensure the two are parallel.

# 2) Accurately measure the distance from the center of the robot to
# the face of the AprilTag to set the X translation value below.

# 3) Accurately measure the distance from the floor (base of the roobt)
# to the center of the AprilTag to set the Z translation value below.

# 4) Using PhotonVision web UI, confirm that the target AprilTag is being
# detected and the target translation values are being reported correctly.

# 5) Using AdvantageScope, connect to the robot and expand the photonvision
# topic, then expand the camera name being calculated, then expand to the
# "targetPose", and finally expand both translation and rotation/quaternion.
# Create a new "Statistics" panel in AdvantageScope with an empty "Measurements"
# section. Under both translation and rotation sections under targetPose, drag each
# individual value into the Measurements section so that the calculation of
# median averages can begin. Wait at least 30 seconds for the Median value for each
# translation (x, y, z) and rotation (w, x, y, z) property to settle. Use each of
# those Median values to set the appropriate constant value in the script
# below for target to camera transform calculation.

# 6) After all of the script constant values have been set, save the script
# and open a new Terminal window in VSCode (do not commit the script changes in git).
# After the Python virtual environment activates, copy, paste, and execute the
# script with the following command:
# python .\lib\scripts\calculateRobotToCameraTransform.py

# 7) Copy the Transform3d value that is printed to the terminal console and
# paste it into the appropriate place in the robot constants for the associated
# pose sensor configuration being calculated here.

# Example output from example values below:
# Transform3d(Translation3d(0.149506, -0.055318, 0.271137), Rotation3d(-0.001852, -0.181301, 0.020370))

# 8) Repeat this same process from step 1 above for all other pose sensor cameras on the robot.

# Note: the constant values below are for example only - see the steps above for setting real/correct values.

TARGET_TO_ROBOT_TRANSLATION_X: units.meters = units.inchesToMeters(31.75 - (5.0 / 16.0) + (27.0 / 2.0) + 0.25) # distance from center of robot to face of AprilTag
TARGET_TO_ROBOT_TRANSLATION_Y: units.meters = 0.0 # distance from center of robot to center of AprilTag (should be zero using alignment method above)
TARGET_TO_ROBOT_TRANSLATION_Z: units.meters = units.inchesToMeters(-44.25) # distance from center of AprilTag to the floor (will be negative value as the robot is below the AprilTag)
TARGET_TO_ROBOT_ROTATION_YAW: units.degrees = -90.0 # rotation of robot to AprilTag (e.g. front of robot turned to face toward AprilTag is 180 degrees, rear of robot facing tag will be 0 degrees, sides will be -90/90 accordingly)

TARGET_TO_CAMERA_TRANSLATION_X = 0.922 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_TRANSLATION_Y = 0.423 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_TRANSLATION_Z = 0.422 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_W = 0.375 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_X = -0.149 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_Y = 0.083 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_Z = -0.911 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics

# ===========================================================================

targetToRobot = Transform3d(
    Translation3d(TARGET_TO_ROBOT_TRANSLATION_X, TARGET_TO_ROBOT_TRANSLATION_Y, TARGET_TO_ROBOT_TRANSLATION_Z),
    Rotation3d().fromDegrees(0, 0, TARGET_TO_ROBOT_ROTATION_YAW)
)

targetToCamera = Transform3d(
    Translation3d(TARGET_TO_CAMERA_TRANSLATION_X, TARGET_TO_CAMERA_TRANSLATION_Y, TARGET_TO_CAMERA_TRANSLATION_Z),
    Rotation3d(Quaternion(TARGET_TO_CAMERA_QUATERNION_W, TARGET_TO_CAMERA_QUATERNION_X, TARGET_TO_CAMERA_QUATERNION_Y, TARGET_TO_CAMERA_QUATERNION_Z)))

robotToCamera = Pose3d().transformBy(targetToCamera.inverse()) - Pose3d().transformBy(targetToRobot)

constant = str(robotToCamera).replace("x=", "").replace("y=", "").replace("z=", "")

print(constant)
