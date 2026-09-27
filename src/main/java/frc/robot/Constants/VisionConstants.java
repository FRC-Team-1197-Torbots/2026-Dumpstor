package frc.robot.Constants;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class VisionConstants {
    // Replace with your actual camera names as configured in the PhotonVision UI
    public static final String CAMERA_1_NAME = "Front";
    public static final String CAMERA_2_NAME = "Right";
    public static final String CAMERA_3_NAME = "Left";
    public static final String CAMERA_4_NAME = "Back";

    // Robot to camera transforms
    // These need to be accurately measured on the physical robot.
    // Measurements are in meters and radians.
    // X is forward, Y is left, Z is up relative to the robot center.
    public static final Transform3d ROBOT_TO_CAM_1 = new Transform3d(
        new Translation3d(0, 0.0680851318, 0.529841079), new Rotation3d(Math.toRadians(18.025109), 0, 0)
    );
    public static final Transform3d ROBOT_TO_CAM_2 = new Transform3d(
        new Translation3d(0.421767, -0.244475, 0.5160704436), new Rotation3d(0, 0, Math.PI/2)
    );
    public static final Transform3d ROBOT_TO_CAM_3 = new Transform3d(
        new Translation3d(-0.421767, -0.244475, 0.516704436), new Rotation3d(0, 0, 3 * Math.PI / 2)
    );
    public static final Transform3d ROBOT_TO_CAM_4 = new Transform3d(
        new Translation3d(0, -0.2305671538, 0.3928973092), new Rotation3d(Math.toRadians(-27.718532), 0, Math.PI)
    );
}
