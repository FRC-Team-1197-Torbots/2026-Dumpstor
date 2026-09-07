package frc.robot.Constants;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class VisionConstants {
    // Replace with your actual camera names as configured in the PhotonVision UI
    public static final String CAMERA_1_NAME = "Front_Left_Camera";
    public static final String CAMERA_2_NAME = "Front_Right_Camera";
    public static final String CAMERA_3_NAME = "Back_Left_Camera";
    public static final String CAMERA_4_NAME = "Back_Right_Camera";

    // Robot to camera transforms
    // These need to be accurately measured on the physical robot.
    // Measurements are in meters and radians.
    // X is forward, Y is left, Z is up relative to the robot center.
    public static final Transform3d ROBOT_TO_CAM_1 = new Transform3d(
        new Translation3d(0.3, 0.3, 0.5), new Rotation3d(0, 0, 0)
    );
    public static final Transform3d ROBOT_TO_CAM_2 = new Transform3d(
        new Translation3d(0.3, -0.3, 0.5), new Rotation3d(0, 0, 0)
    );
    public static final Transform3d ROBOT_TO_CAM_3 = new Transform3d(
        new Translation3d(-0.3, 0.3, 0.5), new Rotation3d(0, 0, Math.PI)
    );
    public static final Transform3d ROBOT_TO_CAM_4 = new Transform3d(
        new Translation3d(-0.3, -0.3, 0.5), new Rotation3d(0, 0, Math.PI)
    );
}
