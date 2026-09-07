package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

import frc.robot.Constants.VisionConstants;

public class Vision extends org.wpilib.command2.SubsystemBase {
    private final PhotonCamera cam1;
    private final PhotonCamera cam2;
    private final PhotonCamera cam3;
    private final PhotonCamera cam4;

    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;
    private final PhotonPoseEstimator poseEstimator3;
    private final PhotonPoseEstimator poseEstimator4;

    private org.wpilib.vision.apriltag.AprilTagFieldLayout fieldLayout;

    public Vision() {
        // Initialize cameras
        cam1 = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
        cam2 = new PhotonCamera(VisionConstants.CAMERA_2_NAME);
        cam3 = new PhotonCamera(VisionConstants.CAMERA_3_NAME);
        cam4 = new PhotonCamera(VisionConstants.CAMERA_4_NAME);

        // Load the field layout for the 2026 game using the updated v2027 method
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag field layout! Vision pose estimation will not work.");
            e.printStackTrace();
        }

        // Initialize pose estimators for each camera. v2027 removes PoseStrategy from constructor.
        poseEstimator1 = new PhotonPoseEstimator(fieldLayout, VisionConstants.ROBOT_TO_CAM_1);
        poseEstimator2 = new PhotonPoseEstimator(fieldLayout, VisionConstants.ROBOT_TO_CAM_2);
        poseEstimator3 = new PhotonPoseEstimator(fieldLayout, VisionConstants.ROBOT_TO_CAM_3);
        poseEstimator4 = new PhotonPoseEstimator(fieldLayout, VisionConstants.ROBOT_TO_CAM_4);
    }

    /**
     * Gets the latest estimated robot pose from a specific camera.
     * 
     * =========================================================================
     * INTEGRATION INSTRUCTIONS (Once physical cameras are hooked up):
     * =========================================================================
     * 1. In your Drivetrain subsystem (e.g., CommandSwerveDrivetrain), you likely have a
     *    SwerveDrivePoseEstimator that tracks your odometry.
     * 2. Inside the periodic() method of your Drivetrain, you should loop through
     *    each camera (1 through 4) and call this method.
     * 3. For each camera, if getEstimatedGlobalPose returns an Optional that is present:
     *    a. Extract the result: EstimatedRobotPose visionPose = result.get();
     *    b. Feed it to your drivetrain pose estimator:
     *       poseEstimator.addVisionMeasurement(
     *           visionPose.estimatedPose.toPose2d(),
     *           visionPose.timestampSeconds
     *       );
     * 4. Optional but Recommended: Before adding the measurement, check the ambiguity 
     *    or distance to the tags (accessible via visionPose.targetsUsed) and ignore 
     *    frames that are too noisy or far away.
     * =========================================================================
     *
     * @param cameraIndex The index of the camera (1 to 4)
     * @return An Optional containing the EstimatedRobotPose if a valid target was seen, or empty if not.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int cameraIndex) {
        if (fieldLayout == null) return Optional.empty();

        PhotonCamera cam;
        PhotonPoseEstimator poseEstimator;

        switch (cameraIndex) {
            case 1: cam = cam1; poseEstimator = poseEstimator1; break;
            case 2: cam = cam2; poseEstimator = poseEstimator2; break;
            case 3: cam = cam3; poseEstimator = poseEstimator3; break;
            case 4: cam = cam4; poseEstimator = poseEstimator4; break;
            default: return Optional.empty();
        }

        var results = cam.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        // Return the pose estimation from the most recent frame in the queue
        return poseEstimator.estimateCoprocMultiTagPose(results.get(results.size() - 1));
    }

    @Override
    public void periodic() {
        // Any periodic vision logging (e.g. to SmartDashboard) can go here.
        // The actual pose integration will happen in the Drivetrain subsystem by polling getEstimatedGlobalPose()
    }
}
