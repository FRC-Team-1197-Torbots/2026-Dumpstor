package frc.robot.Constants;

public final class FieldConstants {
        public static final double FieldLength = 16.541;
        public static final double FieldWidth = 8.211;

        // Passing Targets (Aim Points near Feeder Station / Corners)
        // Offset by ~1.5m to ensure we don't shoot off the field
        public static final double PassingMargin = 1.5;

        // 2026 Manual: 156.61 inches
        public static final double BlueAllianceLineX = org.wpilib.math.util.Units.inchesToMeters(156.61);
        public static final double RedAllianceLineX = FieldLength - BlueAllianceLineX;
        // Based on analysis: Original code had X/Y swapped relative to comments.
        // Comment: X=182.1" (4.62m), Y=159.1" (4.04m).
        // Y=4.04m is roughly CENTER field width (8.2m total width).

        // Blue Target (The one on the Blue Side)
        // Note: In some games you shoot at your OWN side (Tower?), in others OPPOSITE
        // (Speaker).
        // Assuming X=4.62 is the specific target location:
        public static final org.wpilib.math.geometry.Pose2d BlueTargetPose = new org.wpilib.math.geometry.Pose2d(
                4.62, 4.035, org.wpilib.math.geometry.Rotation2d.fromDegrees(0));

        // Red Target (Mirrored / Rotated)
        // Standard Field Length approx 16.54m (54ft ish)
        // If Rotational Symmetry (180 deg rotation around center):
        // Red X = FieldLength - Blue X = 16.54 - 4.6228 = 11.9172
        // Red Y = FieldWidth - Blue Y = 8.21 - 4.0386 = 4.17
        public static final org.wpilib.math.geometry.Pose2d RedTargetPose = new org.wpilib.math.geometry.Pose2d(
                16.54105 - 4.62, 4.035, org.wpilib.math.geometry.Rotation2d.fromDegrees(180));

    }
