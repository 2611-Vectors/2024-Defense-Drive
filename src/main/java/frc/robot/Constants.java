package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static double MAX_SPEED = 5.2;
    public static double NathanSpeed = 2.0;
    public static final double TILT_ANGLE_OFFSET = -25;

    public final class IntakeConstants {
        public static final int GROUND_PICKUP = 52;
        public static final int RIGHT_HOTWHEEL = 53;
        public static final int LEFT_HOTWHEEL = 54;
    }

    public final class ShooterConstants {
        public static final int TILT = 44;
        public static final int TOP_SHOOTER = 42;
        public static final int BOTTOM_SHOOTER = 41;
        public static final int LOADING_DRUM = 43;
    }

    public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER_ID = 0;
        public static final int OPERATOR_CONTROLLER_ID = 1;
        public static final LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(3.25);
        public static final AngularVelocity MAX_TURN_SPEED = DegreesPerSecond.of(270.0);
    }

    public final class PowerConstants {
        public static final double INTAKE_POWER = 0.8;
        public static final double DRUM_VELOCITY = 4000;
        public static final double MAX_TILT_POWER = 1;
        public static final double SHOOTER_VELOCITY = 1000;
    }

    public static class VisionConstants {
        // AprilTag Field Layout (copied from 2026-Beta-Bot)
        public static AprilTagFieldLayout aprilTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final double FIELD_WIDTH = 16.541;
        public static final double FIELD_HEIGHT = 8.069;

        // No cams yet, these are placeholders
        public static final String leftCam = "Camera1";
        public static final Transform3d robotToLeftCam = new Transform3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

        public static final String rightCam = "Camera2";
        public static final Transform3d robotToRightCam = new Transform3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

        public static final double MAX_AMBIGUITY = 0.2;
        public static final double MAX_Z_ERROR = 0.2;

        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.03; // Radians

        public static double[] cameraStdDevFactors = new double[] {1.0};

        public static double linearStdDevMegatag2Factor = 0.5;
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
    }

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
