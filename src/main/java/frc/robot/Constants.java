package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static double MAX_SPEED = 5.2;
  public static double NathanSpeed = 2.0;
  public static final double TILT_ANGLE_OFFSET = -25;
  public static final double APRIL_TAG_OFFSET = 1.01237005816;
  public static final int TEAMSWITCH_ID = 2;

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

  public final class ClimbConstants {
    public static final int RETIRED_CLIMB_A = 61;
    public static final int RETIRED_CLIMB_B = 62;
  }

  public final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public final class PowerConstants {
    public static final double INTAKE_POWER = 0.8;
    public static final double DRUM_POWER = 0.7;
    public static final double MAX_TILT_POWER = 1;
    public static final double SHOOTER_POWER = 1;
  }

  public static class DashboardConstants {
    public static double SHOOTER_TIMEOUT = 3;

    // Shooter PID tuning - start with these values and adjust on dashboard
    // P: Proportional gain - increase if response is slow, decrease if oscillating
    // I: Integral gain - helps eliminate steady-state error (usually 0 or very small)
    // D: Derivative gain - adds damping to prevent overshoot
    // FF: Feedforward - critical for velocity control, roughly 12V / max_rpm
    public static double P = 0.0001;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double FF = 0.00024; // Adjust based on your shooter max RPM

    public static double SHOOTER_RPM = 60;

    public static void initDashboard() {
      SmartDashboard.putNumber("Shooter/Shooter Timeout", SHOOTER_TIMEOUT);

      SmartDashboard.putNumber("Shooter/Shooter PID/P", P);
      SmartDashboard.putNumber("Shooter/Shooter PID/I", I);
      SmartDashboard.putNumber("Shooter/Shooter PID/D", D);
      SmartDashboard.putNumber("Shooter/Shooter PID/ FF", FF);

      SmartDashboard.putNumber("Shooter/Shooter RPM", SHOOTER_RPM);
    }

    public static void updateFromDashboard() {
      double dashboardTimeout =
          SmartDashboard.getNumber("Shooter/Shooter Timeout", SHOOTER_TIMEOUT);
      SHOOTER_TIMEOUT = Math.round(dashboardTimeout);

      double dashboardP = SmartDashboard.getNumber("Shooter/Shooter PID/P", P);
      double dashboardI = SmartDashboard.getNumber("Shooter/Shooter PID/I", I);
      double dashboardD = SmartDashboard.getNumber("Shooter/Shooter PID/D", D);
      double dashboardFF = SmartDashboard.getNumber("Shooter/Shooter PID/ FF", FF);
      P = dashboardP;
      D = dashboardD;
      I = dashboardI;
      FF = dashboardFF;

      double dashboardRPM = SmartDashboard.getNumber("Shooter/Shooter RPM", SHOOTER_RPM);
      SHOOTER_RPM = Math.round(dashboardRPM);
    }
  }

  public static class VisionConstants {
    // AprilTag Field Layout (copied from 2026-Beta-Bot)
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double FIELD_WIDTH = 16.541;
    public static final double FIELD_HEIGHT = 8.069;

    // No cams yet, these are placeholders
    public static final String leftCam = "Camera1";
    public static final Transform3d robotToLeftCam =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

    public static final String rightCam = "Camera2";
    public static final Transform3d robotToRightCam =
        new Transform3d(
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
