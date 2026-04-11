package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static double MAX_SPEED = 5.2;
  public static double NathanSpeed = 2.0;
  public static PIDFF_CONSTANTS shooterPID = new PIDFF_CONSTANTS(0.0006, 0.000001, 0.0000015, 0.0);
  public static PIDFF_CONSTANTS ShooterAnglePID = new PIDFF_CONSTANTS(0.0, 0, 0, 0.0001);
  public static final double TILT_ANGLE_OFFSET = -25;
  public static final double APRIL_TAG_OFFSET = 1.01237005816;
  public static final int TEAMSWITCH_ID = 2;

  public final class IntakeConstants {
    public static final int LOADING_DRUM = 43;
    public static final int GROUND_PICKUP = 52;
    public static final int RIGHT_HOTWHEEL = 53;
    public static final int LEFT_HOTWHEEL = 54;
  }

  public final class ShooterConstants {
    public static final int TILT = 44;
    public static final int TOP_SHOOTER = 61;
    public static final int BOTTOM_SHOOTER = 62;
    public static final double MAX_TILT_POWER = 1;
  }

  public final class ClimbConstants {
    public static final int RETIRED_CLIMB_A = 41;
    public static final int RETIRED_CLIMB_B = 42;
  }

  public final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public final class PowerConstants {
    public static final double INTAKE_POWER = 0.8;
    public static final double FEED_POWER = 0.7;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class PIDFF_CONSTANTS {
    public double p, i, d, ff;

    PIDFF_CONSTANTS(double p, double i, double d, double ff) {
      this.p = p;
      this.i = i;
      this.d = d;
      this.ff = ff;
    }

    public double getP() {
      return p;
    }

    public double getI() {
      return i;
    }

    public double getD() {
      return d;
    }

    public double getFF() {
      return ff;
    }
  }
}
