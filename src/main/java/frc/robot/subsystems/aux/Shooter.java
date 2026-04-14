/**************************************************************************
 * All PID stuff needs to be comlpetely reworked to the new 2025+ RevLIB API
 **************************************************************************/

package frc.robot.subsystems.aux;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

@SuppressWarnings("removal")
public class Shooter extends SubsystemBase {
  SparkMax topShooter =
      new SparkMax(Constants.ShooterConstants.TOP_SHOOTER, SparkMax.MotorType.kBrushless);
  SparkMax bottomShooter =
      new SparkMax(Constants.ShooterConstants.BOTTOM_SHOOTER, SparkMax.MotorType.kBrushless);

  SparkMaxConfig shooterPIDConfig = new SparkMaxConfig();
  // Shooter PID tuning
  public Shooter() {
    shooterPIDConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0006, 0.000001, 0.0000015)
        .velocityFF(0.0);

    topShooter.configure(
        shooterPIDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterPIDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopShooter() {
    topShooter.setVoltage(0);
    bottomShooter.setVoltage(0);
  }

  public static int SHOOTER_RPM = 60;

  public static void initDashboard() {
    SmartDashboard.putNumber("Shooter/TargetRPM", SHOOTER_RPM);
  }

  public static void updateFromDashboard() {
    double val = SmartDashboard.getNumber("Shooter/TargetRPM", SHOOTER_RPM);
    SHOOTER_RPM = (int) Math.round(val);
  }

  public void RPMshoot() {
    SparkClosedLoopController topShooterPIDController = topShooter.getClosedLoopController();
    SparkClosedLoopController bottomShooterPIDController = bottomShooter.getClosedLoopController();
    topShooterPIDController.setSetpoint(SHOOTER_RPM, ControlType.kVelocity);
    bottomShooterPIDController.setSetpoint(SHOOTER_RPM, ControlType.kVelocity);
  }

  public void shoot() {
    topShooter.setVoltage(PowerConstants.SHOOTER_POWER);
    bottomShooter.setVoltage(-PowerConstants.SHOOTER_POWER);
  }

  public void dumpShoot() {
    topShooter.setVoltage(-5);
    bottomShooter.setVoltage(5);
  }

  public Command dumpShootCommand() {
    return this.run(() -> dumpShoot());
  }

  public Command RPMshootCommand() {
    return this.run(() -> RPMshoot());
  }

  public Command shootCommand() {
    return this.run(() -> shoot());
  }
}
