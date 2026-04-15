/**************************************************************************
 * All PID stuff needs to be comlpetely reworked to the new 2025+ RevLIB API
 **************************************************************************/

package frc.robot.subsystems.aux;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

@SuppressWarnings("removal")
public class Shooter extends SubsystemBase {
  SparkFlex topShooter =
      new SparkFlex(Constants.ShooterConstants.TOP_SHOOTER, SparkFlex.MotorType.kBrushless);
  SparkFlex bottomShooter =
      new SparkFlex(Constants.ShooterConstants.BOTTOM_SHOOTER, SparkFlex.MotorType.kBrushless);

  SparkFlexConfig shooterConfig = new SparkFlexConfig();
  // Shooter PID tuning
  public Shooter() {
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    topShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopShooter() {
    topShooter.set(0);
    bottomShooter.set(0);
  }

  public void shoot() {
    topShooter.set(PowerConstants.SHOOTER_POWER);
    bottomShooter.set(-PowerConstants.SHOOTER_POWER);
  }

  public void dumpShoot() {
    topShooter.set(-PowerConstants.SHOOTER_POWER);
    bottomShooter.set(PowerConstants.SHOOTER_POWER);
  }

  public Command dumpShootCommand() {
    return this.run(() -> dumpShoot());
  }

  public Command shootCommand() {
    return this.run(() -> shoot());
  }

  public Command stopShootCommand() {
    return this.run(() -> stopShooter());
  }
}
