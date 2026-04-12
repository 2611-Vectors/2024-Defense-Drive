package frc.robot.subsystems.aux;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("removal")
public class Intake extends SubsystemBase {
  boolean run;

  public DigitalInput irSensor = new DigitalInput(1);
  SparkMax loadingDrum =
      new SparkMax(Constants.IntakeConstants.LOADING_DRUM, SparkMax.MotorType.kBrushless);
  SparkMax leftHotwheel =
      new SparkMax(Constants.IntakeConstants.LEFT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkMax rightHotwheel =
      new SparkMax(Constants.IntakeConstants.RIGHT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkFlex groundPickup =
      new SparkFlex(Constants.IntakeConstants.GROUND_PICKUP, SparkFlex.MotorType.kBrushless);

  boolean robotRunning;

  public Intake() {
    SparkMaxConfig drumConfig = new SparkMaxConfig();
    drumConfig.idleMode(IdleMode.kBrake);
    loadingDrum.configure(
        drumConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drumPower(double power) {
    loadingDrum.set(power);
  }

  public void intakeMotorPower(double power) {
    groundPickup.set(power);
    leftHotwheel.set(power);
    rightHotwheel.set(-power);
  }

  public void autoIntake() {
    run = true;
  }

  public void updateAutoIntake() {
    if (run) {
      if (irSensor.get()) {
        loadingDrum.set(Constants.PowerConstants.FEED_POWER);
        groundPickup.set(Constants.PowerConstants.INTAKE_POWER);
        leftHotwheel.set(Constants.PowerConstants.INTAKE_POWER);
        rightHotwheel.set(-Constants.PowerConstants.INTAKE_POWER);
      } else {
        loadingDrum.set(0.0);
        groundPickup.set(0.0);
        leftHotwheel.set(0.0);
        rightHotwheel.set(0.0);
      }
    }
  }

  public void stopIntakeAndFeed() {
    loadingDrum.set(0.0);
    groundPickup.set(0.0);
    leftHotwheel.set(0.0);
    rightHotwheel.set(0.0);
    run = false;
  }

  public Command intakeCommand() {
    return this.run(() -> updateAutoIntake());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Shooter", !irSensor.get());
    SmartDashboard.putBoolean("Run Teleop Intake", run);
    if (RobotState.isTeleop()) {
      updateAutoIntake();
    }
  }
}
