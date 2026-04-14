package frc.robot.subsystems.aux;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;

@SuppressWarnings("removal")
public class Intake extends SubsystemBase {
  boolean run;

  public DigitalInput irSensor = new DigitalInput(1);
  SparkMax leftHotwheel =
      new SparkMax(Constants.IntakeConstants.LEFT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkMax rightHotwheel =
      new SparkMax(Constants.IntakeConstants.RIGHT_HOTWHEEL, SparkMax.MotorType.kBrushless);
  SparkFlex groundPickup =
      new SparkFlex(Constants.IntakeConstants.GROUND_PICKUP, SparkFlex.MotorType.kBrushless);
  SparkMax loadingDrum =
      new SparkMax(Constants.ShooterConstants.LOADING_DRUM, SparkMax.MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    SparkFlexConfig pickupConfig = new SparkFlexConfig();
    pickupConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    loadingDrum.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightHotwheel.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    groundPickup.configure(
        pickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    groundPickup.set(Constants.PowerConstants.INTAKE_POWER);
    leftHotwheel.set(Constants.PowerConstants.INTAKE_POWER);
    rightHotwheel.set(-Constants.PowerConstants.INTAKE_POWER);
    loadingDrum.set(Constants.PowerConstants.DRUM_POWER);
  }

  public static double INTAKE_RPM = 60;

  public static void initDashboard() {
    SmartDashboard.putNumber("Intake/Target RPM", INTAKE_RPM);
  }

  public static void updateFromDashboard() {
    double val = SmartDashboard.getNumber("Intake/Target RPM", INTAKE_RPM);
    INTAKE_RPM = (double) Math.round(val);
  }

  public void intakeRPM() {
    SparkClosedLoopController pickupPIDController = groundPickup.getClosedLoopController();
    SparkClosedLoopController leftHotwheelPIDController = leftHotwheel.getClosedLoopController();
    SparkClosedLoopController rightHotwheelController = rightHotwheel.getClosedLoopController();
    pickupPIDController.setSetpoint(60, ControlType.kVelocity);
    leftHotwheelPIDController.setSetpoint(60, ControlType.kVelocity);
    rightHotwheelController.setSetpoint(60, ControlType.kVelocity);
  }

  public void autoIntake() {
    run = true;
  }

  public void updateAutoIntake() {
    if (run) {
      if (irSensor.get()) {
        loadingDrum.set(PowerConstants.DRUM_POWER);
        groundPickup.set(PowerConstants.INTAKE_POWER);
        leftHotwheel.set(PowerConstants.INTAKE_POWER);
        rightHotwheel.set(-PowerConstants.INTAKE_POWER);
      } else {
        loadingDrum.set(0.0);
        groundPickup.set(0.0);
        leftHotwheel.set(0.0);
        rightHotwheel.set(0.0);
      }
    }
  }

  public void stopIntake() {
    loadingDrum.set(0.0);
    groundPickup.set(0.0);
    leftHotwheel.set(0.0);
    rightHotwheel.set(0.0);
    run = false;
  }

  public Command autoIntakeCommand() {
    return this.run(() -> autoIntake());
  }

  public void feedMotorPower(double power) {
    loadingDrum.set(power);
  }

  public void intakeMotorPower(double power) {
    groundPickup.set(power);
    leftHotwheel.set(power);
    rightHotwheel.set(-power);
  }

  public void stopIntakeAndFeed() {
    loadingDrum.set(0.0);
    groundPickup.set(0.0);
    leftHotwheel.set(0.0);
    rightHotwheel.set(0.0);
    run = false;
  }

  public void drumShoot() {
    loadingDrum.set(PowerConstants.SHOOTER_POWER);
  }

  public Command drumShootCommand() {
    return this.run(() -> drumShoot());
  }

  public void stopDrumShoot() {
    loadingDrum.set(0);
  }

  public Command stopDrumShootCommand() {
    return this.run(() -> stopDrumShoot());
  }

  public Command stopIntakeCommand() {
    return this.run(() -> stopIntake());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Run Teleop Intake", run);
    SmartDashboard.putBoolean("Note In Shooter", !irSensor.get());
    if (RobotState.isTeleop()) {
      updateAutoIntake();
    }
  }
}
