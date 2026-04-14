package frc.robot.subsystems.aux;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
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

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
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
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    loadingDrum.set(Constants.PowerConstants.DRUM_POWER);
    groundPickup.set(Constants.PowerConstants.INTAKE_POWER);
    leftHotwheel.set(Constants.PowerConstants.INTAKE_POWER);
    rightHotwheel.set(-Constants.PowerConstants.INTAKE_POWER);
  }

  public void stopIntake() {
    loadingDrum.set(0.0);
    groundPickup.set(0.0);
    leftHotwheel.set(0.0);
    rightHotwheel.set(0.0);
  }

  public void dumpIntake() {
    loadingDrum.set(-Constants.PowerConstants.DRUM_POWER);
    groundPickup.set(-Constants.PowerConstants.INTAKE_POWER);
    leftHotwheel.set(-Constants.PowerConstants.INTAKE_POWER);
    rightHotwheel.set(Constants.PowerConstants.INTAKE_POWER);
  }

  public static int INTAKE_RPM = 60;

  public static void initDashboard() {
    SmartDashboard.putNumber("Intake/Target RPM", INTAKE_RPM);
  }

  public static void updateFromDashboard() {
    double val = SmartDashboard.getNumber("Intake/Target RPM", INTAKE_RPM);
    INTAKE_RPM = (int) Math.round(val);
  }

  public void intakeRPM() {
    SparkClosedLoopController drumPIDController = loadingDrum.getClosedLoopController();
    SparkClosedLoopController pickupPIDController = groundPickup.getClosedLoopController();
    SparkClosedLoopController leftHotwheelPIDController = leftHotwheel.getClosedLoopController();
    SparkClosedLoopController rightHotwheelController = rightHotwheel.getClosedLoopController();
    drumPIDController.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
    pickupPIDController.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
    leftHotwheelPIDController.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
    rightHotwheelController.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
  }

  public Command intakeRPMCommand() {
    return this.run(() -> intakeRPM());
  }

  public Command stopIntakeCommand() {
    return this.run(() -> stopIntake());
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command dumpIntakeCommand() {
    return this.run(() -> dumpIntake());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Shooter", !irSensor.get());
  }
}
