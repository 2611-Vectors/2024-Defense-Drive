package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.Neo;
import frc.robot.VectorKit.hardware.Vortex;
import frc.robot.VectorKit.tuners.PidTuner;

public class Intake extends SubsystemBase {
  public DigitalInput irSensor = new DigitalInput(1);

  private final Vortex groundPickup = new Vortex(IntakeConstants.GROUND_PICKUP);
  private final Neo leftHotwheel = new Neo(IntakeConstants.LEFT_HOTWHEEL);
  private final Neo rightHotwheel = new Neo(IntakeConstants.RIGHT_HOTWHEEL);
  private final Neo loadingDrum = new Neo(ShooterConstants.LOADING_DRUM);

  private final PidTuner drumTuner = new PidTuner("/Loading Drum/Tuning", 0.1, 0.0, 0.0, 0.0, 0.0);

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    groundPickup.setBrakeMode(true);
    leftHotwheel.setBrakeMode(true);
    rightHotwheel.setBrakeMode(true);
    loadingDrum.setBrakeMode(true);

    leftHotwheel.addFollower(rightHotwheel, Neo.MotorAlignmentValue.Opposed);

    loadingDrum.addTuner(drumTuner);
  }

  public Command intake() {
    return new ParallelCommandGroup(
            groundPickup.set(() -> PowerConstants.INTAKE_POWER),
            leftHotwheel.set(() -> PowerConstants.INTAKE_POWER),
            loadingDrum.setVelocity(() -> PowerConstants.DRUM_VELOCITY, () -> RPM))
        .onlyWhile(() -> !irSensor.get())
        .andThen(() -> stopAll()).handleInterrupt();
  }

  private void stopAll() {
        Commands.runOnce(groundPickup.set(() -> 0.0));
        leftHotwheel.set(() -> 0.0);
        loadingDrum.setVelocity(() -> 0.0, () -> RPM);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Shooter", !irSensor.get());
  }
}
