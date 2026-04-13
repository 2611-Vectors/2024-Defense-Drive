package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.Shooter;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class ShooterCommand extends Command {
  Shooter shooter;
  Intake intake;
  Supplier<Boolean> bButtonSupplier;
  Supplier<Boolean> yButtonSupplier;
  Supplier<Boolean> aButtonSupplier;
  boolean rightTriggerDebounce;
  CommandSwerveDrivetrain m_Drivetrain;

  public ShooterCommand(
      CommandSwerveDrivetrain drivetrain,
      Shooter shooter,
      Intake intake,
      Supplier<Boolean> bButtonSupplier,
      Supplier<Boolean> yButtonSupplier,
      Supplier<Boolean> aButtonSupoSupplier) {
    this.m_Drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.bButtonSupplier = bButtonSupplier;
    this.yButtonSupplier = yButtonSupplier;
    this.yButtonSupplier = aButtonSupplier;
    rightTriggerDebounce = false;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    intake.intakeMotorPower(0);
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
