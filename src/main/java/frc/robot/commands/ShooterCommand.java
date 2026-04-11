package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.Shooter;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class ShooterCommand extends Command {
    Shooter shooter;
    Intake intake;
    Supplier<Boolean> bButtonSupplier;
    Supplier<Boolean> yButtonSupplier;
    Supplier<Boolean> aButtonSupplier;
    boolean rightTriggerDebounce;
    private final XboxController driverController = new XboxController(0);
    CommandSwerveDrivetrain m_Drivetrain;

    public ShooterCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, 
        Supplier<Boolean> bButtonSupplier, Supplier<Boolean> yButtonSupplier, Supplier<Boolean> aButtonSupoSupplier) {
        this.m_Drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.bButtonSupplier = bButtonSupplier;
        this.yButtonSupplier = yButtonSupplier;
        // Shouldnt it be this.aButtonSupplier = aButtonSupplier; or no?
        this.yButtonSupplier = aButtonSupplier;
        rightTriggerDebounce = false;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.targetAngle = 187;
        shooter.enableShooter = false;
    }

    @Override
    public void execute() {
        if (RobotState.isTeleop()) {
            if (driverController.getRightTriggerAxis() > 0.5) {
                if (!rightTriggerDebounce) {
                    rightTriggerDebounce = true;
                    m_Drivetrain.resetPose(new Pose2d());
                }
            } else {
                rightTriggerDebounce = false;
            }
            shooter.setTiltAngle();
            if (shooter.enableShooter) {
                shooter.setShooterVelocity();
            } else {
                shooter.stopShooter();
            }
        }
    }

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
