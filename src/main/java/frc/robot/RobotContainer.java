package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VectorKit.vision.Vision;
import frc.robot.VectorKit.vision.VisionIOPhotonVision;
import frc.robot.VectorKit.vision.VisionIOPhotonVisionSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HandleAutonShoot;
import frc.robot.commands.PathfindToStart;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.Shooter;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final NetworkTableEntry maxSpeedEntry =
      NetworkTableInstance.getDefault().getTable("Robot").getEntry("MaxSpeed");
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Subsystems
  private final Drive m_Drive;
  private final Shooter Shooter;
  private final Intake Intake;

  private final CommandSwerveDrivetrain m_CommandSwerveDrive = TunerConstants.createDrivetrain();

  @SuppressWarnings("unused")
  private final Vision m_Vision;

  // Controller
  private final CommandXboxController Controller =
      new CommandXboxController(Constants.ControllerConstants.DRIVER_CONTROLLER_PORT);

  // Dashboard Inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Test Command
  public Command timeoutShoot() {
    return new SequentialCommandGroup(
        Shooter.shootCommand(),
        Intake.stopDrumShootCommand().withTimeout(DashboardConstants.SHOOTER_TIMEOUT),
        Intake.drumShootCommand(),
        Shooter.stopShootCommand(),
        Intake.stopDrumShootCommand());
  }

  public RobotContainer() {
    Shooter = new Shooter();
    Intake = new Intake();

    switch (Constants.currentMode) {
      case REAL:
        m_Drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                m_Drive::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.leftCam, VisionConstants.robotToLeftCam),
                new VisionIOPhotonVision(
                    VisionConstants.rightCam, VisionConstants.robotToRightCam));
        break;

      case SIM:
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                m_Drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCam, VisionConstants.robotToLeftCam, m_Drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCam, VisionConstants.robotToRightCam, m_Drive::getPose));
        break;

      default:
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        m_Vision = null;

        break;
    }

    autoChooser = new LoggedDashboardChooser<>("AutoChoices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("Enable Intake", new HandleAutonShoot(Intake, Shooter));

    // initialize NetworkTable default value so Advantage Scope (or any
    // NetworkTables client) will see a sensible starting value.
    maxSpeedEntry.setDouble(1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    configureBindings();
  }

  private double getMaxSpeed() {
    return maxSpeedEntry.getDouble(1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    m_CommandSwerveDrive.setDefaultCommand(
        m_CommandSwerveDrive.applyRequest(
            () -> {
              final double maxSpeed = getMaxSpeed();
              return new SwerveRequest.FieldCentric()
                  .withDeadband(maxSpeed * 0.1)
                  .withRotationalDeadband(MaxAngularRate * 0.1)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withVelocityX(
                      -Controller.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      -Controller.getLeftX() * maxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      -Controller.getRightX()
                          * MaxAngularRate); // Drive counterclockwise with negative X (left)
            }));

    // Lock to 0° when A button is held
    Controller.a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> -Controller.getLeftY(),
                () -> -Controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    Controller.x().onTrue(Commands.runOnce(m_Drive::stopWithX, m_Drive));

    // Reset gyro to 0° when Back button is pressed
    Controller.back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_Drive.setPose(
                            new Pose2d(m_Drive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_Drive)
                .ignoringDisable(true));

    // Start/Stop with right trigger (rebound to d-pad up for testing)
    Controller.povUp()
        .whileTrue(
            Commands.startEnd(() -> Shooter.shootCommand(), () -> Shooter.stopShootCommand()));
    Controller.povUp()
        .whileTrue(
            Commands.startEnd(
                () -> Intake.drumShootCommand(), () -> Intake.stopDrumShootCommand()));

    // Timeout to let the shooter get up to speed with right trigger
    Controller.rightTrigger().whileTrue(timeoutShoot());

    // Stop shooter with right bumper
    Controller.rightBumper().whileTrue(Shooter.stopShootCommand());
    Controller.rightBumper().whileTrue(Intake.stopDrumShootCommand());

    // Auto intake with left trigger
    Controller.leftTrigger()
        .whileTrue(Commands.startEnd(() -> Intake.autoIntake(), () -> Intake.stopIntake()));

    // Dump intake and shooter
    Controller.leftBumper()
        .whileTrue(
            Commands.startEnd(() -> Intake.intakeDumpCommand(), () -> Intake.stopIntakeCommand()));
    Controller.leftBumper()
        .whileTrue(
            Commands.startEnd(() -> Shooter.dumpShootCommand(), () -> Shooter.stopShootCommand()));

    // RPM shoot command with d-pad up
    Controller.povDown()
        .whileTrue(
            Commands.startEnd(() -> Shooter.RPMShootCommand(), () -> Shooter.stopShootCommand()));
    Controller.povDown()
        .whileTrue(
            Commands.startEnd(
                () -> Intake.drumRPMShootCommand(), () -> Intake.stopDrumShootCommand()));
  }

  public Command getAutonomousCommand() {
    // Guard against the chooser returning null (nothing selected on the dashboard).
    Command chosen = autoChooser.get();
    if (chosen == null) {
      // No autonomous selected; return an explicit no-op command instead of risking an NPE.
      return Commands.none();
    }

    // If the chooser directly returned a PathPlannerAuto, use it.
    if (chosen instanceof PathPlannerAuto) {
      return new PathfindToStart((PathPlannerAuto) chosen);
    }

    // Otherwise, try to extract a name from the chosen command (if it exposes one),
    // and build a PathPlannerAuto from that name. Fall back to returning the chosen
    // command itself if we can't determine a name.
    try {
      java.lang.reflect.Method getNameMethod = chosen.getClass().getMethod("getName");
      Object nameObj = getNameMethod.invoke(chosen);
      if (nameObj instanceof String) {
        return new PathfindToStart(new PathPlannerAuto((String) nameObj));
      }
    } catch (ReflectiveOperationException ignored) {
      // If reflection fails, we'll fall through to returning the chosen command.
    }

    // Default fallback: return whatever command was selected (it might already be a full auto),
    // or a no-op if not appropriate.
    return chosen != null ? chosen : Commands.none();
  }
}
