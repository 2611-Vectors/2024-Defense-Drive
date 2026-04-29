package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.VectorKit.vision.Vision;
import frc.robot.VectorKit.vision.VisionIOPhotonVision;
import frc.robot.VectorKit.vision.VisionIOPhotonVisionSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindToStart;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive m_Drive;
  private final Shooter Shooter;
  private final Intake m_Intake;

  @SuppressWarnings("unused")
  private final Vision m_Vision;

  // Controller
  private final CommandXboxController m_DriverController =
      new CommandXboxController(Constants.ControllerConstants.DRIVER_CONTROLLER_ID);

  // Dashboard Inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    Shooter = new Shooter();
    m_Intake = new Intake();

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

    configureBindings();
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    m_Drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_Drive,
            () -> -m_DriverController.getLeftY(),
            () -> -m_DriverController.getLeftX(),
            () -> -m_DriverController.getRightX()));

    // Lock to 0° when A button is held
    m_DriverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> -m_DriverController.getLeftY(),
                () -> -m_DriverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    m_DriverController.x().onTrue(Commands.runOnce(m_Drive::stopWithX, m_Drive));

    // Reset gyro to 0° when Back button is pressed
    m_DriverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_Drive.setPose(
                            new Pose2d(m_Drive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_Drive)
                .ignoringDisable(true));

    m_DriverController.a().whileTrue(m_Intake.intake());
  }

  public Command getAutonomousCommand() {
    return new PathfindToStart(new PathPlannerAuto(autoChooser.get().getName()));
  }
}
