// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.Shooter;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;

  private final CommandXboxController m_DriverController =
      new CommandXboxController(Constants.ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final Shooter m_Shooter;
  private final Intake m_Intake;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_Shooter = new Shooter();
    m_Intake = new Intake();
  }

  @Override
  public void robotInit() {
    Shooter.initDashboard();
    Constants.initDashboard();
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    Shooter.updateFromDashboard();
    Constants.updateFromDashboard();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}
  /*
  //maxSpeed tuntime variable breaks AutonomousCommand but 2024 has no auto so we chillin
  @Override
  public void autonomousInit() {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
          CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
  } */

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
    // Intakes with left trigger
    m_DriverController.leftTrigger().whileTrue(m_Intake.intakeCommand());

    // Shoots (hopefully) with right trigger
    m_DriverController.rightTrigger().whileTrue(m_Shooter.shootCommand());

    // Runs the RPMshoot command to spin shooter at 60 rpm w/ right bumper (maybe)
    m_DriverController.rightTrigger().whileTrue(m_Shooter.RPMshootCommand());

    // Runs intake and shooter backwards w/ left bumper
    m_DriverController.leftTrigger().whileTrue(m_Shooter.dumpShootCommand());
    m_DriverController.leftTrigger().whileTrue(m_Intake.dumpIntakeCommand());
    // m_DriverController.leftTrigger().whileTrue(m_Intake.stopIntakeCommand());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
