// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.aux.Intake;
import frc.robot.subsystems.aux.Shooter;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {}

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    Shooter.initDashboard();
    Intake.initDashboard();
    Constants.initDashboard();
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    Shooter.updateFromDashboard();
    Intake.updateFromDashboard();
    Constants.updateFromDashboard();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  // maxSpeed tuntime variable breaks AutonomousCommand but 2024 has no auto so we chillin
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // shedule the autonomous command
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
