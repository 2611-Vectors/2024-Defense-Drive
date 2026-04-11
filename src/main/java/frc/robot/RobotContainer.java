// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class RobotContainer {

    /*MaxSpeed directly scales joystick inputs, this is where we should test and 
    adjust float multiplier to find a comfortable but still fast max speed before we edit SpeedAt12Volts 
    in TunerConstants (add a runtime variable we can chance in advantage scope on the fly?) */
    // MaxSpeed is now exposed as a NetworkTables entry so it can be changed at
    // runtime (e.g. via Advantage Scope) without redeploying code. We set the
    // default to the original constant value from TunerConstants.
    private final NetworkTableEntry maxSpeedEntry = NetworkTableInstance.getDefault()
        .getTable("Robot")
        .getEntry("MaxSpeed");
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Note: we no longer build a single `drive` template at construction time
    // because the deadband depends on MaxSpeed which is runtime-configurable.
    // Instead we construct a FieldCentric request where needed using the
    // current value from NetworkTables.
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Initialize telemetry with the original default speed so logs start
    // consistent before any runtime changes are made.
    private final Telemetry logger = new Telemetry(1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // initialize NetworkTable default value so Advantage Scope (or any
        // NetworkTables client) will see a sensible starting value.
        maxSpeedEntry.setDouble(1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

        configureBindings();
    }

    /**
     * Read the current MaxSpeed from NetworkTables. Advantage Scope or any
     * NetworkTables client can change this value at runtime.
     */
    private double getMaxSpeed() {
        return maxSpeedEntry.getDouble(1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically. Build a
            // FieldCentric request each iteration so changes to MaxSpeed take
            // effect immediately.
            drivetrain.applyRequest(() -> {
                final double maxSpeed = getMaxSpeed();
                return new SwerveRequest.FieldCentric()
                    .withDeadband(maxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(-joystick.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    //maxSpeed tuntime variable breaks AutonomousCommand but 2024 has no auto so we 
/* 
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
        */
}