package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.Vortex;
import frc.robot.VectorKit.hardware.Vortex.MotorAlignmentValue;
import frc.robot.VectorKit.tuners.PidTuner;

public class Shooter extends SubsystemBase {
    private final Vortex topShooter = new Vortex(ShooterConstants.TOP_SHOOTER);
    private final Vortex bottomShooter = new Vortex(ShooterConstants.BOTTOM_SHOOTER);

    private final PidTuner shooterPidTuner = new PidTuner("/Shooter/Tuning", 0, 0, 0, 0, 0);

    public Shooter() {
        topShooter.setBrakeMode(false);
        bottomShooter.setBrakeMode(false);

        topShooter.addTuner(shooterPidTuner);
        topShooter.addFollower(bottomShooter, MotorAlignmentValue.Opposed);
    }

    public Command stopShooter() {
        return new ParallelCommandGroup(
                topShooter.setVelocity(() -> 0.0, () -> RPM), bottomShooter.setVelocity(() -> 0.0, () -> RPM));
    }

    public Command shoot() {
        return topShooter.setVelocity(() -> PowerConstants.SHOOTER_VELOCITY, () -> RPM);
    }

    @Override
    public void periodic() {}
}
