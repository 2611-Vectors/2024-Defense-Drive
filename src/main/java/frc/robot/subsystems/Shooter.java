package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.Vortex;
import frc.robot.VectorKit.hardware.Vortex.MotorAlignmentValue;
import frc.robot.VectorKit.tuners.PidTuner;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
    private final Vortex topShooter = new Vortex(ShooterConstants.TOP_SHOOTER);
    private final Vortex bottomShooter = new Vortex(ShooterConstants.BOTTOM_SHOOTER);

    private final LoggedNetworkNumber shooterSpeed =
            new LoggedNetworkNumber("/Shooter/Target RPM", PowerConstants.SHOOTER_VELOCITY);
    private final PidTuner shooterPidTuner = new PidTuner("/Shooter/Tuning", 0.0001, 0, 0, 0, 0.0018);

    public Shooter() {
        topShooter.setBrakeMode(false);
        bottomShooter.setBrakeMode(false);

        topShooter.addTuner(shooterPidTuner);
        topShooter.addFollower(bottomShooter, MotorAlignmentValue.Opposed);
    }

    public void stopShooter() {
        topShooter.stop();
    }

    public Command shoot() {
        return topShooter.setVelocity(() -> shooterSpeed.get(), () -> RPM).handleInterrupt(() -> stopShooter());
    }

    public Boolean isUpToSpeed() {
        return Math.abs(topShooter.getRPM() - shooterSpeed.get()) <= 50.0;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/RPM", topShooter.getRPM());
    }
}
