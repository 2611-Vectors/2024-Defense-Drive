package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.Neo;
import frc.robot.VectorKit.hardware.Vortex;
import frc.robot.VectorKit.tuners.PidTuner;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final DigitalInput irSensor = new DigitalInput(1);

    private final Vortex groundPickup = new Vortex(IntakeConstants.GROUND_PICKUP);
    private final Neo leftHotwheel = new Neo(IntakeConstants.LEFT_HOTWHEEL);
    private final Neo rightHotwheel = new Neo(IntakeConstants.RIGHT_HOTWHEEL);
    private final Neo loadingDrum = new Neo(ShooterConstants.LOADING_DRUM);

    private final PidTuner drumTuner = new PidTuner("/Loading Drum/Tuning", 0.0001, 0.0, 0.0, 0.0, 0.0021);

    public Intake() {
        groundPickup.setBrakeMode(true);
        leftHotwheel.setBrakeMode(true);
        rightHotwheel.setBrakeMode(true);
        loadingDrum.setBrakeMode(true);

        leftHotwheel.setInverted(InvertedValue.CounterClockwise_Positive);
        leftHotwheel.addFollower(rightHotwheel, Neo.MotorAlignmentValue.Opposed);

        loadingDrum.addTuner(drumTuner);
    }

    public Command intake() {
        return new ParallelCommandGroup(
                        groundPickup.set(() -> PowerConstants.INTAKE_POWER),
                        leftHotwheel.set(() -> PowerConstants.INTAKE_POWER),
                        loadingDrum.setVelocity(() -> PowerConstants.DRUM_VELOCITY, () -> RPM))
                .onlyWhile(() -> irSensor.get())
                .finallyDo(() -> stopAll());
    }

    public Command feed() {
        return loadingDrum
                .setVelocity(() -> PowerConstants.DRUM_VELOCITY, () -> RPM)
                .finallyDo(() -> stopAll());
    }

    public Command extake() {
        return new ParallelCommandGroup(
                        groundPickup.set(() -> -PowerConstants.INTAKE_POWER),
                        leftHotwheel.set(() -> -PowerConstants.INTAKE_POWER),
                        loadingDrum.setVelocity(() -> -PowerConstants.DRUM_VELOCITY, () -> RPM))
                .finallyDo(() -> stopAll());
    }

    public void stopAll() {
        groundPickup.stop();
        leftHotwheel.stop();
        loadingDrum.stop();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Note Loaded", !irSensor.get());
        Logger.recordOutput("Loading Drum/RPM", loadingDrum.getRPM());
    }
}
