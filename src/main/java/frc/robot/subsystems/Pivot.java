// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.VectorKit.hardware.AbsoluteEncoder;
import frc.robot.VectorKit.hardware.Neo;
import frc.robot.VectorKit.tuners.TunablePidController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Pivot extends SubsystemBase {
    /** Creates a new Pivot. */
    private final Neo m_PivotMotor = new Neo(PivotConstants.TILT_ID);

    private final AbsoluteEncoder m_PivotEncoder =
            new AbsoluteEncoder(PivotConstants.ENCODER_ID, PivotConstants.PIVOT_OFFSET);

    private final TunablePidController pidController = new TunablePidController("/Pivot/Tuning", 0.02, 0, 0);

    public Pivot() {
        m_PivotMotor.setBrakeMode(true);
    }

    public Command manualVoltage() {
        LoggedNetworkNumber v = new LoggedNetworkNumber("/Pivot/Voltage", 0.0);
        return m_PivotMotor.set(() -> v.get());
    }

    public Command manualPosition() {
        LoggedNetworkNumber p = new LoggedNetworkNumber("/Pivot/Position", 10.0);
        Supplier<Double> v = () -> MathUtil.clamp(
                pidController.calculate(
                        m_PivotEncoder.get(),
                        MathUtil.clamp(p.get(), PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE)),
                -0.5,
                0.5);
        Logger.recordOutput("Pivot/Target Voltage", v.get());
        return m_PivotMotor.set(() -> v.get());
    }

    public Command stop() {
        return runOnce(() -> m_PivotMotor.stop());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Pivot/Encoder/New Offset", m_PivotEncoder.getRaw() * 360.0);
        Logger.recordOutput("Pivot/Encoder/Current Angle", m_PivotEncoder.get());

        Logger.recordOutput("Pivot/Amperage", m_PivotMotor.getAmperage());
    }
}
