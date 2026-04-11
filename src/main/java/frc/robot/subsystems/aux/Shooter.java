/**************************************************************************
All PID stuff needs to be comlpetely reworked to the new 2025+ RevLIB API
**************************************************************************/

package frc.robot.subsystems.aux;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDFF_CONSTANTS;

public class Shooter extends SubsystemBase {
    SparkMax tilt = new SparkMax(Constants.ShooterConstants.TILT, SparkMax.MotorType.kBrushless);
    SparkMax bottomShooter = new SparkMax(Constants.ShooterConstants.BOTTOM_SHOOTER, SparkFlex.MotorType.kBrushless);
    SparkMax topShooter = new SparkMax(Constants.ShooterConstants.TOP_SHOOTER, SparkFlex.MotorType.kBrushless);
    DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(0);

    SparkClosedLoopController bottomShooterPID, topShooterPID;
    PIDController tiltPID;

    public RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
    public double velocityRPM, targetAngle =190;
    public double p = 6e-5, i = 0, d = 0, pivotFF = 0.000180;
    public double ffPower = 0.024;
    public boolean enableShooter = false;

    public Shooter(){
        topShooterPID = topShooter.getClosedLoopController();
        bottomShooterPID = bottomShooter.getClosedLoopController();

        tiltPID = new PIDController(0.02, 0.0001, 0.000001);

        shooterTopEncoder = topShooter.getEncoder();
        shooterBottomEncoder = bottomShooter.getEncoder();

        velocityRPM = 3000;
// Doesn't work due to RevLIB API changes, need to be updated 
        // configureShooterPID(topShooterPID, Constants.shooterPID);
        // configureShooterPID(bottomShooterPID, Constants.shooterPID);

        // tiltPID.setP(p);
        // tiltPID.setI(i);
        // tiltPID.setD(d);

        // SmartDashboard.putNumber("P Angle", p);
        // SmartDashboard.putNumber("I Angle", i);
        // SmartDashboard.putNumber("D Angle", d);
        // SmartDashboard.putNumber("FF Angle", pivotFF);

        SmartDashboard.putNumber("Velocity", velocityRPM);
        SmartDashboard.putNumber("Target Angle", targetAngle);

        enableShooter = false;
    }

     public void setTargetAngle(double tAngle) {
    targetAngle = tAngle;
  }

//  
//   public void configurePID(SparkClosedLoopController motorPID, double p, double i, double d, double ff) {
//     motorPID.setP(p);
//     motorPID.setI(i);
//     motorPID.setD(d);
//     motorPID.setFF(ff);
//     motorPID.setOutputRange(-1, 1);
//   }
//
//   public void configureShooterPID(SparkClosedLoopController motorPID, PIDFF_CONSTANTS shooterConstants) {
//     configurePID(motorPID, 
//     shooterConstants.getP(), 
//     shooterConstants.getI(), 
//     shooterConstants.getD(),
//     shooterConstants.getFF());
//   }
    @Override
    public void periodic() {
        // This method will be called once per scheduler
        // double _targetAngle = SmartDashboard.getNumber("Target Angle", 0.0);
        // double _velocity = SmartDashboard.getNumber("Velocity", 0.0);

        // double _p = SmartDashboard.getNumber("P Angle", 0.0);
        // double _i = SmartDashboard.getNumber("I Angle", 0.0);
        // double _d = SmartDashboard.getNumber("D Angle", 0.0);
        // double _ff = SmartDashboard.getNumber("FF Angle", 0.0);
        SmartDashboard.putNumber("Top Speed", shooterTopEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Speed", shooterBottomEncoder.getVelocity());
        // if (_targetAngle != tempTargetAngle) {
        // tempTargetAngle = _targetAngle;
        // }
        // if (_velocity != tempVelocityRPM) {
        // tempVelocityRPM = _velocity;
        // }
        /***********************************
         * // sets the PID of angle on shooter from Smart Dashboard
         * if(_p != p) {
         * p = _p;
         * pivotPID.setP(p);
         * }
         * if(_i != i) {
         * i = _i;
         * pivotPID.setI(i);
         * }
         * if(_d != d) {
         * d = _d;
         * pivotPID.setD(d);
         * }
         * if(_ff != pivotFF) {
         * pivotFF = _ff;
         * }
         ***************************************************************************/
        // if (_p != p) {
        // p = _p;
        // pivotPID.setP(p);
        // }
        // if (_i != i) {
        // i = _i;
        // pivotPID.setI(i);
        // }
        // if (_d != d) {
        // d = _d;
        // pivotPID.setD(d);
        // }
        // if (_ff != pivotFF) {
        // pivotFF = _ff;
        // }
        SmartDashboard.putNumber("Pivot Encoder", getTiltAngle());
    }

    public void setShooterVelocity() {
        // topShooterPID.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
        // bottomShooterPID.setReference(-velocityRPM,
        // SparkMax.ControlType.kVelocity);

        topShooterPID.setSetpoint(velocityRPM, SparkMax.ControlType.kVelocity);
        bottomShooterPID.setSetpoint(-velocityRPM * 0.8, SparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Target", velocityRPM);
        SmartDashboard.putNumber("Actual Top", shooterTopEncoder.getVelocity());
        SmartDashboard.putNumber("Actual Bottom", shooterBottomEncoder.getVelocity());
        SmartDashboard.putNumber("Difference of Shooter",
            shooterTopEncoder.getVelocity() - Math.abs(shooterBottomEncoder.getVelocity()));
    }

    public void stopShooter() {
        topShooter.set(0);
        bottomShooter.set(0);
    }

    public void setTiltAngle() {
            SmartDashboard.putNumber("Target Angle", targetAngle);
            if (targetAngle < 185 || targetAngle > 223 || getTiltAngle() > 223) {
        tilt.set(0);
        SmartDashboard.putNumber("Targeting an invalid value", targetAngle);
        return;
        }

        // Amount of power to 
        double power = (tiltPID.calculate(getTiltAngle(), targetAngle) + ffPower);

        if (getTiltAngle() < 185 && Math.signum(power) == -1) {
            SmartDashboard.putString("Trying to hit ground", "1");
            tilt.set(0);
            return;
        }

        SmartDashboard.putNumber("PID Power", Math.signum(power) * Math.min(Math.abs(power), Constants.ShooterConstants.MAX_TILT_POWER));
        tilt.set(Math.signum(power) * Math.min(Math.abs(power), Constants.ShooterConstants.MAX_TILT_POWER));
    }

    public void stopTilt() {
        tilt.set(0);
    }

    public double getTiltAngle() {
        double correctedAngle = tiltEncoder.get() * 360 + Constants.TILT_ANGLE_OFFSET;
        if (correctedAngle < 0) {
            correctedAngle += 360;
        }
        if (correctedAngle > 360) {
            correctedAngle %= 360;
        }
        return correctedAngle;
    }
}
