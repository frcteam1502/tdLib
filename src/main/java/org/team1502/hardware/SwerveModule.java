package org.team1502.hardware;

import org.team1502.configuration.builders.motors.ISwerveModule;
import org.team1502.configuration.builders.motors.SwerveModuleBuilder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements ISwerveModule, Sendable {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final CANcoder absEncoder;
    private final SparkClosedLoopController drivePIDController;
    private final PIDController turningPIDController;
    public final double maxSpeed;

    private double commandedSpeed;
    private double commandedAngle;

    public SwerveModule(SwerveModuleBuilder config) {
        config.setSwerveModuleInstance(this); // for logging
    
        this.maxSpeed = config.calculateMaxSpeed();
    
        this.driveMotor = config.DrivingMotor().buildSparkMax();
        this.driveEncoder = config.DrivingMotor().getRelativeEncoder();
        this.drivePIDController = config.DrivingMotor().getPIDController();
    
        this.turningMotor = config.TurningMotor().buildSparkMax();
        this.turningPIDController = config.TurningMotor().PID().createPIDController();
    
        this.absEncoder = config.Encoder().buildCANcoder();    
    }

    /** Returns the current state of the module.
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getAbsPositionZeroed()));
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /** Returns the current position of the module.
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getAbsPositionZeroed()));
    }

    public void zeroModule() {
        driveEncoder.setPosition(0);
    }

    /** Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {   
        //Set SmartDashboard variables
        this.commandedSpeed = desiredState.speedMetersPerSecond;
        this.commandedAngle = desiredState.angle.getDegrees();

        if (Math.abs(desiredState.speedMetersPerSecond) < .2) {
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        } else {
            // Optimize the reference state to avoid spinning further than 90 degrees
            desiredState.optimize(new Rotation2d(getAbsPositionZeroed()));

            //Set SmartDashboard variables
            this.commandedSpeed = desiredState.speedMetersPerSecond;
            this.commandedAngle = desiredState.angle.getDegrees();

            // Calculate the turning motor output from the turning PID controller.
            final double turnOutput = turningPIDController.calculate(getAbsPositionZeroed(), desiredState.angle.getRadians());
            turningMotor.setVoltage(turnOutput);

            var desiredSpeed = desiredState.speedMetersPerSecond/maxSpeed;
            drivePIDController.setReference(desiredSpeed, ControlType.kVelocity);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("Speed Command", ()->getCommandedSpeed(), null);
        builder.addDoubleProperty("Angle Command", ()->getCommandedAngle(), null);
        builder.addDoubleProperty("Speed Setpoint", ()->getControllerSetpoint(), null);
        builder.addDoubleProperty("Speed", ()->getModuleVelocity(), null);
        builder.addDoubleProperty("Angle", ()->Math.toDegrees(getAbsPositionZeroed()), null);
    }

    //CANcoders in Phoenix return rotations 0 to 1 (or -0.5 to 0.5)
    private double getAbsPositionZeroed() { return angleFromRotations(absEncoder.getAbsolutePosition().getValue()); }
    private double getCommandedSpeed() { return commandedSpeed; }
    private double getModuleVelocity() { return driveEncoder.getVelocity(); }
    private double getCommandedAngle() { return commandedAngle; }
    private double getControllerSetpoint() { return driveMotor.get(); }

    static double angleFromRotations(Angle rot) { return rot.magnitude() * 2 * Math.PI; }

}
