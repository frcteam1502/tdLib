package org.team1502.hardware;

import java.util.function.*;

import org.team1502.configuration.builders.motors.SwerveDriveBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    final SwerveDriveKinematics kinematics;
    final SwerveDrivePoseEstimator odometry;
    final Supplier<Rotation2d> gyroAngle;
    final SwerveModules swerveModules;
    final double maxFreeSpeed;
    final double maxRotationalSpeed;
    //** adjustable desaturate speed */
    public double maxSpeed;

    Pose2d pose = new Pose2d();

    public SwerveDrive(SwerveDriveBuilder builder, Supplier<Rotation2d> fnGyroAngle) {
        this.gyroAngle = fnGyroAngle;
        this.kinematics = builder.getKinematics();
        this.swerveModules = builder.getSwerveModules();
        this.maxFreeSpeed = builder.calculateMaxSpeed();
        this.maxSpeed = builder.TopSpeed();
        this.maxRotationalSpeed = builder.calculateMaxRotationSpeed();
        this.odometry = new SwerveDrivePoseEstimator(this.kinematics, this.gyroAngle.get(), this.swerveModules.getModulePositions(), this.pose);
    }

    /** Use ChassisSpeeds to set swerve modules */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        //Convert from robot frame of reference (ChassisSpeeds) to swerve module frame of reference (SwerveModuleState)
        var swerveModuleStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        //Normalize wheel speed commands to make sure no speed is greater than the maximum achievable wheel speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        //Set the speed and angle of each module
        setDesiredState(swerveModuleStates);
    }
    
    //=== SWERVE MODULE STATE ==========================================//
    /** set swerve module states directly */
    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
         swerveModules.setDesiredState(swerveModuleStates); 
    }

    //=== ODOMETRY ==========================================//
    public void updateOdometry() {
        odometry.update(gyroAngle.get(), swerveModules.getModulePositions());
    }
    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(gyroAngle.get(), swerveModules.getModulePositions(), pose2d);
    }
}
