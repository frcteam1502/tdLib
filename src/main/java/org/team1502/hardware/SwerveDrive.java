package org.team1502.hardware;

import org.team1502.configuration.builders.motors.SwerveDriveBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    final SwerveDriveKinematics kinematics;
    final SwerveModules swerveModules;
    final double maxFreeSpeed;
    final double maxRotationalSpeed;
    //** adjustable destaturate speed */
    public double maxSpeed;

    public SwerveDrive(SwerveDriveBuilder builder) {
        kinematics = builder.getKinematics();
        swerveModules = builder.getSwerveModules();
        maxFreeSpeed = builder.calculateMaxSpeed();
        maxSpeed = builder.TopSpeed();
        maxRotationalSpeed = builder.calculateMaxRotationSpeed();
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
    
    /** set swerve module states directly */
    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
         swerveModules.setDesiredState(swerveModuleStates); 
    }

}
