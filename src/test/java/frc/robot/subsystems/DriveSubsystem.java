package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.*;

import org.team1502.configuration.builders.motors.SwerveDriveBuilder;
import org.team1502.configuration.factory.RobotConfiguration;
import org.team1502.hardware.SwerveDrive;
import org.team1502.hardware.SwerveModules;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem implements Subsystem {
    final Pigeon2 gyro;
    /** generic Angle in degrees */
    final Supplier<Angle> gyroYaw;
    /** generic Rotation2d in radians */
    final Supplier<Rotation2d> gyroRotation2d;

    final SwerveDrive swerveDrive;
    ChassisSpeeds speedCommands = new ChassisSpeeds(); // re-use existing object

    public DriveSubsystem(RobotConfiguration configuration) {
        gyro = configuration.Pigeon2().buildPigeon2();
        gyroYaw = gyro.getYaw().asSupplier();
        gyroRotation2d = ()->new Rotation2d(gyroYaw.get());

        swerveDrive = configuration.SwerveDrive().buildSwerveDrive(gyroRotation2d);

    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
        speedCommands.vxMetersPerSecond = xSpeed;
        speedCommands.vyMetersPerSecond = ySpeed; 
        speedCommands.omegaRadiansPerSecond = rotationSpeed;
        if (fieldRelative){
            speedCommands.toRobotRelativeSpeeds(gyroRotation2d.get());
        }
        this.swerveDrive.driveRobotRelative(speedCommands);
    }

    public void driveRobotRelative(ChassisSpeeds speedCommands)
    {
        this.swerveDrive.driveRobotRelative(speedCommands);
    } 
}
