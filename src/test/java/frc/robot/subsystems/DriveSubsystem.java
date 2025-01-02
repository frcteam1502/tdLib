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
import org.team1502.swerve.SwerveDrive;
import org.team1502.swerve.SwerveModules;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem implements Subsystem {
    final Pigeon2 gyro;
    /** generic Angle in degrees */
    public /*final*/ Supplier<Angle> gyroYaw;
    /** generic Rotation2d in radians */
    public /*final*/ Supplier<Rotation2d> gyroRotation2d;

    public final SwerveDrive swerveDrive;
    //ChassisSpeeds speedCommands = new ChassisSpeeds(); // re-use existing object

    public DriveSubsystem(RobotConfiguration configuration) {
        gyro = configuration.Pigeon2().buildPigeon2();
        var statusCode  = gyro.setYaw(0); // whichever way we are pointing is 0 (+X direction)
        gyroYaw = gyro.getYaw().asSupplier();
        gyroRotation2d = ()->new Rotation2d(gyroYaw.get());

        swerveDrive = configuration.SwerveDrive().buildSwerveDrive(gyroRotation2d);
    }

    @Override
    public void periodic() {
        swerveDrive.periodic();
    }

    public void swerveDrive(double forwardUnitVelocity, double leftUnitVelocity, double ccwUnitVelocity) {
        swerveDrive.swerveDrive(forwardUnitVelocity, leftUnitVelocity, ccwUnitVelocity);
    }

    public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldRelative) {
        ChassisSpeeds speedCommands = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        if (fieldRelative){
            speedCommands.toRobotRelativeSpeeds(gyroRotation2d.get());
        }
        driveRobotRelative(speedCommands);
    }

    public void driveRobotRelative(ChassisSpeeds speedCommands)
    {
        this.swerveDrive.driveRobotRelative(speedCommands);
    } 
}
