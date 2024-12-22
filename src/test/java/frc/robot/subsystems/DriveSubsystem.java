package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1502.configuration.builders.motors.SwerveDriveBuilder;
import org.team1502.configuration.factory.RobotConfiguration;
import org.team1502.hardware.SwerveDrive;
import org.team1502.hardware.SwerveModules;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem implements Subsystem {
    final Pigeon2 gyro;
    final SwerveDrive swerveDrive;

    public DriveSubsystem(RobotConfiguration configuration) {
        gyro = configuration.Pigeon2().buildPigeon2();
        swerveDrive = configuration.SwerveDrive().buildSwerveDrive();
        // SwerveDriveBuilder swerveDrive = configuration.SwerveDrive();
        // SwerveModules swerveModules = new SwerveModules(swerveDrive);
        // SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
        // double maxSpeed = swerveDrive.calculateMaxSpeed();

    //public DriveSubsystem(SwerveDrive swerveDrive) {
        System.out.println("DriveSubsystme constructed");
    }

    public void Drive(double xSpeed, double ySpeed, double rotationSpeed)
    {
        this.swerveDrive.driveRobotRelative(null);
    } 
}
