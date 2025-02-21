package frc.robot.subsystems;

import java.util.function.Supplier;

import org.team1502.configuration.factory.RobotConfiguration;
import org.team1502.drivers.MecanumDriver;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Mecanum1Subsystem implements Subsystem {
    final Pigeon2 m_gyro;
    /** generic Angle in degrees */
    public /*final*/ Supplier<Angle> m_gyroYaw;
    /** generic Rotation2d in radians */
    public /*final*/ Supplier<Rotation2d> m_gyroRotation2d;

    public final MecanumDriver m_drive;
    final RobotConfiguration m_configuration;
    public Mecanum1Subsystem(RobotConfiguration configuration) {
        m_configuration = configuration;
        m_gyro = configuration.Pigeon2().buildPigeon2();
        var statusCode  = m_gyro.setYaw(0); // whichever way we are pointing is 0 (+X direction)
        m_gyroYaw = m_gyro.getYaw().asSupplier();
        m_gyroRotation2d = ()->new Rotation2d(m_gyroYaw.get());

        m_drive = configuration.MecanumDrive().buildDriver(m_gyroRotation2d);
    }

    @Override
    public void periodic() {
        m_drive.update();
    }

    public void resetOdometry(Pose2d pose) { m_drive.resetOdometry(pose); }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
    
    public TrajectoryConfig getTrajectoryConfig() {
        return m_drive.getTrajectoryConfig();  
    }
    public MecanumControllerCommand buildMecanumControllerCommand(Trajectory trajectory) {
        return m_drive.buildMecanumControllerCommand(trajectory, this);        
    }


}
