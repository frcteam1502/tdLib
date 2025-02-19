package org.team1502.drivers;

import java.util.List;
import java.util.function.Supplier;

import org.team1502.configuration.builders.drives.MecanumDriveBuilder;
import org.team1502.configuration.builders.motors.MotorControllerBuilder;
import org.team1502.configuration.builders.motors.PID;
import org.team1502.drivers.MecanumDriver.Modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MecanumDriver {
    final MecanumDrive m_drive;
    final MecanumDriveKinematics m_kinematics;
    final MecanumDriveOdometry m_odometry;
    final Supplier<Rotation2d> m_yawSupplier;
    final Modules m_modules;
    final MecanumDriveBuilder m_builder;
    public MecanumDriver(MecanumDriveBuilder builder, Supplier<Rotation2d> yawSupplier) {
        m_builder = builder;
        m_yawSupplier = yawSupplier;
        m_modules = new Modules(builder.getModules());
        m_kinematics = builder.Chassis().getMecanumDriveKinematics();
        m_odometry = new MecanumDriveOdometry(
                m_kinematics,
                m_yawSupplier.get(),
                new MecanumDriveWheelPositions());
        
        m_modules.buildMotorControllers();
        m_drive = builder.getMecanumDrive();
    }

    public void periodic() {
        // Update the odometry in the periodic block
        var rotation = m_yawSupplier.get();
        MecanumDriveWheelPositions distances = getCurrentWheelDistances();

        m_odometry.update(rotation, distances);

    }
   
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getRotation2d(), getCurrentWheelDistances(), pose);
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_drive.driveCartesian(xSpeed, ySpeed, rot, m_yawSupplier.get());
        } else {
            m_drive.driveCartesian(xSpeed, ySpeed, rot);
        }
    }
    
    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            m_builder.getDouble("maxVelocity"),
            m_builder.getDouble("maxAcceleration"))
            .setKinematics(m_kinematics);
    }

    public MecanumControllerCommand buildMecanumControllerCommand(
        Trajectory trajectory,
        Subsystem subsystem) {
        return new MecanumControllerCommand(
                trajectory,
                ()->getPose(),
                m_kinematics,
                m_builder.PIDController("xController").getPIDController(),
                m_builder.PIDController("yController").getPIDController(),
                m_builder.PIDController("thetaController").getProfiledPIDController(),
                m_builder.getDouble("maxVelocity"),
                m_modules::setDriveMotorControllersVelocity,
                subsystem);

    }
    
    Rotation2d getRotation2d() { return m_yawSupplier.get(); }

    MecanumDriveWheelPositions getCurrentWheelDistances() {
        return m_modules.getCurrentWheelDistances();
    }
    
    public void setDriveMotorControllersVolts(
      double frontLeftVoltage,
      double frontRightVoltage,
      double rearLeftVoltage,
      double rearRightVoltage) {

    }

    class Modules {
        List<MotorControllerBuilder> m_modules;

        public Modules(List<MotorControllerBuilder> modules) {
            m_modules = modules;
        }

        public void buildMotorControllers() {
            m_modules.get(0).buildSparkMax();
            m_modules.get(1).buildSparkMax();
            m_modules.get(2).buildSparkMax();
            m_modules.get(3).buildSparkMax();

        }
        public MecanumDriveWheelPositions getCurrentWheelDistances() {
            return new MecanumDriveWheelPositions(
                    m_modules.get(0).getPosition(),
                    m_modules.get(1).getPosition(),
                    m_modules.get(2).getPosition(),
                    m_modules.get(3).getPosition());
        }

        public MecanumDriveWheelPositions getCurrentWheelSpeeds() {
            return new MecanumDriveWheelPositions(
                    m_modules.get(0).getVelocity(),
                    m_modules.get(1).getVelocity(),
                    m_modules.get(2).getVelocity(),
                    m_modules.get(3).getVelocity());
        }
        public void setDriveMotorControllersVelocity(MecanumDriveWheelSpeeds speeds) {
            m_modules.get(0).setVelocity(speeds.frontLeftMetersPerSecond);
            m_modules.get(1).setVelocity(speeds.frontRightMetersPerSecond);
            m_modules.get(2).setVelocity(speeds.rearLeftMetersPerSecond);
            m_modules.get(3).setVelocity(speeds.rearRightMetersPerSecond);
            
        }
        public void setDriveMotorControllersVolts(
                double frontLeftVoltage,
                double frontRightVoltage,
                double rearLeftVoltage,
                double rearRightVoltage) {
            m_modules.get(0).setVoltage(frontLeftVoltage);
            m_modules.get(1).setVoltage(frontRightVoltage);
            m_modules.get(2).setVoltage(rearLeftVoltage);
            m_modules.get(3).setVoltage(rearRightVoltage);
            
        }

    }

}
