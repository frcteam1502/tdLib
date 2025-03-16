package org.team1502.drivers;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.drives.MecanumDriveBuilder;
import org.team1502.configuration.builders.drives.SwerveDriveBuilder;
import org.team1502.configuration.builders.motors.Motor;
import org.team1502.configuration.builders.motors.MotorControllerBuilder;
import org.team1502.configuration.factory.Evaluator;
import org.team1502.configuration.factory.FactoryTestsBase;
import org.team1502.configuration.factory.RobotConfiguration;
import org.team1502.injection.RobotFactory;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Mecanum1Subsystem;

public class mecanumTests {

    static Angle gyroAngle = Units.Degrees.of(0);
    public static Angle getAngle() {
        return gyroAngle;
    }

    @Test
    public void mecanumTest1() {
        var robotConfiguration = RobotConfiguration.Test("mecanumTest1", fn->fn
            .Parts(inventory->inventory
                .Motor(Motor.NEO, m->m
                    .MotorType(MotorType.kBrushless)
                    .FreeSpeedRPM(5_820.0)
                )
                .MotorController("Mecanum", Manufacturer.REVRobotics, c->c
                    .Motor(Motor.NEO)
                    .IdleMode(IdleMode.kBrake)
                    .GearBox(g->g
                        .Gear("Stage1", 14, 50)
                        .Gear("Stage2", 14, 50) // 12.75:1, 44.11 ft-lbs, 445.18 rpm, 15.54 ft/s
                        .Wheel(Inches.of(8.0)))
                    .SmartCurrentLimit(40)
                )

            )  
        );
        robotConfiguration.Build(builder->builder
            .Subsystem(Mecanum1Subsystem.class, sys->sys
                .Pigeon2(g->g
                    .CanNumber(14))
                .MecanumDrive(m->m
                    .Chassis(c->c.Rectangular(Inches.of(6), Inches.of(8)))
                    .MotorController("Front Left", "Mecanum", c->c
                        .CanNumber(3))
                    .MotorController("Front Right", "Mecanum", c->c
                        .Reversed()
                        .CanNumber(5))
                    .MotorController("Rear Left", "Mecanum", c->c
                        .CanNumber(4))
                    .MotorController("Rear Right", "Mecanum", c->c
                        .Reversed()
                        .CanNumber(14))
                    // MecanumComtroller Command Information
                    .TrajectoryConfig(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(3.0))
                    .PIDController(MecanumDriver.XController, p->p.Gain(0.5, 0.0, 0.0))
                    .PIDController(MecanumDriver.YController, p->p.Gain(0.5, 0.0, 0.0))
                    .PIDController(MecanumDriver.ThetaController, p->p
                        .Gain(0.5, 0.0, 0.0)
                        .Constraints(Math.PI, Math.PI))
                )
            )        
        );

        RobotFactory factory = RobotFactory.Create(Mecanum1Subsystem.class, robotConfiguration);
        Evaluator mecanumTest1 = robotConfiguration.Values();
        
        var mecanum1Subsystem = mecanumTest1.Subsystem("frc.robot.subsystems.Mecanum1Subsystem");
        var mecanumDrive = mecanum1Subsystem.MecanumDrive();
        var xController = mecanumDrive.PIDController("xController");
        var pX = xController.P();
        assertEquals(0.5, pX);

        var driveSubsystem = factory.getInstance(Mecanum1Subsystem.class);
        driveSubsystem.m_gyroYaw = ()->getAngle();
        var driver = driveSubsystem.m_drive;
        var modules = driveSubsystem.m_drive.m_modules;
        
        driveSubsystem.drive(1.0, 0, 0, false);
        dump(modules.m_modules);
        driveSubsystem.drive(-1.0, 0, 0, false);
        dump(modules.m_modules);
        driveSubsystem.drive(0, 1, 0, false);
        dump(modules.m_modules);
        driveSubsystem.drive(0, -1, 0, false);
        dump(modules.m_modules);
        driveSubsystem.drive(1, -1, 0, false);
        dump(modules.m_modules);
        driveSubsystem.drive(0.75, 0, -0.25, false); // CCW! @param zRotation is CW
        dump(modules.m_modules);
        driveSubsystem.drive(0.75, 0, 0.25, false); // CW
        dump(modules.m_modules);

        gyroAngle = Units.Degree.of(45);
        driveSubsystem.drive(0.0, 1.0, 0, true);
        dump(modules.m_modules);

        driveSubsystem.drive(1.0, 0, 0, false);
        for (int i=0; i<4; i++) {
            assertEquals(1.0, modules.m_modules.get(i).get()); 
        }
        // remember: it is the pattern on the floor, not the top of the wheel that drives the robot
        driveSubsystem.drive(0.0, 1.0, 0, false); // West, Left
        assertEquals(1.0, modules.m_modules.get(0).get());
        assertEquals(-1.0, modules.m_modules.get(1).get());
        assertEquals(-1.0, modules.m_modules.get(2).get());
        assertEquals(1.0, modules.m_modules.get(3).get());
        driveSubsystem.drive(0.0, -1.0, 0, false); // East, Right
        assertEquals(-1.0, modules.m_modules.get(0).get());
        assertEquals(1.0, modules.m_modules.get(1).get());
        assertEquals(1.0, modules.m_modules.get(2).get());
        assertEquals(-1.0, modules.m_modules.get(3).get());
        driveSubsystem.drive(0.0, 1.0, 0, true); // East, Right
        assertEquals(1.0, modules.m_modules.get(0).get());
        assertEquals(0.0, modules.m_modules.get(1).get(), 0.01);
        assertEquals(0.0, modules.m_modules.get(2).get(), 0.01);
        assertEquals(1.0, modules.m_modules.get(3).get());
        driveSubsystem.drive(1.0, -1.0, 0, false); // NE
        assertEquals(0, modules.m_modules.get(0).get());   // x + y
        assertEquals(1.0, modules.m_modules.get(1).get()); // x - y (normalized)
        assertEquals(1.0, modules.m_modules.get(2).get()); // x - y (normalized)
        assertEquals(0, modules.m_modules.get(3).get());   // x + y
        driveSubsystem.drive(0.75, 0, 0.25, false); // CW
        assertEquals(1.0, modules.m_modules.get(0).get(), 0.01); // x + y + zrot
        assertEquals(0.5, modules.m_modules.get(1).get(), 0.01); // x - y - zrot
        assertEquals(1.0, modules.m_modules.get(2).get(), 0.01); // x - y + zrot
        assertEquals(0.5, modules.m_modules.get(3).get(), 0.01); // x + y - zrot

        var config = driveSubsystem.getTrajectoryConfig();
            Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Pose2d.kZero,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.kZero),
            config);
        
        var mecanumControllerCommand = driveSubsystem.buildMecanumControllerCommand(exampleTrajectory);
        var command = Commands.sequence(
            new InstantCommand(() -> driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose())),
            mecanumControllerCommand,
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false))
        );
    
        FactoryTestsBase.DumpParts(robotConfiguration);
    }

    void dump(List<MotorControllerBuilder> modules) {
        System.out.println(String.format("%.1f %.1f %.1f %.1f", modules.get(0).get(), modules.get(1).get(),modules.get(2).get(),modules.get(3).get())); 
    }
    
}
