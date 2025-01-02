package org.team1502.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.motors.Motor;
import org.team1502.configuration.builders.motors.SwerveDriveBuilder;
import org.team1502.configuration.builders.motors.SwerveModuleBuilder;
import org.team1502.configuration.factory.Evaluator;
import org.team1502.configuration.factory.FactoryTestsBase;
import org.team1502.configuration.factory.PartFactory;
import org.team1502.configuration.factory.RobotConfiguration;
import org.team1502.injection.RobotFactory;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.DriveSubsystem;

public class swerveTests {
    @Test
    public void importSwerveModuleTest() {

        var robotConfiguration = RobotConfiguration.Create("swerveTest1", fn -> fn
            .Parts(inventory -> {
                Motors(inventory);
                Mk4iL3(inventory);
                return inventory;
            }
        ));
        robotConfiguration.Build(builder->builder
            .SwerveModule("Module#1", sm->sm
                .CanNumber(2)
            )
        );

        FactoryTestsBase.DumpParts(robotConfiguration);

        Evaluator evaluator = robotConfiguration.Values();
        SwerveModuleBuilder smb1 = evaluator.SwerveModule("Module#1");

        var useHAL = System.getenv("HAL");
        SwerveModule sm = new SwerveModule(smb1);

        AssertMK4iL3(sm);

        sm.driveMotor.close();
        sm.turningMotor.close();
        sm.absEncoder.close();
    }

    @Test
    public void importSwerveDriveTest() {

        var robotConfiguration = RobotConfiguration.Create("swerveTest1", fn -> fn
            .Parts(inventory -> {
                Motors(inventory);
                Mk4iL3(inventory);
                return inventory;
            }
        ));
        robotConfiguration.Build(builder->builder
            .SwerveDrive(sd->sd
                .Chassis(c -> c
                    .Square(19.75)
                )
                .SwerveModule("Module#1", sm->sm
                    .CanNumber(4)
                )
                .SwerveModule("Module#2", sm->sm
                    .CanNumber(6)
                )
                .SwerveModule("Module#3", sm->sm
                    .CanNumber(8)
                )
                .SwerveModule("Module#4", sm->sm
                    .CanNumber(10)
                )
                .TopSpeed(4.6)
            )
        );

        FactoryTestsBase.DumpParts(robotConfiguration);

        Evaluator evaluator = robotConfiguration.Values();
        
        SwerveDriveBuilder swerveDrive = evaluator.SwerveDrive();
        SwerveModules modules = new SwerveModules(swerveDrive);
        SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
        double maxSpeed = swerveDrive.calculateMaxSpeed();
        //SwerveDrivePoseEstimator odometry = swerveDrive.getKinematics();
        AssertMK4iL3(modules);

        SwerveModule[] swerveModules = modules.m_modules;

        for (int i=0; i<4; i++) {
            swerveModules[i].driveMotor.close();
            swerveModules[i].turningMotor.close();
            swerveModules[i].absEncoder.close();
        }        
    }

    @Test
    public void startSwerveDriveTest() {

        RobotConfiguration robotConfiguration = RobotConfiguration.Create("swerveTest1", fn -> fn
            .Parts(inventory -> {
                Motors(inventory);
                Mk4iL3(inventory);
                return inventory;
            }
        ));
        robotConfiguration.Build(builder->builder
            .Subsystem(DriveSubsystem.class, sys->sys
                .Pigeon2(g->g
                    .CanNumber(14))

                .SwerveDrive(sd->sd
                    .Chassis(c -> c
                        .Square(19.75)
                    )
                    .SwerveModule("Module#1", sm->sm
                        .CanNumber(4)
                    )
                    .SwerveModule("Module#2", sm->sm
                        .CanNumber(6)
                    )
                    .SwerveModule("Module#3", sm->sm
                        .CanNumber(8)
                    )
                    .SwerveModule("Module#4", sm->sm
                        .CanNumber(10)
                    )
                )
            )
        );
        
        //Angle gyroAngle = Units.Degrees.of(0);
    
        RobotFactory factory = RobotFactory.Create(DriveSubsystem.class, robotConfiguration);
        DriveSubsystem driveSubsystem = factory.getInstance(DriveSubsystem.class);
        SwerveModule[] swerveModules = driveSubsystem.swerveDrive.swerveModules.m_modules;
        driveSubsystem.gyroYaw = ()->getAngle();
        driveSubsystem.swerveDrive.maxSpeed = 4.0;

        //TODO: measure voltage
        driveSubsystem.drive(1.0, 0, 0, false);
        for (int i=0; i<4; i++) {
            assertEquals(1.0, swerveModules[i].commandedSpeed); 
            assertEquals(0.0, swerveModules[i].commandedAngle); 
        }
        
        driveSubsystem.drive(1.0, 1.0, 0, false);
        for (int i=0; i<4; i++) {
            assertEquals(1.414, swerveModules[i].commandedSpeed, 0.001); 
            assertEquals(45.0, swerveModules[i].commandedAngle); 
        }
        driveSubsystem.drive(1.0, 1.0, 0, true);
        for (int i=0; i<4; i++) {
            assertEquals(1.414, swerveModules[i].commandedSpeed, 0.001); 
            assertEquals(45.0, swerveModules[i].commandedAngle); 
        }

        gyroAngle = Units.Degree.of(90);
        driveSubsystem.drive(1.0, 1.0, 0, true);
        for (int i=0; i<4; i++) {
            assertEquals(1.414, swerveModules[i].commandedSpeed, 0.001); 
            assertEquals(-45.0, swerveModules[i].commandedAngle); 
        }

        for (int i=0; i<4; i++) {
            swerveModules[i].driveMotor.close();
            swerveModules[i].turningMotor.close();
            swerveModules[i].absEncoder.close();
        }        
    }


    static Angle gyroAngle = Units.Degrees.of(0);
    public static Angle getAngle() {
        return gyroAngle;
    }

    public static void Motors(PartFactory inventory) {inventory
        .Motor(Motor.NEO, m -> m
            .MotorType(MotorType.kBrushless)
            .FreeSpeedRPM(5_820.0) // from MK4i docs, see data sheet for empirical values
        )
        .Motor(Motor.VORTEX, m -> m
            .MotorType(MotorType.kBrushless)
            .FreeSpeedRPM(6_784.0) // from REV
        )
        .Motor(Motor.NEO550, m -> m
            .MotorType(MotorType.kBrushless)
            .FreeSpeedRPM(11_000.0) // from REV
        );
    }

    public static void Mk4iL3(PartFactory inventory) {inventory
        .SwerveModule(sm -> sm
            .CANCoder(cc -> cc)
            .TurningMotor(Manufacturer.REVRobotics, mc -> mc
                .Motor(Motor.NEO550)
                .IdleMode(IdleMode.kCoast)
                .Reversed() // all turn motors are reversed
                .GearBox(g-> g
                    .Gear("Stage1", 14, 50)
                    .Gear("Stage2", 10, 60)
                )
                .AngleController(-180, 180)
                .PIDController(p->p
                    .Gain(3.4, 0.0, 0.0)
                    .EnableContinuousInput(-Math.PI, Math.PI)
                )
            )
            .DrivingMotor(Manufacturer.REVRobotics, mc -> mc
                .Motor(Motor.VORTEX)
                .IdleMode(IdleMode.kBrake)
                .Reversed() // all drive motors are reversed
                .GearBox(g-> g
                    .Gear("Stage1", 14, 50)
                    .Gear("Stage2", 28, 16)
                    .Gear("Stage3", 15, 45)
                    .Wheel(4.0)
                )
                .PID(.0005, 0.0, 0.0, 1.0)
                .ClosedLoopRampRate(.5)
                .SmartCurrentLimit(30)
            )
        );
    }
    
    public static void AssertMK4iL3(SwerveModule swerveModule) {
        double maxSpeed = swerveModule.maxSpeed; // 5.8946 m/s
        assertTrue(5.89 < maxSpeed);
        assertTrue(maxSpeed < 5.895);
        SwerveModuleState s1 = swerveModule.getState();
        swerveModule.setDesiredState(new SwerveModuleState(1.0, Rotation2d.kZero));
    }
    public static void AssertMK4iL3(SwerveModules swerveModules) {
        SwerveModuleState[] swerveModuleStates = swerveModules.getModuleStates();
    }
    // 19-3/4 25-1/4    
}
