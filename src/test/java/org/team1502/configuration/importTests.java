package org.team1502.configuration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.motors.Motor;
import org.team1502.configuration.builders.motors.MotorController;
import org.team1502.configuration.builders.motors.SwerveDrive;
import org.team1502.hardware.SwerveModule;
import org.team1502.hardware.SwerveModules;
import org.team1502.injection.RobotFactory;
import org.team1502.configuration.builders.motors.SwerveModuleBuilder;
import org.team1502.configuration.factory.Evaluator;
import org.team1502.configuration.factory.FactoryTestsBase;
import org.team1502.configuration.factory.PartFactory;
import org.team1502.configuration.factory.RobotConfiguration;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Written to test WPILib 2025 Beta 2 against new vendor APIs */
public class importTests {
    @Test
    public void importTest1() { // basic MotorController test
        final String REV_1 = "REV_1";
        final String SPARK_1 = "SPARK_1";

        // Add "templates" to Parts Factory
        var robotConfiguration = RobotConfiguration.Create("partTest1", fn -> fn
            // .Parts creates the robotBuilder and the partFactory
            // getFactory()->getBuilder().getPartFactory()
            .Parts(inventory -> inventory
                // Motor has a Define for the createFunction
                .Motor(REV_1, m->m // the buildFunction
                    .MotorType(MotorType.kBrushless)
                )
                .MotorController(SPARK_1, Manufacturer.REVRobotics, mc->mc
                    .Motor(REV_1)
                    .IdleMode(IdleMode.kBrake)
                    .Reversed()
                    .GearBox(g-> g
                        .Gear("Stage1", 10, 60)
                        .Gear("Stage2", 20, 10)
                        .Wheel(4.0)
                    )
                )
            )
        );

        // use Parts (templates) to create REV#1 "instance" of a MotorController
        robotConfiguration.Build(builder->builder
        //.Subsystem("Robot", r->r    )
            //.Value(MotorController.wheelDiameter, 4.0)
            .MotorController("REV#1", SPARK_1, mc1->mc1
                .CanNumber(1)
            )
        );

        Evaluator evaluator = robotConfiguration.Values();

        //var robot = evaluator.Part("Robot");
        var mc1 = evaluator.MotorController("REV#1");
        assertEquals(1, mc1.CanNumber());
        assertEquals("REVRobotics", mc1.CanInfo().Manufacturer().name());
        assertEquals("kBrake", mc1.IdleMode().name());

        var m1= mc1.Motor();
        assertEquals("kBrushless", m1.MotorType().name());

        FactoryTestsBase.DumpParts(robotConfiguration);

        SparkMax max = mc1.buildSparkMax();
        IdleMode idleMode = max.configAccessor.getIdleMode();
        boolean isInverted = max.configAccessor.getInverted();
        double positionFactor = max.configAccessor.encoder.getPositionConversionFactor();
        double velocityFactor = max.configAccessor.encoder.getVelocityConversionFactor();
        
        RelativeEncoder relativeEncoder = mc1.getRelativeEncoder();
        idleMode = max.configAccessor.getIdleMode();
        isInverted = max.configAccessor.getInverted();
        positionFactor = max.configAccessor.encoder.getPositionConversionFactor(); // PI/3 1.047 radians
        velocityFactor = max.configAccessor.encoder.getVelocityConversionFactor();
        double pos = relativeEncoder.getPosition(); // 1 rotation = diameter(1)[units] * PI * gear-ratio(1) [unists]
        double rpm = relativeEncoder.getVelocity();
    }

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
                    .Frame(25.25)
                    .WheelDiameter(4.0)
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
        
        SwerveDrive swerveDrive = evaluator.SwerveDrive();
        SwerveModules swerveModules = new SwerveModules(swerveDrive);
        SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
        double maxSpeed = swerveDrive.calculateMaxSpeed();
        //SwerveDrivePoseEstimator odometry = swerveDrive.getKinematics();
        AssertMK4iL3(swerveModules);
    }

    @Test
    public void startSwerveDriveTest() {

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
                    .Frame(25.25)
                    .WheelDiameter(4.0)
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

        RobotFactory factory = RobotFactory.Create(DriveSubsystem.class, robotConfiguration);

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
