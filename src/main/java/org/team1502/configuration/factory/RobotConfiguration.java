package org.team1502.configuration.factory;

import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import org.team1502.configuration.builders.*;
import org.team1502.configuration.builders.motors.*;
import org.team1502.configuration.builders.pneumatics.*;
import org.team1502.configuration.builders.power.*;
import org.team1502.configuration.builders.sensors.*;

public class RobotConfiguration {

    public String name;
    private RobotBuilder _robotBuilder;
    private Evaluator _evaluator;
    
    public static RobotConfiguration Create(String name, Function<RobotConfiguration, RobotConfiguration> fn) {
        var robot = new RobotConfiguration();
        robot.name = name;
        return fn.apply(robot);
    }

    static RobotConfiguration Create(Function<RobotConfiguration, RobotConfiguration> fn) {
        var robot = new RobotConfiguration();
        return fn.apply(robot);

    }

    RobotConfiguration() { }
    private RobotConfiguration(RobotBuilder robotBuilder) {
        _robotBuilder = robotBuilder;
    }
    private HashMap<String, String> disabledMap = new HashMap<>();
    public RobotConfiguration DisableSubsystem(Class<?> subsystemclass) {
         return DisableSubsystem(subsystemclass.getName());
    }
    public RobotConfiguration DisableSubsystem(String className) {
        disabledMap.put(className, className);
        return this;
    }
    public boolean isDisabled(String clsName) {
        return disabledMap.containsKey(clsName);
    }
    
    RobotBuilder getBuilder() {
        if (_robotBuilder == null) {
            _robotBuilder = RobotBuilder.Create();
        }
        return _robotBuilder;
    }

    PartFactory getFactory() {
        return getBuilder().getPartFactory();
    }
    
    private Evaluator getEvaluator() {
        if (_evaluator == null) {
            _evaluator = new Evaluator(getBuilder());
        }
        return _evaluator;
    }

    //public CanMap getCanMap() {return getBuilder().getCanMap();}
    public PowerDistributionModule getPowerDistributionModule() {return getBuilder().getPowerDistributionModule();}

    public RobotConfiguration Parts(Function<PartFactory, PartFactory> fn) {
        fn.apply(getFactory());
        return this;
    }
    
    public RobotConfiguration Build(Function<RobotBuilder, RobotBuilder> fn) {
        fn.apply(getBuilder());
        return this;
    }

    public RobotConfiguration PowerDistributionModule(Function<PowerDistributionModule, Builder> fn) {
        fn.apply(getPowerDistributionModule());
        return this;
    }

    public Evaluator Values() {
        return getEvaluator();
    }

    public RobotConfiguration Values(Function<Evaluator, Evaluator> fn) {
        fn.apply(getEvaluator());
        return this;
    }

    public Object getValue(String valueName) {
        return Values().getValue(valueName);
    }

    public Object getValue(String valueName, String partName) {
        return Values().getValue(valueName, partName);
    }

    public <T> T Eval(Function<Evaluator,T> fn) {
        return Values().Eval(fn);
    }

    public Builder Part(String name) { return Values().Part(name); }
    public Builder findPart(String name) { return _robotBuilder.findInstalled(name); }

    public RobotConfiguration findSubsystemConfiguration(String partName) {
        var subsystem = findPart(partName);
        if (subsystem != null && subsystem.hasValue("robotBuilder")){
            return new RobotConfiguration((RobotBuilder)subsystem.Value("robotBuilder"));
        }
        return null;
    }

    public RobotConfiguration Subsystem(Class<?> subsystemClass) { return Subsystem(subsystemClass.getName()); }
    public RobotConfiguration Subsystem(String partName) { return new RobotConfiguration((RobotBuilder)Part(partName).Value("robotBuilder")); }
    public Object Value(String valueName) { return _robotBuilder.getPart().Value(valueName); }

    public MotorController MotorController() { return Values().MotorController(); }
    public MotorController MotorController(String name) { return Values().MotorController(name); }
    public Encoder Encoder() { return Encoder(Encoder.CLASSNAME); }
    public Encoder Encoder(String name) { return Values().Encoder(name); }
    public Solenoid Solenoid(String name) { return Solenoid.WrapPart(_robotBuilder.getPart(), name); }

    public edu.wpi.first.wpilibj.DigitalInput DigitalInput(String name) { 
        return new DigitalInput(_robotBuilder.getPart().getPart(name).getInt(RoboRIO.digitalInput));
    }

    public Chassis Chassis() { return Values().SwerveDrive().Chassis(); }
    public SwerveDriveBuilder SwerveDrive() { return Values().SwerveDrive(); }
    public SwerveModuleBuilder SwerveModule(String name) { return Values().SwerveDrive().SwerveModule(name); }
    public PowerDistributionModule MPM(String name) { return Values().MPM(name); }
    public PowerDistributionModule PDH() { return Values().PDH(); }
    public IMU Pigeon2() { return Values().Pigeon2(); }
    public GyroSensor GyroSensor() { return Values().GyroSensor(); }
    public GyroSensor GyroSensor(String name) { return (GyroSensor)getValue(name); }
    public Builder RadioPowerModule() { return Values().RadioPowerModule(); }
    public RoboRIO RoboRIO() { return Values().RoboRIO(); }
    public Builder RadioBarrelJack() { return Values().RadioBarrelJack(); }
    public Builder EthernetSwitch() { return Values().EthernetSwitch(); }
    public PneumaticsController PCM() { return Values().PCM(); }

    public void registerLoggerObjects(
            BiConsumer<String, SparkMax> motorLogger,
            Consumer<Pigeon2> pigeonLogger,
            BiConsumer<String, CANcoder> encoderLogger,
            BiConsumer<String, DoubleSupplier> sensorLogger
        ) {

        for (SwerveModuleBuilder sm : SwerveDrive().getModules()) {
            var drive = sm.DrivingMotor();
            motorLogger.accept(drive.FriendlyName() + " Drive", drive.CANSparkMax());    
        }
        for (SwerveModuleBuilder sm : SwerveDrive().getModules()) {
            var drive = sm.TurningMotor();
            motorLogger.accept(drive.FriendlyName() + " Turn", drive.CANSparkMax());    
        }
        
        pigeonLogger.accept(Pigeon2().Pigeon2());

        for (SwerveModuleBuilder sm : SwerveDrive().getModules()) {
            var drive = sm.Encoder();
            encoderLogger.accept(drive.FriendlyName() + " Abs", drive.CANcoder());    
        }

        for (SwerveModuleBuilder sm : SwerveDrive().getModules()) {
            sensorLogger.accept(sm.Abbreviation() + " Drive Speed", ()->sm.getSwerveModuleInstance().getVelocity());    
        }
    }

}
