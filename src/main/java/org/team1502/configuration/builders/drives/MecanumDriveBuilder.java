package org.team1502.configuration.builders.drives;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.List;
import java.util.function.*;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.Chassis;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;
import org.team1502.configuration.builders.motors.MotorControllerBuilder;
import org.team1502.configuration.builders.motors.PID;
import org.team1502.drivers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class MecanumDriveBuilder extends Builder {
    public static final String CLASSNAME = "MecanumDrive";
    public static Function<IBuild, MecanumDriveBuilder> Define = build->new MecanumDriveBuilder(build);
    public static MecanumDriveBuilder Wrap(Builder builder) { return builder == null ? null : new MecanumDriveBuilder(builder.getIBuild(), builder.getPart()); }
    public static MecanumDriveBuilder WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static MecanumDriveBuilder WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    public MecanumDriveBuilder(IBuild build) { super(build, CLASSNAME); }
    /** PartFactory doesn't need to see this one */
    private MecanumDriveBuilder(IBuild build, Part part) { super(build, part); }

    // public MecanumDriveBuilder MecanumModule(String name, Function<MecanumModuleBuilder, Builder> fn) {
    //     var module = addPiece(MecanumModuleBuilder.Define, name, MecanumModuleBuilder.CLASSNAME, fn);
    //     module.Value(MecanumModuleBuilder.location, getKinematic(getPieces().size()));
    //     //module.Value(MotorController.wheelDiameter, Chassis().getWheelDiameter());
    //     return this;
    // }

    public MecanumDriver buildDriver(Supplier<Rotation2d> yawSupplier) { return new MecanumDriver(this, yawSupplier); }
    
    public Chassis Chassis() { return Chassis.WrapPart(this); }
    public MecanumDriveBuilder Chassis(Function<Chassis, Builder> fn) {
        addPart(Chassis.Define, fn);
        return this;
    }
    public MotorControllerBuilder MotorController(String name) { return MotorControllerBuilder.WrapPart(this, name); }
    public MecanumDriveBuilder MotorController(String name, String partName, Function<MotorControllerBuilder, Builder> fn) {
        addPiece(name, partName, fn);
        return this;
    }

    public PID PIDController(String name) { return PID.WrapPart(this, name); }
    public MecanumDriveBuilder PIDController(String name, Function<PID, Builder> fn) {
        return (MecanumDriveBuilder)AddPart(name, PID.Define, fn);
    }

    public MecanumDriveBuilder TrajectoryConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
        return TrajectoryConfig(maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond));
    }
    /** m/s and m/s/s */
    public MecanumDriveBuilder TrajectoryConfig(double maxVelocity, double maxAcceleration) {
        setValue("maxVelocity", maxVelocity);
        setValue("maxAcceleration", maxAcceleration);
        return this;
    }

    /** max speed (m/s) based on reported free-speed */
    public double calculateMaxSpeed() { 
        return MotorControllerBuilder.Wrap(getPiece(0)).calculateMaxSpeed();
    }

    /** max rotation speed (rad/s) based on reported free-speed */
    public double calculateMaxRotationSpeed() { 
        return calculateMaxSpeed() / Chassis().getDriveBaseRadius();
    }

    public Translation2d getKinematic(int moduleNumber) {
        return Chassis().getModuleLocation(moduleNumber);
    }

    public MecanumDriveKinematics getMecanumDriveKinematics() {
        return Chassis().getMecanumDriveKinematics();
    }

    public List<MotorControllerBuilder> getModules() {
        var parts = getPieces();
        return parts.stream()
            .map(p->MotorControllerBuilder.Wrap(p))
            .toList();
    }

    public MecanumDrive getMecanumDrive() {
        var modules = getModules();
        return new MecanumDrive(modules.get(0), modules.get(2), modules.get(1), modules.get(3));        
    }

}
