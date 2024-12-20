package org.team1502.configuration.builders.motors;

import java.util.List;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.Chassis;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

public class SwerveDrive extends Builder {
    public static final String CLASSNAME = "SwerveDrive";
    public static final String goStraightGain = "goStraightGain";
    public static final String topSpeed = "topSpeed";
    public static Function<IBuild, SwerveDrive> Define = build->new SwerveDrive(build);
    public static SwerveDrive Wrap(Builder builder) { return builder == null ? null : new SwerveDrive(builder.getIBuild(), builder.getPart()); }
    public static SwerveDrive WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static SwerveDrive WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    public SwerveDrive(IBuild build) { super(build, CLASSNAME); }
    public SwerveDrive(IBuild build, Part part) { super(build, part); }

    public SwerveDrive SwerveModule(String name, Function<SwerveModuleBuilder, Builder> fn) {
        var module = addPiece(SwerveModuleBuilder.Define, name, SwerveModuleBuilder.CLASSNAME, fn);
        module.Value(SwerveModuleBuilder.location, getKinematic(getPieces().size()));
        //module.Value(MotorController.wheelDiameter, Chassis().getWheelDiameter());
        return this;
    }

    public Chassis Chassis() { return Chassis.WrapPart(this); }
    public SwerveDrive Chassis(Function<Chassis, Builder> fn) {
        var chassis = addPart(Chassis.Define, fn);
        Value(MotorController.wheelDiameter, chassis.getWheelDiameter());
        return this;
    }

    /** How fast to track target angle when not turning */
    public Double GoStraightGain() { return getDouble(goStraightGain); }
    public SwerveDrive GoStraightGain(double gain) {
        Value(goStraightGain, gain);
        return this;
    }

    public Double TopSpeed() { return getDouble(topSpeed); }
    public SwerveDrive TopSpeed(double metersPerSecond) {
        Value(topSpeed, metersPerSecond);
        return this;
    }

    /** max speed (m/s) based on reported free-speed */
    public double calculateMaxSpeed() { 
        return SwerveModuleBuilder.Wrap(getPiece(0)).calculateMaxSpeed();
    }

    /** max rotation speed (rad/s) based on reported free-speed */
    public double calculateMaxRotationSpeed() { 
        return calculateMaxSpeed() / Chassis().getDriveBaseRadius();
    }

    /** offset (m) */
    public Translation2d getKinematic(int moduleNumber) {
        return Chassis().getModuleLocation(moduleNumber);
    }
    public SwerveDriveKinematics getKinematics() {
        Translation2d[] kinematics = getPieces().stream()
            .map(module->(Translation2d)module.getValue(SwerveModuleBuilder.location))
            .toArray(Translation2d[]::new);
        return new SwerveDriveKinematics(kinematics);
    }

    public List<SwerveModuleBuilder> getModules() {
        var parts = getPieces();
        return parts.stream()
            .map(p->SwerveModuleBuilder.Wrap(p))
            .toList();
    }
    public SwerveModuleBuilder SwerveModule(String name) {
        return SwerveModuleBuilder.Wrap(getPieces().stream()
            .filter(p->p.Name() == name)
            .findFirst()
            .get());
    }
    /** NOTE: NOT an index! number is e.g., 1 through 4 */
    public SwerveModuleBuilder SwerveModule(int number) {
        return SwerveModuleBuilder.Wrap(getPieces().get(number - 1));
    }

}
