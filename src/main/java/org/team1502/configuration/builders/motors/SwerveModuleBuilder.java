package org.team1502.configuration.builders.motors;

import java.util.function.Function;

import com.ctre.phoenix6.hardware.CANcoder;

import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.Connector;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;
import org.team1502.configuration.builders.power.Power;

public class SwerveModuleBuilder extends Builder {
    public static final String CLASSNAME = "SwerveModule";
    /** offset (m) */
    public static final String location = "location";
    private static final String absoluteEncoder = "Encoder";
    private static final String turningMotor = "TurningMotor";
    private static final String drivingMotor = "DrivingMotor";
    private static final String isReversed = "isReversed";
    public static Function<IBuild, SwerveModuleBuilder> Define = build->new SwerveModuleBuilder(build);
    public static SwerveModuleBuilder Wrap(Builder builder) { return builder == null ? null : new SwerveModuleBuilder(builder.getIBuild(), builder.getPart()); }
    public static SwerveModuleBuilder WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static SwerveModuleBuilder WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    public SwerveModuleBuilder(IBuild build) { super(build, CLASSNAME); }
    public SwerveModuleBuilder(IBuild build, Part part) { super(build, part); }

    public SwerveModuleBuilder Wrap(Function<SwerveModuleBuilder, Builder> fn) {
        fn.apply(this);
        return this;
    }
    public CANCoder Encoder() {return CANCoder.WrapPart(this, SwerveModuleBuilder.absoluteEncoder);}
    public SwerveModuleBuilder Encoder(Function<CANCoder, Builder> fn) {
        fn.apply(Encoder());
        return this;
    }
    
    public SwerveModuleBuilder CANCoder(Function<CANCoder, Builder> fn) {
        addPart(CANCoder.Define, SwerveModuleBuilder.absoluteEncoder, fn);
        return this;
    }

    public MotorController TurningMotor() { return MotorController.WrapPart(this, SwerveModuleBuilder.turningMotor); }
    public SwerveModuleBuilder TurningMotor(Manufacturer manufacturer, Function<MotorController, Builder> fn) {
        addPart(MotorController.Define(manufacturer), SwerveModuleBuilder.turningMotor, fn);
        return this;
    }
    
    public MotorController DrivingMotor() { return MotorController.WrapPart(this, SwerveModuleBuilder.drivingMotor); }
    public SwerveModuleBuilder DrivingMotor(Manufacturer manufacturer, Function<MotorController, Builder> fn) {
        addPart(MotorController.Define(manufacturer), SwerveModuleBuilder.drivingMotor, fn);
        return this;
    }

    public boolean Reversed() { return getBoolean(SwerveModuleBuilder.isReversed, false); }
    public SwerveModuleBuilder Reversed(boolean value) {
        TurningMotor().Reversed(value);
        DrivingMotor().Reversed(value);
        return this;
    }
    
    /**
     * Root number of standard convention of CAN number and PDH channel
     */
    public SwerveModuleBuilder CanNumber(int rootNumber) { return CanNumbers(rootNumber, rootNumber, rootNumber+1); }
    public SwerveModuleBuilder CanNumberDown(int rootNumber) { return CanNumbers(rootNumber, rootNumber, rootNumber-1); }
    public int CanNumberEncoder() { return Encoder().CanNumber(); }
    public int CanNumberTurningMotor() { return TurningMotor().CanNumber(); }
    public int CanNumberDrivingMotor() { return DrivingMotor().CanNumber(); }
    public SwerveModuleBuilder CanNumbers(int absoluteEncoder, int turningMotor, int drivingMotor) {
        Encoder().CanNumber(absoluteEncoder);

        TurningMotor().CanNumber(turningMotor); 
        Connector.findConnector(TurningMotor(),Power.Signal).Label(turningMotor + " " + turningMotor + " " + turningMotor);
        TurningMotor().Abbreviation(Abbreviation()+"T");
        TurningMotor().FriendlyName((FriendlyName()+" Turning"));
        TurningMotor().PDH(turningMotor);
        
        DrivingMotor().CanNumber(drivingMotor);
        Connector.findConnector(DrivingMotor(), Power.Signal).Label(drivingMotor + " " + drivingMotor + " " + drivingMotor);
        DrivingMotor().Abbreviation(Abbreviation()+"D");
        DrivingMotor().FriendlyName((FriendlyName()+" Driving"));
        DrivingMotor().PDH(drivingMotor);

        return this;
    }

    /** in meters per second */
    public double calculateMaxSpeed() {
        return DrivingMotor().calculateMaxSpeed(); 
    }

    public ISwerveModule getSwerveModuleInstance() {
        return (ISwerveModule)Value("getSwerveModuleInstance");
    }

    public SwerveModuleBuilder setSwerveModuleInstance(ISwerveModule sm) {
        Value("getSwerveModuleInstance", sm);
        return this;
    }

    public CANcoder getCANcoder() {
        var encoder = Encoder();
        var canCoder = encoder.CANcoder();
        return (canCoder != null) ? canCoder : encoder.buildCANcoder();
    }
}
