package org.team1502.configuration.builders.motors;

import java.util.function.BiConsumer;
import java.util.function.Function;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

import com.revrobotics.RelativeEncoder;

import org.team1502.configuration.CAN.DeviceType;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;
import org.team1502.configuration.builders.power.Power;

public class MotorController extends Builder {
    private static final DeviceType deviceType = DeviceType.MotorController; 
    public static final String CLASSNAME = "MotorController";

    private static final String isReversed = "isReversed";
    private static final String closedLoopRampRate = "closedLoopRampRate";
    private static final String smartCurrentLimit = "smartCurrentLimit";
    /** Wheel Diameter (in) */
    public static final String wheelDiameter = "wheelDiameter";
    public static final String isAngleController = "isAngleController";
    public static final String angleMin = "angleMin";
    public static final String angleMax = "angleMax";
    public static final String hasPID = "hasPID"; // has config for built-in pid
    
    public static final Function<IBuild, MotorController> Define(Manufacturer manufacturer) {
        return build->new MotorController(build,manufacturer);
    }
    public static MotorController Wrap(Builder builder) { return builder == null ? null : new MotorController(builder.getIBuild(), builder.getPart()); }
    public static MotorController WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static MotorController WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    // Define
    public MotorController(IBuild build, Manufacturer manufacturer) {
        super(build, deviceType, manufacturer);
        addConnector(Power.Signal, Power.Vin).FriendlyName(Power.Connector);
        addChannel(Power.Signal, Power.Vout).FriendlyName("Motor Power Out");
    }
    public MotorController(IBuild build, Part part) {
        super(build, part);
    }
    
    public Motor Motor() {return Motor.WrapPart(this, Motor.CLASSNAME); }
    public MotorController Motor(String partName) {
        return Motor(partName, m->m);
    }
    public MotorController Motor(String partName, Function<Motor, Builder> fn) {
        var motor = addPart(Motor.Define, Motor.CLASSNAME, partName, fn);
        this.Powers(motor);
        return this;
    }
    
    public MotorController Follower() { return WrapPart(this, "Follower"); }
    public MotorController Follower(String partName, Function<MotorController, Builder> fn) {
        addPart(MotorController.Define(Manufacturer.REVRobotics), "Follower", partName, fn);
        return this;
    }


    public IdleMode IdleMode() { return (IdleMode)getValue(Motor.idleMode); }
    public MotorController IdleMode(IdleMode value) {
        setValue(Motor.idleMode, value);
        return this;
    }
    
    public boolean IsReversed() {
        return getBoolean(isReversed, false);
    }
    
    /** invert the direction of a speed controller */
    public MotorController Reversed() { return Reversed(true); }
    public MotorController Reversed(boolean value) {
        setValue(isReversed, value);
        return this;
    }

    /** A rotating/revolving actuator with a diameter in inches -- used for position conversion */
    public MotorController Wheel(double diameter) {
        Value(wheelDiameter, diameter);
        return this;
    }
    public GearBox GearBox() { return GearBox.WrapPart(this); }
    public MotorController GearBox(Function<GearBox, Builder> fn) {
        return (MotorController)AddPart(GearBox.Define, fn);
    }

    public MotorController AngleController() { return AngleController(-180, 180); }
    public MotorController AngleController(double minAngle, double maxAngle) {
        Value(MotorController.isAngleController, true);
        Value(MotorController.angleMin, minAngle);
        Value(MotorController.angleMax, maxAngle);
        return this;
    }
    
    public PID PID() { return PID.WrapPart(this); }
    /** PID values for built-in CLosed Loop Controller ONLY*/
    public MotorController PID(Function<PID, Builder> fn) {
        Value(hasPID, true);
        return (MotorController)AddPart(PID.Define, fn);
    }
    /** Define a WPIlib PIDController */
    public MotorController PIDController(Function<PID, Builder> fn) {
        return (MotorController)AddPart(PID.Define, fn);
    }
    /** PID values for built-in CLosed Loop Controller ONLY*/
    public MotorController PID(double p, double i, double d) {
        return PID(pid->pid.P(p).I(i).D(d));
    }
    /** PID values for built-in CLosed Loop Controller ONLY */
    public MotorController PID(double p, double i, double d, double ff) {
        return PID(pid->pid.P(p).I(i).D(d).FF(ff));
    }

    /** Time in seconds to go from 0 to full throttle. */
    public Double ClosedLoopRampRate() { return getDouble(closedLoopRampRate); }
    public MotorController ClosedLoopRampRate(double rate) {
        Value(closedLoopRampRate, rate);
        return this;
    }
    /** The current limit in Amps. */
    public Integer SmartCurrentLimit() { return getInt(smartCurrentLimit); }
    public MotorController SmartCurrentLimit(Integer limit) {
        Value(smartCurrentLimit, limit);
        return this;
    }

    // public SparkClosedLoopController buildPIDController(MotorFeedbackSensor feedbackDevice) {
    //     var pid = PID().setPIDController(CANSparkMax());
    //     pid.setFeedbackDevice(feedbackDevice);
    //     return pid;
    // }
    public SparkClosedLoopController getPIDController() {
        //return PID().setPIDController(CANSparkMax());
        return CANSparkMax().getClosedLoopController();
    }

    public SparkMax buildSparkMax() {
        var max = CANSparkMax(new SparkMax(CanNumber(), Motor().MotorType()));
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode());
        config.inverted(IsReversed());
        if (hasValue(closedLoopRampRate)) {
            config.closedLoopRampRate(ClosedLoopRampRate());
        }
        if (hasValue(smartCurrentLimit)) {
            config.smartCurrentLimit(SmartCurrentLimit());
        }
        if (getBoolean(hasPID, false)){
            PID().setPIDController(config);
        }

        // NOTE: ensure this fails gracefully if no factors present
        // and doesn't have any other side effects if no conversion it intended
        config.encoder.positionConversionFactor(getPositionConversionFactor());
        config.encoder.velocityConversionFactor(getVelocityConversionFactor());

        //TODO: test Follower logic;
        if (hasValue("Follower")) {
            var follower = Follower();
            var followerMotor = follower.buildSparkMax();
            config.follow(followerMotor, follower.IsReversed());
        }
        
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return max;
    }
    public SparkMax CANSparkMax() {
        return (SparkMax)Value("CANSparkMax");
    }
    public SparkMax CANSparkMax(SparkMax motor) {
        Value("CANSparkMax", motor);
        return motor;
    }

    public RelativeEncoder getRelativeEncoder() {
        return CANSparkMax().getEncoder();
    }
    public double getPosition() { return getRelativeEncoder().getPosition(); }
    public RelativeEncoder setPosition(double position) {
        CANSparkMax().getClosedLoopController().setReference(position, ControlType.kPosition);
        return getRelativeEncoder(); 
    }

    SwerveModuleBuilder getSwerveModule() {
        var parent = getParentOfType(SwerveModuleBuilder.CLASSNAME);
        return parent == null ? null : SwerveModuleBuilder.Wrap(parent);
    }

    /** add encoder conversion factors to existing configuration 
    public RelativeEncoder buildRelativeEncoder() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder.positionConversionFactor(getPositionConversionFactor());
        config.encoder.velocityConversionFactor(getVelocityConversionFactor());

        SparkMax max = CANSparkMax();
        max.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        return getRelativeEncoder();
    }
    */

    /** mpr * 60 = position/minute (like rpm)  */
    public double getVelocityConversionFactor() { return getPositionConversionFactor()/60;  }

    /** radians for angles, meters for linear */
    public double getPositionConversionFactor() {
        return getTotalEncoderConversionFactor(); 
    }

    /** GearBox can have an optional wheel/actuator as a kind of final stage */
    private double getTotalEncoderConversionFactor() {
        double ratio = 1.0;
        if (GearBox() != null) {
            ratio = getGearRatio();
            if (GearBox().hasActuator()) {
                return ratio; //all done and accounted for
            }
        }
        if (getBoolean(MotorController.isAngleController, false)) {
            return 2 * Math.PI * ratio;
        }
        // at this point we have a gear-ratio or 1.0, but for position to make sense we need a length
        double length = getWheelDiameter() * Math.PI;
        return ratio * length;
    }

    private double getGearRatio() { return GearBox() != null ? GearBox().GearRatio() : 1.0; }

    /** meters (1) */
    private double getWheelDiameter() {
        double inches = findDouble(MotorController.wheelDiameter, Double.NaN);
        return Double.isNaN(inches) ? 1.0 : Units.inchesToMeters(inches);
    }

    /** meters/second */
    public double calculateMaxSpeed() {
        return Motor().FreeSpeedRPM() / 60.0 * getTotalEncoderConversionFactor();
    }
    /** in wheel diameter units/second */
    public double calculateMaxSpeed(Double wheelDiameter) {
        return Motor().FreeSpeedRPM() / 60.0
                * getGearRatio()
                * wheelDiameter * Math.PI; 
    }

    public void registerLoggerObjects(BiConsumer<String, SparkMax> motorLogger) {
        motorLogger.accept(FriendlyName(), CANSparkMax());
        if (hasValue("Follower")) {
            Follower().registerLoggerObjects(motorLogger);
        }
    }

}
