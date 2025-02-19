package org.team1502.configuration.builders.motors;

import java.util.function.Function;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

public class PID extends Builder {
    private static final String NAME = "PID";
    private static final String pid_controller = "pid_controller";
    public static Function<IBuild, PID> Define = b->new PID(b);
    public static PID Wrap(Builder builder) { return builder == null ? null : new PID(builder.getIBuild(), builder.getPart()); }
    public static PID WrapPart(Builder builder) { return WrapPart(builder, NAME); }
    public static PID WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    public PID(IBuild build) { super(build, NAME); }
    public PID(IBuild build, Part part) { super(build, part); }

    public double P() {return getDouble("p"); }
    public PID P(double p) {return (PID)setValue("p", p); }

    public double I() {return getDouble("i"); }
    public PID I(double p) {return (PID)setValue("i", p); }

    public double D() {return getDouble("d"); }
    public PID D(double p) {return (PID)setValue("d", p); }

    public double FF() {return getDouble("ff"); }
    public PID FF(double p) {return (PID)setValue("ff", p); }

    public PID Gain(double p, double i, double d, double ff) {
        Gain(p,i,d);
        FF(ff);
        return this;
    }
    public PID Gain(double p, double i, double d) {
        P(p); 
        I(i);
        D(d);
        return this;
    }
    public PID OutputRange(double min, double max) {
        setValue("min", min); 
        setValue("max", max); 
        return this;
    }
    public PID Constraints(double maxVelocity, double maxAcceleration) {
        setValue("maxVelocity", maxVelocity); 
        setValue("maxAcceleration", maxAcceleration); 
        return this;
    }
    public PID EnableContinuousInput(double minimumInput, double maximumInput) {
        setValue("minimumInput", minimumInput); 
        setValue("maximumInput", maximumInput); 
        return this;
    }

    public PIDController getPIDController() {
        var pidController = (PIDController)getValue(pid_controller);
        return (pidController != null) ? pidController : buildPIDController();
    }
    public PIDController buildPIDController() {
        var pidController = new PIDController(P(), I(), D());
        if (hasValue("minimumInput") && hasValue("maximumInput")) {
            pidController.enableContinuousInput(getDouble("minimumInput"), getDouble("maximumInput"));
        }
        setValue(pid_controller, pidController);
        return pidController;
    }
    public ProfiledPIDController getProfiledPIDController() {
        var pidController = (ProfiledPIDController)getValue(pid_controller);
        return (pidController != null) ? pidController : buildProfiledPIDController();
    }
    public ProfiledPIDController buildProfiledPIDController() {
        var pidController = new ProfiledPIDController(P(), I(), D(),
                                new TrapezoidProfile.Constraints(
                                    getDouble("maxVelocity"),
                                    getDouble("maxAcceleration")));
        setValue(pid_controller, pidController);
        return pidController;
    }

    public SparkMaxConfig setPIDController(SparkMaxConfig config) {
        config.closedLoop.pid(P(),I(), D());
        if (hasValue("ff")) {
            config.closedLoop.velocityFF(FF());
        }
        if (hasValue("min") && hasValue("max")) {
            config.closedLoop.outputRange(getDouble("min"), getDouble("max"));
        }
        return config;
    }
    /*
     * 
    public SparkMaxConfig setPIDController(SparkMax motor) {
        return setPIDController(new SparkMaxConfig())
    }
    public SparkClosedLoopController setPIDController(SparkMax motor) {
        var pidController = motor.getClosedLoopController();
        setPIDController(pidController);
        return pidController;
    }
    
    public void setPIDController(SparkClosedLoopController pidController) {
        pidController.setP(P());
        pidController.setI(I());
        pidController.setD(D());
        if (hasValue("ff")) {
            pidController.setFF(FF());
        }
        if (hasValue("min") && hasValue("max")) {
            pidController.setOutputRange(getDouble("min"), getDouble("max"));
        }
    
    }
     */

}
/* NOTE: 4 PID slots (e.g., per reference?)

  public enum AccelStrategy {
    kTrapezoidal(0),
    kSCurve(1);
  public enum ArbFFUnits {
    kVoltage(0),
    kPercentOut(1);


    public enum ControlType {
        kDutyCycle(0),
        kVelocity(1),
        kVoltage(2),
        kPosition(3),
        kSmartMotion(4),
        kCurrent(5),
        kSmartVelocity(6);

        @SuppressWarnings("MemberName")
        public final int value;

        ControlType(int value) {
        this.value = value;
        }
  }

 */