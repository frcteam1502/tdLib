package org.team1502.configuration.builders.motors;

import java.util.function.Function;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

public class Spark extends Builder { // PWMMotorController, e.g., Blinken controller
    private static final String isReversed = "isReversed";
    private static final String _spark_ = "_spark_";
    
    public static final String CLASSNAME = "Spark";
    public static final Function<IBuild, Spark> Define = build->new Spark(build);
    public static Spark Wrap(Builder builder) { return builder == null ? null : new Spark(builder.getIBuild(), builder.getPart()); }
    public static Spark WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static Spark WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }
    public Spark(IBuild build) {
         super(build, CLASSNAME);
    }

    public Spark(IBuild build, Part part) { super(build, part); }
    public boolean IsReversed() {
        return getBoolean(isReversed, false);
    }
    
    /** invert the direction of a speed controller */
    public Spark Reversed() { return Reversed(true); }
    public Spark Reversed(boolean value) {
        setValue(isReversed, value);
        return this;
    }

    
    // Spark Instance
    
    public edu.wpi.first.wpilibj.motorcontrol.Spark buildSpark() {
       return setSpark(new edu.wpi.first.wpilibj.motorcontrol.Spark(getPWMChannel())); 
    }

    public edu.wpi.first.wpilibj.motorcontrol.Spark getSpark() {
       return (edu.wpi.first.wpilibj.motorcontrol.Spark)(Value(_spark_)); 
    }
    public edu.wpi.first.wpilibj.motorcontrol.Spark setSpark(edu.wpi.first.wpilibj.motorcontrol.Spark spark) {
       Value(_spark_, spark); 
       return spark;
    }
    
    public double get() { return getSpark().get(); }
    public void set(double speed) {
        getSpark().set(speed);
    }

}
