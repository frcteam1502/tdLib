package org.team1502.configuration.builders.power;

import java.util.function.Function;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

/** Peak power and maybe eventually other power-related properties like "smart current" */
public class PowerProfile extends Builder {
    public static final String CLASSNAME = "PowerProfile";
    private static final String peakPower = "peakPower";
    public static final Function<IBuild, PowerProfile> Define = build->new PowerProfile(build);
    public static PowerProfile Wrap(Builder builder) { return builder == null ? null : new PowerProfile(builder.getIBuild(), builder.getPart()); }
    public static PowerProfile WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static PowerProfile WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }
    public PowerProfile(IBuild build) { super(build, CLASSNAME); }
    public PowerProfile(IBuild build, Part part) { super(build, part); }

    
    public Double PeakPower() { return getDouble(peakPower, 0.0); }
    public PowerProfile PeakPower(double number) {
        setValue(peakPower, number);
        return this;
    }

/*
    private static String channel = "channel";
    private static String wireLabel = "wireLabel";
    public Integer Channel() { return getInt(channel); }
    public PowerProfile Channel(int number) {
        setValue(channel, number); // TODO: update PDH
        return this;
    }
    
    public Double TotalPower() {
        var powered = getPieces().stream()
            .map(p->PowerProfile.WrapPart(p))
            .toList();
        var peaks = powered.stream().map(stage->stage.TotalPower());
        return PeakPower() + peaks.reduce(1.0, (stageA,stageB) -> stageA + stageB);
    }
  
    public PowerProfile AddPowered(Builder sink) {
        refPiece(sink);
        return this;
    }
 */
}
