package org.team1502.configuration.builders.sensors;

import java.util.function.Function;

import org.team1502.configuration.CAN.CanInfo;
import org.team1502.configuration.CAN.DeviceType;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

public class Accelerometer extends Builder {
    private static final DeviceType deviceType = DeviceType.Accelerometer;

    public static final Function<IBuild, Accelerometer> Define(Manufacturer manufacturer) {
        return build->new Accelerometer(build, manufacturer);
    } 
    public static Accelerometer Wrap(Builder builder) { return builder == null ? null : new Accelerometer(builder.getIBuild(), builder.getPart()); }
    public static Accelerometer WrapPart(Builder builder) { return WrapPart(builder, deviceType.name()); }
    public static Accelerometer WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }
    
    public Accelerometer(IBuild build, Manufacturer manufacturer) {
        super(build, deviceType); 
        CanInfo.addConnector(this, deviceType, manufacturer);
    }
    
    public Accelerometer(IBuild build, Part part) { super(build, part); }


}
