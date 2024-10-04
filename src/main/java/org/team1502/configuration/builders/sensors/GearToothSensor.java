package org.team1502.configuration.builders.sensors;

import java.util.function.Function;

import org.team1502.configuration.CAN.CanInfo;
import org.team1502.configuration.CAN.DeviceType;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

public class GearToothSensor extends Builder {
    private static final DeviceType deviceType = DeviceType.GearToothSensor;

    public static final Function<IBuild, GearToothSensor> Define(Manufacturer manufacturer) {
        return build->new GearToothSensor(build, manufacturer);
    } 
    public static GearToothSensor Wrap(Builder builder) { return builder == null ? null : new GearToothSensor(builder.getIBuild(), builder.getPart()); }
    public static GearToothSensor WrapPart(Builder builder) { return WrapPart(builder, deviceType.name()); }
    public static GearToothSensor WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }
    
    public GearToothSensor(IBuild build, Manufacturer manufacturer) {
        super(build, deviceType); 
        CanInfo.addConnector(this, deviceType, manufacturer);
    }

    public GearToothSensor(IBuild build, Part part) { super(build, part); }

}