package org.team1502.configuration.builders.motors;

import java.util.function.Function;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

import edu.wpi.first.math.util.Units;

public class GearBox extends Builder{
    private static final String NAME = "GearBox";
    public static Function<IBuild, GearBox> Define = b->new GearBox(b);
    public static GearBox Wrap(Builder builder) { return builder == null ? null : new GearBox(builder.getIBuild(), builder.getPart()); }
    public static GearBox WrapPart(Builder builder) { return WrapPart(builder, NAME); }
    public static GearBox WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

    public GearBox(IBuild build) { super(build, NAME); }
    public GearBox(IBuild build, Part part) { super(build, part); }

    public GearBox Gear(String stage, int drivingTeeth, int drivenTeeth) {
        Piece(Gear.Define, stage, g->g
            .DrivingTeeth(drivingTeeth)
            .DrivenTeeth(drivenTeeth));
        return this;
    }
    public boolean hasActuator() {
        return hasValue(MotorController.wheelDiameter);
    }

    /** size of actuator attached to gearbox (inches), e.g., wheel arm */
    public GearBox Wheel(double diameter) {
        Value(MotorController.wheelDiameter, diameter);
        return this;
    }
    
    /** all stages driving-teeth / driven-teeth (plus actuator) */
    public double GearRatio() {
        var stages = getPieces();
        var ratios = stages.stream().map(stage->stage.getDoubleFromInt(Gear.drivingTeeth)/stage.getDoubleFromInt(Gear.drivenTeeth));
        double gearRatio = ratios.reduce(1.0, (stageA,stageB) -> stageA * stageB);
        if (hasActuator()) {
            gearRatio *= Math.PI * Units.inchesToMeters(getDouble(MotorController.wheelDiameter));
        }
        return gearRatio;
    }
}
