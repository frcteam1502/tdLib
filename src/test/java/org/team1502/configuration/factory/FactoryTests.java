package org.team1502.configuration.factory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.Part;
import org.team1502.configuration.builders.TestBuilder;
import org.team1502.configuration.builders.motors.Motor;
import org.team1502.configuration.builders.power.Power;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FactoryTests extends FactoryTestsBase {
    @Test
    public void partTest() { // simple check to see if PartFactory can build a part
        RobotConfiguration robotConfiguration = getPartTest1();
        RobotBuilder robotBuilder = robotConfiguration.getBuilder();
        PartFactory partFactory = robotConfiguration.getFactory();
        PartBuilder<?> partFn1 = partFactory.getTemplate(MOTOR_MOTOR_1);
        Builder builder1 = partFn1.createBuilder(robotBuilder, "Part 1");
        
        Part part1 = builder1.getPart();
        assertEquals(Motor.CLASSNAME, part1.getType());
        assertEquals("Part 1", part1.getValue(Part.BUILD_NAME));
        assertEquals("Part 1", part1.getName());
        assertEquals("MOTOR_1", part1.getValue(Part.TEMPLATE_NAME));
        assertEquals(1, part1.getValue(Part.CREATED_NAME));
        assertEquals("kBrushless", ((MotorType)part1.getValue(Motor.motorType)).name());
        
        Part Vin = part1.getPart(Power.Vin);
        assertEquals(Power.Connector, Vin.getValue(Part.friendlyName));
        assertEquals(2, Vin.getValue(Part.CREATED_NAME));
        assertEquals(part1.getName() + "." + Vin.getName(), Vin.getKey());
        assertEquals("Part 1.Vin", Vin.getKey());

        ArrayList<Part> parts = robotBuilder.getParts(); // 2 parts (Vin created by Motor.Define)
        assertEquals(2, parts.size());
        DumpParts(robotConfiguration);
    }

    public static String MOTOR_MOTOR_1 = "MOTOR_1";
    public static String MOTOR_CONTROLLER_1 = "MOTOR_1";
    public static RobotConfiguration getPartTest1() {
        return RobotConfiguration.Create("partTest1", config -> config
            // .Parts creates the robotBuilder and the partFactory
            // getFactory()->getBuilder().getPartFactory()
            .Parts(inventory -> inventory
                // Motor has a Define for the createFunction
                .Motor(MOTOR_MOTOR_1, m -> m // the buildFunction
                    .MotorType(MotorType.kBrushless)
                )
                // partFactory has 1 function to create a Motor:Builder in it
            )
            // robotBuilder
        );
    }
}
