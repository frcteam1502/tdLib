package org.team1502.configuration;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.factory.Evaluator;
import org.team1502.configuration.factory.FactoryTestsBase;
import org.team1502.configuration.factory.RobotConfiguration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Written to test WPILib 2025 Beta 2 against new vendor APIs */
public class importTests {
    @Test
    public void importTest1() { // basic MotorController test
        final String REV_1 = "REV_1";
        final String SPARK_1 = "SPARK_1";

        // Add "templates" to Parts Factory
        var robotConfiguration = RobotConfiguration.Create("partTest1", fn -> fn
            // .Parts creates the robotBuilder and the partFactory
            // getFactory()->getBuilder().getPartFactory()
            .Parts(inventory -> inventory
                // Motor has a Define for the createFunction
                .Motor(REV_1, m->m // the buildFunction
                    .MotorType(MotorType.kBrushless)
                )
                .MotorController(SPARK_1, Manufacturer.REVRobotics, mc->mc
                    .Motor(REV_1)
                    .IdleMode(IdleMode.kBrake)
                    .Reversed()
                    .GearBox(g-> g
                        .Gear("Stage1", 10, 60)
                        .Gear("Stage2", 20, 10)
                        .Wheel(Inches.of(4.0))
                    )
                )
            )
        );

        // use Parts (templates) to create REV#1 "instance" of a MotorController
        robotConfiguration.Build(builder->builder
        //.Subsystem("Robot", r->r    )
            //.Value(MotorController.wheelDiameter, 4.0)
            .MotorController("REV#1", SPARK_1, mc1->mc1
                .CanNumber(1)
            )
        );

        Evaluator evaluator = robotConfiguration.Values();

        //var robot = evaluator.Part("Robot");
        var mc1 = evaluator.MotorController("REV#1");
        assertEquals(1, mc1.CanNumber());
        assertEquals("REVRobotics", mc1.CanInfo().Manufacturer().name());
        assertEquals("kBrake", mc1.IdleMode().name());

        var m1= mc1.Motor();
        assertEquals("kBrushless", m1.MotorType().name());

        FactoryTestsBase.DumpParts(robotConfiguration);

        SparkMax max = mc1.buildSparkMax();
        IdleMode idleMode = max.configAccessor.getIdleMode();
        boolean isInverted = max.configAccessor.getInverted();
        double positionFactor = max.configAccessor.encoder.getPositionConversionFactor();
        double velocityFactor = max.configAccessor.encoder.getVelocityConversionFactor();
        
        RelativeEncoder relativeEncoder = mc1.getRelativeEncoder();
        idleMode = max.configAccessor.getIdleMode();
        isInverted = max.configAccessor.getInverted();
        positionFactor = max.configAccessor.encoder.getPositionConversionFactor(); // PI/3 1.047 radians
        velocityFactor = max.configAccessor.encoder.getVelocityConversionFactor();
        double pos = relativeEncoder.getPosition(); // 1 rotation = diameter(1)[units] * PI * gear-ratio(1) [unists]
        double rpm = relativeEncoder.getVelocity();
    }

}
