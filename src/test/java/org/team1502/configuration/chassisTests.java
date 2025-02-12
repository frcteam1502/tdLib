package org.team1502.configuration;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.Chassis;
import org.team1502.configuration.factory.RobotConfiguration;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class chassisTests {
    @Test
    public void driveChassisTest() {
        var robotConfiguration = RobotConfiguration.Create("partTest1", fn -> fn
            .Build(inventory -> inventory
                .Subsystem("Drive", sys->sys
                    .Part("Mecanum", b->b.addPart(Chassis.Define, c->c
                        .Rectangular(6, 8)))
                )
            )
        );

        var drive = robotConfiguration.Subsystem("Drive");
        var mecanum = drive.Part("Mecanum");
        var chassisPart = mecanum.Part("Chassis");
        var chassis = Chassis.WrapPart(mecanum);
        var radius = chassis.DriveBaseRadius();
        var radiusMeters = chassis.getDriveBaseRadius();
        assertEquals(5, radius);
        assertEquals(0.127, radiusMeters);
    }
}
