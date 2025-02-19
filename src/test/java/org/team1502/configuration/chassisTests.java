package org.team1502.configuration;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team1502.configuration.builders.Chassis;
import org.team1502.configuration.factory.RobotConfiguration;

public class chassisTests {
    @Test
    public void driveChassisTest() {
        var robotConfiguration = RobotConfiguration.Create("partTest1", fn -> fn
            .Build(inventory -> inventory
                .Subsystem("Drive", sys->sys
                    .Part("Mecanum", b->b.addPart(Chassis.Define, c->c
                        .Rectangular(Inches.of(6), Inches.of(8))))
                )
            )
        );

        var drive = robotConfiguration.Subsystem("Drive");
        var mecanum = drive.Part("Mecanum");
        var chassisPart = mecanum.Part("Chassis");
        var chassis = Chassis.WrapPart(mecanum);
        var radiusMeters = chassis.getDriveBaseRadius();
        //assertEquals(5, radius);
        assertEquals(0.127, radiusMeters);
    }
    
    @Test
    public void driveMecanumChassisTest() {
        var robotConfiguration = RobotConfiguration.Create("mecanumTest1", fn -> fn
            .Build(inventory -> inventory
                .Subsystem("Drive", sys->sys
                    .MecanumDrive(b->b
                        .Chassis(c->c
                            .Rectangular(Inches.of(6), Inches.of(8))))
                )
            )
        );

        var drive = robotConfiguration.Subsystem("Drive");
        var mecanum = drive.MecanumDrive();
        var chassis = mecanum.Chassis();
        var radiusMeters = chassis.getDriveBaseRadius();
        assertEquals(0.127, radiusMeters);
    }
}
