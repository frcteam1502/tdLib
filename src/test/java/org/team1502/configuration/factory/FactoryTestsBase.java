package org.team1502.configuration.factory;

import org.team1502.configuration.builders.TestBuilder;

public class FactoryTestsBase {
    public static void DumpParts(RobotConfiguration robotConfiguration) {
        TestBuilder testBuilder = new TestBuilder(robotConfiguration.getBuilder().getParts());
        testBuilder.DumpParts();
    }
}
