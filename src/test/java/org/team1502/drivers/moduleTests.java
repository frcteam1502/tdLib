package org.team1502.drivers;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.hal.HAL;

public class moduleTests {
    private CANcoder absEncoder;
    private Pigeon2 imu;
    private CANcoderSimState absEncoderSim;
    private Pigeon2SimState imuSim;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        absEncoder = new CANcoder(1);
        imu = new Pigeon2(0);
        absEncoderSim = absEncoder.getSimState();
        /*
         * 
         */
         absEncoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
            // .withMagnetOffset(0.25)
            // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1));
            imuSim = imu.getSimState();
        }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach // this method will run after each test
    void shutdown() throws Exception {
        absEncoder.close();
        imu.close();
    }


    @Test
    public void coldStartTest() {
        var yaw = imu.getYaw();
        imuSim.setRawYaw(-90); // docs/comments indicates initial yaw is -90 
        yaw = imu.getYaw(true);
        var yawUnits = yaw.getUnits();
        var yawAngle = yaw.getValue();
        var yawMagnitude1 = yawAngle.magnitude();
        var yawMagnitude2 = yaw.getValueAsDouble(); // same

        // rotot on blocks, manually move the rotation
        var position = absEncoder.getAbsolutePosition();
        absEncoderSim.addPosition(0.25); // e.g., move it to align with +X 0 degrees
        position = absEncoder.getPosition(true);
        absEncoderSim.addPosition(0.25); // e.g., move it to align with +X 0 degrees
        position = absEncoder.getPositionSinceBoot(true);
        position = absEncoder.getAbsolutePosition();
        //absEncoderSim.setRawPosition(1.0);

    }
}
