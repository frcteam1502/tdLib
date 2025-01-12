package org.team1502.swerve;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

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
        var yawUnits = yaw.getUnits();
        var yawAngle = yaw.getValue();
        var yawMagnitude1 = yawAngle.magnitude();
        var yawMagnitude2 = yaw.getValueAsDouble();
        var position = absEncoder.getAbsolutePosition();
        
    }
}
