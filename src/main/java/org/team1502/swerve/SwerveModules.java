package org.team1502.swerve;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team1502.configuration.builders.motors.SwerveDriveBuilder;
import org.team1502.configuration.builders.motors.SwerveModuleBuilder;

public class SwerveModules implements Sendable {
    SwerveModule[] m_modules;
    String[] m_moduleNames;
  
    public SwerveModules(SwerveDriveBuilder swerveDrive) { this(swerveDrive.getModules()); }
    public SwerveModules(List<SwerveModuleBuilder> swerveModules)
    {
        m_modules = swerveModules.stream()
                    .map(m->new SwerveModule(m))
                    .toArray(SwerveModule[]::new);

        m_moduleNames = swerveModules.stream()
                    .map(m->m.FriendlyName())
                    .toArray(String[]::new);
    }

    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
        for(int i = 0; i < swerveModuleStates.length; i++) {
            m_modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(m_modules)
            .map(m->m.getState())
            .toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(m_modules)
            .map(m->m.getPosition())
            .toArray(SwerveModulePosition[]::new);
    }

    public void resetModules() {
        for(int i = 0; i < m_modules.length; i++) {
            m_modules[i].zeroModule();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModules");

        for(int i = 0; i < m_modules.length; i++) {
            SendableRegistry.addLW(m_modules[i], "SwerveModules", m_moduleNames[i] );
        }
    }
    
    public void send() {
        SmartDashboard.putData(this);
        for(int i = 0; i < m_modules.length; i++) {
            SmartDashboard.putData(m_modules[i]);
        }
    }
}
