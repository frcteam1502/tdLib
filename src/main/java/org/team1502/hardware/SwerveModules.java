package org.team1502.hardware;

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
    private SwerveModule[] _modules;
    private String[] _moduleNames;
  
    public SwerveModules(SwerveDriveBuilder swerveDrive) { this(swerveDrive.getModules()); }
    public SwerveModules(List<SwerveModuleBuilder> swerveModules)
    {
        _modules = swerveModules.stream()
                    .map(m->new SwerveModule(m))
                    .toArray(SwerveModule[]::new);

        _moduleNames = swerveModules.stream()
                    .map(m->m.FriendlyName())
                    .toArray(String[]::new);
    }

    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
        for(int i = 0; i < swerveModuleStates.length; i++) {
            _modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(_modules)
            .map(m->m.getState())
            .toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(_modules)
            .map(m->m.getPosition())
            .toArray(SwerveModulePosition[]::new);
    }

    public void resetModules() {
        for(int i = 0; i < _modules.length; i++) {
            _modules[i].zeroModule();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModules");

        for(int i = 0; i < _modules.length; i++) {
            SendableRegistry.addLW(_modules[i], "SwerveModules", _moduleNames[i] );
        }
    }
    
    public void send() {
        SmartDashboard.putData(this);
        for(int i = 0; i < _modules.length; i++) {
            SmartDashboard.putData(_modules[i]);
        }
    }
}
