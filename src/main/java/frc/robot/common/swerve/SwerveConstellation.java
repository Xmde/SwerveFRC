package frc.robot.common.swerve;

import java.security.InvalidParameterException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Represents a collection of swerve modules. */
public class SwerveConstellation implements Sendable {
    private final SwerveModule[] modules;
    private SwerveModuleState[] stopStates;
    public final SwerveDriveKinematics kinematics;
    public final SwerveDriveKinematicsConstraint kinematicsConstraint;
    public final double MAX_SPEED_METERS_PER_SECOND;

    public SwerveConstellation(SwerveModule... modules) {
        this.modules = modules;
        this.kinematics = SwerveModule.computeKinematics(modules);

        double minMax = Double.MAX_VALUE;
        for (SwerveModule module : modules) {
            minMax = Math.min(minMax, module.MAX_SPEED_METERS_PER_SECOND); 
        }
        MAX_SPEED_METERS_PER_SECOND = minMax;

        kinematicsConstraint = new SwerveDriveKinematicsConstraint(kinematics, MAX_SPEED_METERS_PER_SECOND);
    }

    protected void setModuleStates(ChassisSpeeds speeds, Translation2d centerOfRotation) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECOND);
        SwerveModule.setModuleStates(states, modules);
    }

    protected void setModuleStates(ChassisSpeeds speeds) {
        setModuleStates(speeds, new Translation2d());
    }
    protected void stopModules() {
        if (stopStates == null) {
            stopStates = new SwerveModuleState[modules.length];

            if (modules.length == 4) {
                stopStates = new SwerveModuleState[]{
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                };
            } else {
                for (int i = 0; i < modules.length; i++) {
                    stopStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(i % 2 == 0 ? 45: -45));
                }
            }
        }
        SwerveModule.setModuleStates(stopStates, modules);
    }

    public void setStopStates(SwerveModuleState... states) {
        if (states.length != modules.length) throw new InvalidParameterException("The number of states must equal the number of modules");
        stopStates = states;
    }

    public ChassisSpeeds chassisSpeeds() {
        return SwerveModule.computeChassisSpeeds(kinematics, modules);
    }

    public SwerveModuleState[] moduleStates() {
        return SwerveModule.computeModuleStates(modules);
    }

    public SwerveModulePosition[] modulePositions() {
        return SwerveModule.computeModulePositions(modules);
    }

    protected void recalibrate() {
        SwerveModule.recalibrate(modules);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        for (int i = 0; i < modules.length; i++) {
            final int index = i;
            builder.addDoubleProperty("Module " + i + " Angle (Real Deg)", () -> modules[index].getState().angle.getDegrees(), null);
            builder.addDoubleProperty("Module " + i + " Speed (Real MPS)", () -> modules[index].getState().speedMetersPerSecond, null);
        }        
    }
}
