// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class SwerveConstellation implements Sendable {
    public final SwerveModule[] modules;
    public final SwerveDriveKinematics kinematics;
    public final SwerveDriveKinematicsConstraint kinematicsConstraint;
    public final double MAX_SPEED_METERS_PER_SECOND;

    public SwerveConstellation(SwerveModule ...modules) {
        this.modules = modules;
        this.kinematics = SwerveModule.computeKinematics(modules);

        double minMax = 0;
        for (SwerveModule module : modules) {
            minMax = Math.min(minMax, module.MAX_SPEED_METERS_PER_SECOND); 
        }
        MAX_SPEED_METERS_PER_SECOND = minMax;

        kinematicsConstraint = new SwerveDriveKinematicsConstraint(kinematics, MAX_SPEED_METERS_PER_SECOND);
    }

    public void setModuleStates(ChassisSpeeds speeds, Translation2d centerOfRotation) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECOND);
        SwerveModule.setModuleStates(states, modules);    }

    public void setModuleStates(ChassisSpeeds speeds) {
        setModuleStates(speeds, new Translation2d());
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

    public void recalibrate() {
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
