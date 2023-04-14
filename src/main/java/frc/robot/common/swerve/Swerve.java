// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A swerve drivetrain subsystem will extend this class. */
public abstract class Swerve extends SubsystemBase {
    protected abstract SwerveConstellation getConstellation();
    public abstract Rotation2d getGyroscopeRotation();

    protected ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    protected SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        getConstellation().kinematics,
        getGyroscopeRotation(),
        getConstellation().modulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.02, 0.02, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.01)
    );

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getConstellation().modulePositions(), pose);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return getConstellation().chassisSpeeds();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SwerveConstellation constellation = getConstellation();
        poseEstimator.update(getGyroscopeRotation(), constellation.modulePositions());
        constellation.setModuleStates(chassisSpeeds);
        constellation.recalibrate();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Odometry Heading (Deg)", () -> getPose().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Velocity X", () -> getCurrentVelocity().vxMetersPerSecond, null);
        builder.addDoubleProperty("Velocity Y", () -> getCurrentVelocity().vyMetersPerSecond, null);
        builder.addDoubleProperty("Velocity Heading (Deg)", () -> getCurrentVelocity().omegaRadiansPerSecond * 180 / Math.PI, null);
    }
}
