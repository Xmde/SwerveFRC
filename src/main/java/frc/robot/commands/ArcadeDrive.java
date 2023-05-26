package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.oi.inputs.OITrigger.PrioritizedTrigger;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    Drivetrain drivetrain;
    PrioritizedAxis fwd;
    PrioritizedAxis str;
    PrioritizedAxis rcw;

    PrioritizedTrigger resetPose;
    PrioritizedTrigger flipMode;

    final double MAX_SPEED;
    boolean fieldOriented = false;
    boolean flipTrigger = false;

    public ArcadeDrive(Drivetrain drivetrain, OIAxis fwd, OIAxis str, OIAxis rcw, OITrigger resetPose, OITrigger flipMode) {
        this.drivetrain = drivetrain;
        this.fwd = fwd.prioritize(0);
        this.str = str.prioritize(0);
        this.rcw = rcw.prioritize(0);

        this.resetPose = resetPose.prioritize(0);
        this.flipMode = flipMode.prioritize(0);

        addRequirements(drivetrain);
        MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public void execute() {
        ChassisSpeeds speed = new ChassisSpeeds(
            -(fwd.get() * MAX_SPEED),
            -(str.get() * MAX_SPEED),
            -rcw.get() / 10
        );

        if (fieldOriented) {
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed, drivetrain.getPose().getRotation()));
        } else {
            drivetrain.drive(speed);
        }
        if (resetPose.get()) {
            drivetrain.setPose(new Pose2d());
        }
        if (!flipTrigger && flipMode.get()){
            flipTrigger = true;
            fieldOriented = !fieldOriented;
        }
        if (flipTrigger && !flipMode.get()) {
            flipTrigger = false;
        }
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
        fwd.destroy();
        str.destroy();
        rcw.destroy();
        flipMode.destroy();
        resetPose.destroy();
    }
}
