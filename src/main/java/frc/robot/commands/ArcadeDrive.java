package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    Drivetrain drivetrain;
    PrioritizedAxis fwd;
    PrioritizedAxis str;
    PrioritizedAxis rcw;

    public ArcadeDrive(Drivetrain drivetrain, OIAxis fwd, OIAxis str, OIAxis rcw) {
        this.drivetrain = drivetrain;
        this.fwd = fwd.prioritize(0);
        this.str = str.prioritize(0);
        this.rcw = rcw.prioritize(0);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(
            str.get() / 2,
            fwd.get() / 2,
            rcw.get()
        ));
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
        fwd.destroy();
        str.destroy();
        rcw.destroy();
    }
}
