package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CANCoderPrint extends CommandBase {
    AHRS gyro;
    CANCoder[] cancoders;

    public CANCoderPrint(AHRS gyro, CANCoder... cancoders) {
        this.gyro = gyro;
        this.cancoders = cancoders;
    }

    @Override
    public void execute() {
        for (int i = 0; i < cancoders.length; i++) {
            System.out.println("CANCoder number " + i + " at position " + cancoders[i].getAbsolutePosition());
        }
        System.out.println(gyro.getRotation2d());
    }
}
