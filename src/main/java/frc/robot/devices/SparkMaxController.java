package frc.robot.devices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.utilities.MotorController;

public class SparkMaxController implements MotorController {
    private CANSparkMax controller;

    public SparkMaxController(CANSparkMax controller) {
        this.controller = controller;
    }

    @Override
    public void enableBrakeMode() {
        controller.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void enableCoastMode() {
        controller.setIdleMode(IdleMode.kCoast);
    }
}
