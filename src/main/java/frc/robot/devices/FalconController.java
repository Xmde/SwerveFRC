package frc.robot.devices;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.utilities.MotorController;

public class FalconController implements MotorController {
    private TalonFX controller;

    public FalconController(TalonFX controller) {
        this.controller = controller;
    }

    @Override
    public void enableBrakeMode() {
        controller.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void enableCoastMode() {
        controller.setNeutralMode(NeutralMode.Coast);
    }
}
