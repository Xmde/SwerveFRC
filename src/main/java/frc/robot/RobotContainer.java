package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.Ports;

public class RobotContainer {
  private Drivetrain drivetrain;
  private AHRS gyro;
  private ArcadeDrive drive;
  private ControllerDriver driverXBox;

  public RobotContainer() {
    configureBindings();
    gyro = new AHRS();
    drivetrain = new Drivetrain(gyro);
    driverXBox = new ControllerDriver(Ports.OI.DRIVER_XBOX_PORT);
    drive = new ArcadeDrive(drivetrain, driverXBox.leftY, driverXBox.leftX, driverXBox.rightX, driverXBox.dPadDown, driverXBox.buttonA, driverXBox.buttonB);
    drivetrain.setDefaultCommand(drive);
    SmartDashboard.putData("Drivetrain module 0", drivetrain.mod0);
    SmartDashboard.putData("Drivetrain module 1", drivetrain.mod1);
    SmartDashboard.putData("Drivetrain module 2", drivetrain.mod2);
    SmartDashboard.putData("Drivetrain module 3", drivetrain.mod3);
    SmartDashboard.putData("drivetrain", drivetrain);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
