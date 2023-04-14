package frc.robot.utilities;

public interface MotorController {
    public void enableBrakeMode();
    public void enableCoastMode();
    /*
    TODO - Add more methods here if the motor controllers returned from SwerveModule need to do more than just set idle mode.
    My vision for this was to convert the entire project to use the motor controller interface instead of suppliers / consumers before I realized how ambitious of an effort that would be.
    May or may not be worth looking into in the future.
    */
}
