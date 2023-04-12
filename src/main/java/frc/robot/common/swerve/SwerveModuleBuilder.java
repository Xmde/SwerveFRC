// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveModuleBuilder {

    public static enum SWERVE_MODULE_PRESETS {
        SDS_MK4i_L1(8.14, 150.0/7.0, Units.inchesToMeters(4)),
        SDS_MK4i_L2(6.75, 150.0/7.0, Units.inchesToMeters(4)),
        SDS_MK4i_L3(6.12, 150.0/7.0, Units.inchesToMeters(4));

        public final double driveGearRatio;
        public final double turnGearRatio;
        public final double WheelDiameter;

        SWERVE_MODULE_PRESETS(double driveGearRatio, double turnGearRatio, double WheelDiameter) {
            this.driveGearRatio = driveGearRatio;
            this.turnGearRatio = turnGearRatio;
            this.WheelDiameter = WheelDiameter;
        }
    }

    private Translation2d location;
    private double driveGearRatio = 0; // A ratio of 2 means that the drive motor spins twice as fast as the wheel
    private double turnGearRatio = 0; // A ratio of 2 means that the turn motor spins twice as fast as the rotate wheel
    private double WheelDiameter = 0; // In meters
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0);
    private double driveP = 0;
    private double driveI = 0;
    private double driveD = 0;
    private double turnP = 0;
    private double turnI = 0;
    private double turnD = 0;
    private double driveMotorMaxRPM = 0;
    
    private CANSparkMax sparkMaxDriveMotor;
    private CANSparkMax sparkMaxTurnMotor;
    private TalonFX falconDriveMotor;
    private TalonFX falconTurnMotor;
    
    private Supplier<Double> turnEncoderAbsolute; // In radians the encoder reads positive is counterclockwise
    private double turnEncoderAbsoluteRatio = 1; // A ratio of 2 means that the encoder reads twice as fast as the wheel
    private double turnEncoderAbsoluteOffset = 0; // In radians positive is counterclockwise

    private boolean built = false; // If the module has been built

    public SwerveModuleBuilder() {}
    public SwerveModuleBuilder(Translation2d location) {
        this.location = location;
    }
    public SwerveModuleBuilder(Translation2d location, SWERVE_MODULE_PRESETS preset) {
        this.location = location;
        this.driveGearRatio = preset.driveGearRatio;
        this.turnGearRatio = preset.turnGearRatio;
        this.WheelDiameter = preset.WheelDiameter;
    }

    public SwerveModuleBuilder location(Translation2d location) {
        this.location = location;
        return this;
    }
    public SwerveModuleBuilder driveGearRatio(double driveGearRatio) {
        this.driveGearRatio = driveGearRatio;
        return this;
    }
    public SwerveModuleBuilder turnGearRatio(double turnGearRatio) {
        this.turnGearRatio = turnGearRatio;
        return this;
    }
    public SwerveModuleBuilder wheelDiameter(double WheelDiameter) {
        this.WheelDiameter = WheelDiameter;
        return this;
    }
    public SwerveModuleBuilder driveFeedforward(SimpleMotorFeedforward driveFeedforward) {
        this.driveFeedforward = driveFeedforward;
        return this;
    }
    public SwerveModuleBuilder turnFeedforward(SimpleMotorFeedforward turnFeedforward) {
        this.turnFeedforward = turnFeedforward;
        return this;
    }
    public SwerveModuleBuilder drivePID(double driveP, double driveI, double driveD) {
        this.driveP = driveP;
        this.driveI = driveI;
        this.driveD = driveD;
        return this;
    }
    public SwerveModuleBuilder turnPID(double turnP, double turnI, double turnD) {
        this.turnP = turnP;
        this.turnI = turnI;
        this.turnD = turnD;
        return this;
    }
    public SwerveModuleBuilder driveNEO1650(int deviceID) {
        this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        this.driveMotorMaxRPM = 5_676;
        return this;
    }
    public SwerveModuleBuilder turnNEO1650(int deviceID) {
        this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        return this;
    }
    public SwerveModuleBuilder driveFalcon500(int deviceID) {
        this.falconDriveMotor = new TalonFX(deviceID);
        this.driveMotorMaxRPM = 6_380;
        return this;
    }
    public SwerveModuleBuilder turnFalcon500(int deviceID) {
        this.falconTurnMotor = new TalonFX(deviceID);
        return this;
    }
    public SwerveModuleBuilder driveNEO550(int deviceID) {
        this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        this.driveMotorMaxRPM = 11_000;
        return this;
    }
    public SwerveModuleBuilder turnNEO550(int deviceID) {
        this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        return this;
    }
    public SwerveModuleBuilder turnEncoderAbsolute(Supplier<Double> turnEncoderAbsolute) {
        this.turnEncoderAbsolute = turnEncoderAbsolute;
        return this;
    }
    public SwerveModuleBuilder CANCoder(int deviceID, double offset) {
        CANCoder canCoder = new CANCoder(deviceID);
        canCoder.configFactoryDefault();
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.turnEncoderAbsolute = () -> {
            return Math.toRadians(canCoder.getAbsolutePosition());
        };
        this.turnEncoderAbsoluteOffset = offset;
        return this;
    }

    public SwerveModuleBuilder turnEncoderAbsoluteRatio(double turnEncoderAbsoluteRatio) {
        this.turnEncoderAbsoluteRatio = turnEncoderAbsoluteRatio;
        return this;
    }
    public SwerveModuleBuilder turnEncoderAbsoluteOffset(double turnEncoderAbsoluteOffset) {
        this.turnEncoderAbsoluteOffset = turnEncoderAbsoluteOffset;
        return this;
    }

    public SwerveModule build() {
        if (built) {
            throw new IllegalStateException("SwerveModule has already been built");
        }
        if (location == null) {
            throw new IllegalStateException("Location has not been set");
        }
        if (driveGearRatio == 0) {
            throw new IllegalStateException("Drive gear ratio has not been set");
        }
        if (turnGearRatio == 0) {
            throw new IllegalStateException("Turn gear ratio has not been set");
        }
        if (WheelDiameter == 0) {
            throw new IllegalStateException("Wheel diameter has not been set");
        }
        if (driveMotorMaxRPM == 0) {
            throw new IllegalStateException("Drive motor max RPM has not been set");
        }
        if (sparkMaxDriveMotor == null && falconDriveMotor == null) {
            throw new IllegalStateException("Drive motor has not been set");
        }
        if (sparkMaxTurnMotor == null && falconTurnMotor == null) {
            throw new IllegalStateException("Turn motor has not been set");
        }
        if (turnEncoderAbsolute == null) {
            throw new IllegalStateException("Turn encoder has not been set");
        }
        if (sparkMaxDriveMotor != null && falconDriveMotor != null) {
            throw new IllegalStateException("Drive motor has been set twice");
        }
        if (sparkMaxTurnMotor != null && falconTurnMotor != null) {
            throw new IllegalStateException("Turn motor has been set twice");
        }
        
        Supplier<Double> speedSupplier;
        Supplier<Rotation2d> angleSupplier;
        Supplier<Double> distanceSupplier;
        Consumer<Double> speedConsumer;
        Consumer<Rotation2d> angleConsumer;
        Object driveMotor;
        Object turnMotor;
        double maxSpeedMPS = 0;

        // This uses the turnEncoderAbsolute to reset the motor encoder to make sure they match.
        // The turnEncoderAbsolute times (1/turnEncoderAbsoluteRatio) plus the offset is the angle of the wheel in radians.
        // The normal motor encoder needs to be multiplied by the turnGearRatio then converted from rotations to radians.
        Runnable recalibrate; 

        if (sparkMaxDriveMotor != null) {
            RelativeEncoder encoder = sparkMaxDriveMotor.getEncoder();
            encoder.setPositionConversionFactor(1 / (driveGearRatio * WheelDiameter * Math.PI)); // TODO Check to make sure this is accurate
            encoder.setVelocityConversionFactor(1 / (driveGearRatio * WheelDiameter * Math.PI)); // TODO Check to make sure this is accurate
            distanceSupplier = encoder::getPosition;
            speedSupplier = encoder::getVelocity;
            SparkMaxPIDController pidController = sparkMaxDriveMotor.getPIDController();
            pidController.setP(driveP, 0);
            pidController.setI(driveI, 0);
            pidController.setD(driveD, 0);
            pidController.setFeedbackDevice(encoder);
            speedConsumer = (Double speed) -> {
                pidController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed));
            };
            driveMotor = sparkMaxDriveMotor;
        } else if (falconDriveMotor != null) {
            // TODO
            distanceSupplier = null;
            speedSupplier = null;
            speedConsumer = null;

            driveMotor = falconDriveMotor;
            throw new IllegalStateException("Falcons are not supported yet");
        } else {
            throw new IllegalStateException("Drive motor has not been set");
        }

        if (sparkMaxTurnMotor != null) {
            RelativeEncoder encoder = sparkMaxTurnMotor.getEncoder();
            encoder.setPositionConversionFactor(1 / (turnGearRatio * 2 * Math.PI)); // TODO Check to make sure this is accurate
            encoder.setVelocityConversionFactor(1 / (turnGearRatio * 2 * Math.PI)); // TODO Check to make sure this is accurate
            angleSupplier = () -> {
                return new Rotation2d(encoder.getPosition());
            };
            SparkMaxPIDController pidController = sparkMaxTurnMotor.getPIDController();
            pidController.setP(turnP, 0);
            pidController.setI(turnI, 0);
            pidController.setD(turnD, 0);
            pidController.setFeedbackDevice(encoder);
            angleConsumer = (Rotation2d angle) -> {
                pidController.setReference(angle.getRadians(), ControlType.kPosition, 0, turnFeedforward.calculate(angle.getRadians()));
            };
            // Only recalibrate if the error is more than 0.5 degree on either side
            recalibrate = () -> {
                double error = turnEncoderAbsolute.get() * (1 / turnEncoderAbsoluteRatio) + turnEncoderAbsoluteOffset - encoder.getPosition();
                if (Math.abs(error) > Math.toRadians(0.5)) {
                    encoder.setPosition(turnEncoderAbsolute.get() * (1 / turnEncoderAbsoluteRatio) + turnEncoderAbsoluteOffset);
                }
            };

            turnMotor = sparkMaxTurnMotor;
        } else if (falconTurnMotor != null) {
            // TODO
            angleSupplier = null;
            angleConsumer = null;

            turnMotor = falconTurnMotor;
            throw new IllegalStateException("Falcons are not supported yet");
        } else {
            throw new IllegalStateException("Turn motor has not been set");
        }

        maxSpeedMPS = driveMotorMaxRPM * driveGearRatio * WheelDiameter * Math.PI / 60;

        built = true;

        if (location == null || speedSupplier == null || angleSupplier == null || distanceSupplier == null || speedConsumer == null || angleConsumer == null || recalibrate == null || maxSpeedMPS == 0) {
            throw new IllegalStateException("Something went wrong building the SwerveModule");
        }

        return new SwerveModule(location, speedSupplier, angleSupplier, distanceSupplier, speedConsumer, angleConsumer, recalibrate, driveMotor, turnMotor, maxSpeedMPS);
    }

}
