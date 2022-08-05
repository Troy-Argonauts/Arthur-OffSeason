package frc.libs.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ArgoMotor{
    public WPI_TalonFX ArgoFalcon(int canID) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);

        motor.configFactoryDefault();
        motor.setSensorPhase(false);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
        motor.configFeedbackNotContinuous(false, 4);
        motor.configOpenloopRamp(Constants.DriveTrain.RAMP_SECONDS);
        motor.configClosedloopRamp(Constants.DriveTrain.RAMP_SECONDS);
        motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        motor.setNeutralMode(NeutralMode.Coast);

        return motor;
    }

    public WPI_TalonSRX ArgoCIM(int canID) {
        WPI_TalonSRX motor = new WPI_TalonSRX(canID);

        motor.configFactoryDefault();
        motor.setSensorPhase(false);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
        motor.configFeedbackNotContinuous(false, 4);
        motor.configOpenloopRamp(Constants.DriveTrain.RAMP_SECONDS);
        motor.configClosedloopRamp(Constants.DriveTrain.RAMP_SECONDS);
        motor.setNeutralMode(NeutralMode.Coast);

        return motor;
    }

    public CANSparkMax ArgoNeo550(int canID) {
        CANSparkMax motor = new CANSparkMax(canID, CANSparkMax.MotorType.kBrushless);

        motor.setSmartCurrentLimit(14);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setClosedLoopRampRate(10);
        motor.setOpenLoopRampRate(10);

        return motor;
    }
 }
