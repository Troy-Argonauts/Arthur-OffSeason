package frc.libs.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

public class ArgoMotor{
    public static LazyTalonFX generateConfigTalonFX(int canID, double rampRate) {
        LazyTalonFX motor = new LazyTalonFX(canID);

        motor.configFactoryDefault();
        motor.setSensorPhase(false);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
        motor.configFeedbackNotContinuous(false, 4);
        motor.configOpenloopRamp(rampRate);
        motor.configClosedloopRamp(rampRate);
        motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        motor.setNeutralMode(NeutralMode.Coast);

        return motor;
    }

    public static LazyTalonSRX generateConfigTalonSRX(int canID, double rampRate) {
        LazyTalonSRX motor = new LazyTalonSRX(canID);

        motor.configFactoryDefault();
        motor.setSensorPhase(false);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
        motor.configFeedbackNotContinuous(false, 4);
        motor.configOpenloopRamp(rampRate);
        motor.configClosedloopRamp(rampRate);
        motor.setNeutralMode(NeutralMode.Coast);

        return motor;
    }

    public static LazyCANSparkMax generateConfigSparkMax(int canID, double rampRate) {
        LazyCANSparkMax motor = new LazyCANSparkMax(canID, CANSparkMax.MotorType.kBrushless);

        motor.setSmartCurrentLimit(14);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setClosedLoopRampRate(rampRate);
        motor.setOpenLoopRampRate(rampRate);

        return motor;
    }
 }
