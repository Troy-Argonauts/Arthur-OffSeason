package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.util.ArgoMotor;
import frc.libs.util.LazyTalonFX;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final LazyTalonFX shooterFront, shooterBack;
    private boolean active;
    public static double FRONT_SPEED, BACK_SPEED;
    public static int PRESET_POSITION;

    public Shooter() {
        active = false;

        FRONT_SPEED = Constants.Shooter.FRONT_DEFAULT_SPEED;
        BACK_SPEED = Constants.Shooter.BACK_DEFAULT_SPEED;

        shooterFront = ArgoMotor.generateConfigTalonFX(Constants.Shooter.PORT, Constants.Shooter.RAMP_SECONDS);
        shooterBack = ArgoMotor.generateConfigTalonFX(Constants.Shooter.SLAVE_PORT, Constants.Shooter.RAMP_SECONDS);

        shooterFront.setInverted(true);
        shooterBack.setInverted(false);

        StatorCurrentLimitConfiguration statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration();
        statorCurrentLimitConfiguration.currentLimit = 60;
        statorCurrentLimitConfiguration.enable = true;
        statorCurrentLimitConfiguration.triggerThresholdCurrent = 80;
        statorCurrentLimitConfiguration.triggerThresholdTime = 0.5;

        shooterFront.configStatorCurrentLimit(statorCurrentLimitConfiguration);
        shooterBack.configStatorCurrentLimit(statorCurrentLimitConfiguration);

        shooterFront.config_kF(0, Constants.Shooter.kF);
        shooterFront.config_kP(0, Constants.Shooter.kP);
        shooterFront.config_kI(0, Constants.Shooter.kI);
        shooterFront.config_kD(0, Constants.Shooter.kD);
        shooterBack.config_kF(0, Constants.Shooter.kF);
        shooterBack.config_kP(0, Constants.Shooter.kP);
        shooterBack.config_kI(0, Constants.Shooter.kI);
        shooterBack.config_kD(0, Constants.Shooter.kD);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Activated", active);
        SmartDashboard.putString("Front Speed", (int) (Shooter.FRONT_SPEED * 100) + "%");
        SmartDashboard.putString("Rear Speed", (int) (Shooter.BACK_SPEED * 100) + "%");
    }

    public enum ShooterState {
        SHOOT, STOPPED
    }

    public void setState(ShooterState state) {
        switch (state) {
            case SHOOT:
                shooterFront.set(ControlMode.PercentOutput, FRONT_SPEED);
                shooterBack.set(ControlMode.PercentOutput, BACK_SPEED);
                active = true;
                break;
            case STOPPED:
                shooterFront.set(ControlMode.PercentOutput, 0);
                shooterBack.set(ControlMode.PercentOutput, 0);
                active = false;
                break;
        }
    }

    public void setPreset() {
        String[] presetArray = {"Fender Low", "Fender High", "Tarmac High"};
        double[] presetFrontSpeeds = {0.2, 0.33, 0.45};
        double[] presetBackSpeeds = {0.1, 0.33, 0.45};

        if (PRESET_POSITION > (presetArray.length - 1)) {
            PRESET_POSITION = 0;
        } else if (PRESET_POSITION < 0) {
            PRESET_POSITION = (presetArray.length - 1);
        }

        SmartDashboard.putString("Preset", presetArray[PRESET_POSITION]);
        FRONT_SPEED = presetFrontSpeeds[PRESET_POSITION];
        BACK_SPEED = presetBackSpeeds[PRESET_POSITION];
    }

    public void resetPreset(int id) {
        PRESET_POSITION = id;
        setPreset();
    }
}
