package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.util.ArgoMotor;
import frc.libs.util.LazyTalonFX;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private final LazyTalonFX frontLeft, frontRight, rearLeft, rearRight;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    /**
     * Sets the values of the frontLeft and frontRight motors, and creates local rear motors.
     * Has rear motors follow front motors, and sets all motors to coast when stopped.
     */
    public DriveTrain() {
        frontLeft = ArgoMotor.generateConfigTalonFX(Constants.DriveTrain.FRONT_LEFT, Constants.DriveTrain.RAMP_SECONDS);
        frontRight = ArgoMotor.generateConfigTalonFX(Constants.DriveTrain.FRONT_RIGHT, Constants.DriveTrain.RAMP_SECONDS);
        rearLeft = ArgoMotor.generateConfigTalonFX(Constants.DriveTrain.REAR_LEFT, Constants.DriveTrain.RAMP_SECONDS);
        rearRight = ArgoMotor.generateConfigTalonFX(Constants.DriveTrain.REAR_RIGHT, Constants.DriveTrain.RAMP_SECONDS);

        rearLeft.follow(frontLeft);
        rearRight.follow(frontRight);

        frontLeft.setInverted(true);
        rearLeft.setInverted(InvertType.FollowMaster);
        frontRight.setInverted(false);
        rearRight.setInverted(InvertType.FollowMaster);

        gyro.calibrate();
    }

    /**
     * Has the robot move at a certain speed, but allows the robot to turn, using input from Joysticks
     * @param turn Amount to turn the robot
     * @param speed Speed of robot
     */
    public void cheesyDriveAuton(double turn, double speed, double nerf) {
        frontLeft.set(ControlMode.PercentOutput, -(speed - turn) * nerf);
        frontRight.set(ControlMode.PercentOutput, -(speed + turn) * nerf);
    }

    public void cheesyDriveTeleop(double turn, double speed, double nerf) {
        frontLeft.set(ControlMode.PercentOutput, (speed - turn * 0.8) * nerf);
        frontRight.set(ControlMode.PercentOutput, (speed + turn * 0.8) * nerf);
    }

    public double getEncoderPosition(boolean backwards) {
        if (backwards) {
            return -(Math.abs(frontRight.getSelectedSensorPosition()) + Math.abs(frontLeft.getSelectedSensorPosition())) / 2;
        }
        return (Math.abs(frontRight.getSelectedSensorPosition()) + Math.abs(frontLeft.getSelectedSensorPosition())) / 2;
    }

    public void zeroEncoders() {
        frontRight.setSelectedSensorPosition(0);
        frontLeft.setSelectedSensorPosition(0);
        rearRight.setSelectedSensorPosition(0);
        rearLeft.setSelectedSensorPosition(0);
    }

    public double getAngle() {
        return (gyro.getAngle() % 360);
    }

    public void zeroGyro() {
        gyro.reset();
    }

    private void motorBreakMode(boolean enabled) {
        if (enabled) {
            frontLeft.setNeutralMode(NeutralMode.Brake);
            frontRight.setNeutralMode(NeutralMode.Brake);
        } else {
            frontLeft.setNeutralMode(NeutralMode.Coast);
            frontRight.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Encoders", frontRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Encoders", frontLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("encoder", getEncoderPosition(false));

        SmartDashboard.putNumber("Angle", gyro.getAngle());
    }

    public void driveStraight(double inches, double speed) {
        boolean backwards = false;
        if (inches < 0) {
            backwards = true;
        }

        double turningValue = (0 - gyro.getAngle()) * Constants.DriveTrain.kP_TURN;

        double distance = inches * Constants.DriveTrain.NU_PER_INCH;
        SmartDashboard.putNumber("Distance", distance);
        turningValue = Math.copySign(turningValue, distance);

        motorBreakMode(true);

            while (getEncoderPosition(backwards) > distance) {
//                double previousEncoder = getEncoderPosition(backwards);
                cheesyDriveAuton(turningValue, -1, speed);
                SmartDashboard.putNumber("Right Encoders", frontRight.getSelectedSensorPosition());
                SmartDashboard.putNumber("Left Encoders", frontLeft.getSelectedSensorPosition());
                SmartDashboard.putNumber("encoder", getEncoderPosition(backwards));

//                if (previousEncoder == getEncoderPosition(backwards)) {
//                    cheesyDriveAuton(0, 0, 0);
//                }
            }
            cheesyDriveAuton(0,0,1);

            while (getEncoderPosition(backwards) < distance) {
//                double previousEncoder = getEncoderPosition(backwards);
                cheesyDriveAuton(turningValue, 1, speed);
                SmartDashboard.putNumber("Right Encoders", frontRight.getSelectedSensorPosition());
                SmartDashboard.putNumber("Left Encoders", frontLeft.getSelectedSensorPosition());
                SmartDashboard.putNumber("encoder", getEncoderPosition(backwards));

//                if (previousEncoder == getEncoderPosition(backwards)) {
//                    cheesyDriveAuton(0, 0, 0);
//                }
            }
            cheesyDriveAuton(0,0,1);

        motorBreakMode(false);
    }

    public void turnToAngle(double angle) {
        double time = 4.4 * (angle / 360);

        Timer timer = new Timer();
        timer.start();

        while (timer.get() < time) {
            cheesyDriveAuton(1, 0, 0.15);
        }

        cheesyDriveAuton(0,0,1);
        timer.stop();
    }
}