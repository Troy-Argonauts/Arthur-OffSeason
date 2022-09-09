package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.util.ArgoMotor;
import frc.libs.util.LazyTalonFX;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private final LazyTalonFX frontLeft, frontRight, rearLeft, rearRight;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    private Pigeon2 pigeon;

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

        frontLeft.setInverted(false);
        rearLeft.setInverted(InvertType.FollowMaster);
        frontRight.setInverted(true);
        rearRight.setInverted(InvertType.FollowMaster);

        gyro.calibrate();

        pigeon = new Pigeon2(Constants.DriveTrain.pigeon_ID);
    }

    /**
     * Has the robot move at a certain speed, but allows the robot to turn, using input from Joysticks
     * @param turn Amount to turn the robot
     * @param speed Speed of robot
     */
    public void cheesyDriveAuton(double turn, double speed, double nerf) {
        frontLeft.set(ControlMode.PercentOutput, (speed - turn) * nerf);
        frontRight.set(ControlMode.PercentOutput, (speed + turn) * nerf);
    }

    public void cheesyDriveTeleop(double turn, double speed, double nerf) {
        frontLeft.set(ControlMode.PercentOutput, -(speed - turn) * nerf);
        frontRight.set(ControlMode.PercentOutput, -(speed + turn) * nerf);
    }

    public double getEncoderPosition(boolean backwards) {
        if (backwards) {
            return -(frontRight.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2;
        }
        return (frontRight.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder", frontLeft.getSelectedSensorPosition() / Constants.DriveTrain.ENCODER_GEARBOX_SCALE);
        SmartDashboard.putNumber("Right Encoder", frontRight.getSelectedSensorPosition() / Constants.DriveTrain.ENCODER_GEARBOX_SCALE);
        SmartDashboard.putNumber("Angle", getAngle());
    }

    public void zeroEncoders() {
        frontRight.setSelectedSensorPosition(0);
        frontLeft.setSelectedSensorPosition(0);
        rearRight.setSelectedSensorPosition(0);
        rearLeft.setSelectedSensorPosition(0);
    }

    public double getAngle() {
        return pigeon.getYaw(); // positive means turning left
    }

    public void resetGyro() {
        pigeon.setYaw(0);
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
    public double rightEncoder() {
        return frontRight.getSelectedSensorPosition();
    }

    public double leftEncoder() {
        return frontLeft.getSelectedSensorPosition();
    }
    public void distancePID(double inches) {
        SmartDashboard.putBoolean("I got here Left", false);
        SmartDashboard.putBoolean("I got here Right", false);
        double prevrightError = 0;
        double prevleftError = 0;
        double period = 0.01;
        double rightOutput, leftOutput = 0;
        double totalrightError = 0;
        double totalleftError = 0;

        double rightError = (inches * Constants.DriveTrain.DISTANCE_CONVERSION) - (rightEncoder() / Constants.DriveTrain.ENCODER_GEARBOX_SCALE);
        totalrightError += rightError * period;
        rightOutput = (Constants.DriveTrain.kP * rightError + Constants.DriveTrain.kI * totalrightError + Constants.DriveTrain.kD * (rightError - prevrightError)) * period;
        prevrightError = rightError;

        double leftError = (inches * Constants.DriveTrain.DISTANCE_CONVERSION) - (leftEncoder() / Constants.DriveTrain.ENCODER_GEARBOX_SCALE);
        totalleftError += leftError * period;
        leftOutput = (Constants.DriveTrain.kP * leftError + Constants.DriveTrain.kI * totalleftError + Constants.DriveTrain.kD * (leftError - prevleftError)) * period;
        prevleftError = leftError;

//        if (leftError > 250 && leftOutput < 0.06) {
//            leftOutput += 0.01;
//            SmartDashboard.putBoolean("I got here Left", true);
//        }
//
//        if (rightError > 250 && rightOutput < 0.06) {
//            rightOutput += 0.01;
//            SmartDashboard.putBoolean("I got here Right", true);
//        }

//        SmartDashboard.putNumber("Left Error", leftError);
//        SmartDashboard.putNumber("Right Error", rightError);
//        SmartDashboard.putNumber("Right Output", rightOutput);
//        SmartDashboard.putNumber("Left Output", leftOutput);

        frontRight.set(ControlMode.PercentOutput, rightOutput);
        frontLeft.set(ControlMode.PercentOutput, leftOutput);
    }

    public void turnPID(double angle) {
        double prevError = 0;
        double period = 1;
        double output = 0;
        double totalError = 0;

        SmartDashboard.putBoolean("turn cutback", false);

        double turnError = angle - getAngle();
        totalError += turnError * period;
        output = (Constants.DriveTrain.kP_TURN * turnError + Constants.DriveTrain.kI_TURN * totalError + Constants.DriveTrain.kD_TURN * (turnError - prevError)) * period;
        prevError = turnError;

        if (Math.abs(turnError) > 5 && output < 0.07) {
            if (turnError > 0) {
                output += 0.03;
            } else {
                output -= 0.03;
            }
            SmartDashboard.putBoolean("turn cutback", true);
        }

        SmartDashboard.putNumber("Turn Error", turnError);
        SmartDashboard.putNumber("Output", output);

        frontRight.set(ControlMode.PercentOutput, output);
        frontLeft.set(ControlMode.PercentOutput, -output);
    }
}