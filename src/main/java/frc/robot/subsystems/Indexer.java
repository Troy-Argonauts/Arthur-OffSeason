package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.util.ArgoMotor;
import frc.libs.util.LazyCANSparkMax;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final LazyCANSparkMax floorMotor, upMotor;
    private boolean floorActive, upActive;

    public enum IndexerState {
        IN, OUT, STOPPED
    }

    public enum Motor {
        FLOOR, UP
    }
  
    public Indexer() {
        floorMotor = ArgoMotor.generateConfigSparkMax(Constants.Indexer.FLOOR, 0);
        upMotor = ArgoMotor.generateConfigSparkMax(Constants.Indexer.UP, 0);

        floorMotor.setInverted(true);
        upMotor.setInverted(true);

        upMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer Floor Active", floorActive);
        SmartDashboard.putBoolean("Indexer Up Active", upActive);

        SmartDashboard.putNumber("Indexer Floor Amps", floorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer Up Amps", upMotor.getOutputCurrent());
    }

    public void setState(IndexerState indexerState, Motor motor) {
        switch (indexerState) {
            case IN:
                switch (motor) {
                    case FLOOR:
                        floorMotor.set(-Constants.Indexer.FLOOR_SPEED);
                        floorActive = true;
                        break;
                    case UP:
                        upMotor.set(-Constants.Indexer.UP_SPEED);
                        upActive = true;
                        break;
                }
                break;
            case OUT:
                switch (motor) {
                    case FLOOR:
                        floorMotor.set(Constants.Indexer.FLOOR_SPEED);
                        floorActive = true;
                        break;
                    case UP:
                        upMotor.set(Constants.Indexer.UP_SPEED);
                        upActive = true;
                        break;
                }
                break;
            case STOPPED:
                floorMotor.set(0);
                upMotor.set(0);
                upActive = false;
                floorActive = false;
                break;
        }
    }

    public void setState(IndexerState indexerState) {
        setState(indexerState, Motor.UP);
        setState(indexerState, Motor.FLOOR);
    }
}
