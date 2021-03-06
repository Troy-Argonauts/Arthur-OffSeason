package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSystem extends SubsystemBase {

    private final DoubleSolenoid solenoid;
    private DoubleSolenoid.Value currentState;

    public PneumaticsSystem() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.SOLENOID_1, Constants.Pneumatics.SOLENOID_2);
        currentState = DoubleSolenoid.Value.kForward;
        updateState();
    }

    public void pickupIntake(){
        if (currentState == DoubleSolenoid.Value.kReverse) {
            currentState = DoubleSolenoid.Value.kForward;
            updateState();
        }
    }

    public void dropIntake() {
        if (currentState == DoubleSolenoid.Value.kForward) {
            currentState = DoubleSolenoid.Value.kReverse;
            updateState();
        }
    }

    public void updateState() {
        solenoid.set(currentState);
    }
}
