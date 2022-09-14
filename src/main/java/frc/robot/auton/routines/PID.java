package frc.robot.auton.routines;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.auton.commands.DT_PID;
import frc.robot.auton.commands.DT_ResetSensors;

public class PID extends SequentialCommandGroup {
	public PID() {
        addCommands(
                new DT_ResetSensors(),
                new DT_PID(120) // only works between 0 and 5 feet
        );
	}
}