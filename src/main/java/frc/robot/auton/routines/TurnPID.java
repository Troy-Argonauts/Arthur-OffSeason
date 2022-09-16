package frc.robot.auton.routines;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.auton.commands.DT_PID;
import frc.robot.auton.commands.DT_ResetSensors;
import frc.robot.auton.commands.DT_TurnPID;

public class TurnPID extends SequentialCommandGroup {
	public TurnPID() {
        addCommands(
                new DT_ResetSensors(),
                new DT_TurnPID(90)
        );
	}
}