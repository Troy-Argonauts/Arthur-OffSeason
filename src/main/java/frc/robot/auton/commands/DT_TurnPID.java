package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class DT_TurnPID extends SequentialCommandGroup {
	public DT_TurnPID(int angle) {
		addCommands(
				new RunCommand(() -> Robot.getDriveTrain().turnPID(angle))
		);
		addRequirements(Robot.getDriveTrain());
	}
}