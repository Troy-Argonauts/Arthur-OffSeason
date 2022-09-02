package frc.robot.auton.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class DT_PID extends SequentialCommandGroup {
	public DT_PID(int inches) {
        addCommands(
				new RunCommand(() -> Robot.getDriveTrain().distancePID(inches))
        );
		addRequirements(Robot.getDriveTrain());
	}
}