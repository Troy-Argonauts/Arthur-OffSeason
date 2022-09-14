package frc.robot.auton.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;

public class DT_ResetSensors extends ParallelCommandGroup {
	public DT_ResetSensors() {
		addCommands(
				new InstantCommand(Robot.getDriveTrain()::zeroEncoders),
				new InstantCommand(Robot.getDriveTrain()::resetGyro),
				new InstantCommand(Robot.getDriveTrain()::resetPID)
		);
		addRequirements(Robot.getDriveTrain());
	}
}