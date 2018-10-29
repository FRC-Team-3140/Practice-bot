package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;
import main.commands.drivetrain.TimedDrive;


public class baseline  extends CommandGroup implements Constants {
	public baseline() {
		addSequential(new TimedDrive(-0.75, 1.285));
		addSequential(new WaitCommand(0.35));//0.35
	}
}
