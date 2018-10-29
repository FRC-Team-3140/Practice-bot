package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import main.Constants;
import main.Robot;


public class TimedDrive extends TimedCommand implements Constants {
	private double throttle;
	
    public TimedDrive(double throttle, double time) {
    	super(time);
    	this.throttle = throttle;
    	requires(Robot.dt);
    }
}
