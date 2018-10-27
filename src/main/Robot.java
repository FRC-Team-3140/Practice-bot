// Copyright (c) 2018 FIRST 3140. All Rights Reserved.

package main;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import main.subsystems.DriverCamera;
import main.subsystems.Drivetrain;
import main.subsystems.Intake;
import main.subsystems.Pneumatics;

public class Robot extends IterativeRobot implements Constants {
	public static Drivetrain dt;
	public static Pneumatics pn;
	public static Intake in;
	public static DriverCamera dc;
	public static OI oi;
	
	@Override
	public void robotInit() {
		dt = new Drivetrain();
		pn = new Pneumatics();
		in = new Intake();
		oi = new OI();
		dc = new DriverCamera();
	}
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous shows how to selectbetween different autonomous modes using the dashboard. 
	 * The sendable chooser code works with the Java SmartDashboard. 
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();


		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during auto
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
