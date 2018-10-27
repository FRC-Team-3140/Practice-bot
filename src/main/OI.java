package main;

import lib.joystick.XboxController;
import main.commands.pneumatics.arm.ArmClose;
import main.commands.pneumatics.arm.ArmOpen;
import main.commands.pneumatics.arm.SwitchArm;
import main.commands.pneumatics.shift.ShiftDown;
import main.commands.pneumatics.shift.ShiftUp;
import main.commands.pneumatics.tilt.SwitchTilt;
import main.commands.pneumatics.tilt.TiltDown;
import main.commands.pneumatics.tilt.TiltUp;

public class OI implements Constants, HardwareAdapter {
	public OI() {
		xbox.setInternalControl(false);
		xbox2.setInternalControl(false);
		
		// Shoots
		xbox.leftJoystickPress.whenPressed(new ShiftUp());
		xbox.leftJoystickPress.whenReleased(new ShiftDown());
		
		// this the code for tilt
		xbox2.leftBumper.whenPressed(new SwitchTilt(new TiltDown(), new TiltUp()));
		xbox2.rightBumper.whenPressed(new SwitchArm(new ArmOpen(), new ArmClose()));
	}
	
	public static XboxController getXbox() {
		return xbox;
	}
	
    public static XboxController getXbox2() {
		return xbox2;
	}
    
	public void check() {
		xbox.check();
		xbox2.check();
	}