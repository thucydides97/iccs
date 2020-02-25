package Behaviours;

import main.RobotController;
import lejos.robotics.subsumption.Behavior;

public class DriveForward implements Behavior {
	
	private RobotController robot;
	
	private boolean suppressed = false;
	
	public DriveForward(RobotController robot) {
		this.robot = robot;
	}
	
	public boolean takeControl() {
		return true;
	}

	public void action() {
		suppressed = false;
	    robot.moveForward();
	     
	    while(!suppressed) {
	    	Thread.yield();
	    }
	     
	    robot.stopMoving();
	}

	public void suppress() {
		suppressed = true;
	}
}