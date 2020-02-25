package Behaviours;

import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class FrontSensing implements Behavior {
	
	private RobotController robot;
	
	public FrontSensing(RobotController robot) {
		this.robot = robot;
	}

	public boolean takeControl() {
		return robot.isCloseToObstacleUltrasonic();
	}

	public void action() {
		robot.moveBackward();
		
	    try{
	    	Thread.sleep(1000);
	    } catch(Exception e) {}
	    robot.stopMoving();
	    
	    robot.turnLeft(90);
	}

	public void suppress() {
		robot.stopMoving();
	}
		
}
