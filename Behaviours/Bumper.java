package Behaviours;

import main.RobotController;
import lejos.robotics.subsumption.Behavior;

public class Bumper implements Behavior {
	
	private RobotController robot;
	
	private boolean rightSide = false;
	
	private static int DEGREES = 90;
	
	public Bumper(RobotController robot, boolean rightSide) {
		this.robot = robot;
		this.rightSide = rightSide;
	}
	
	public boolean takeControl() {
		if(rightSide) {
			return robot.isTouchSensor1Pressed();
		} else {
			return robot.isTouchSensor2Pressed();
		}
	}

	public void action() {
		robot.moveBackward();
		
	    try{
	    	Thread.sleep(1000);
	    } catch(Exception e) {}
	    
	    robot.stopMoving();
	    if(rightSide) {
	    	robot.turnLeft(DEGREES);	    	
	    } else {
	    	robot.turnRight(DEGREES);
	    }
	}

	public void suppress() {
		robot.stopMoving();
	}
}
