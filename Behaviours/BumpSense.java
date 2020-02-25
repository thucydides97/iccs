package Behaviours;

import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class BumpSense implements Behavior {
	
	private RobotController robot;

	public BumpSense(RobotController robot) {
		this.robot = robot;
	}
	
	public boolean takeControl() {
		return robot.isTouchSensor1Pressed() || robot.isTouchSensor2Pressed();
	}

	public void action() {
		float distance = robot.getSensorDistanceUltrasonic();
		
		robot.moveBackward();
		
		try{
			Thread.sleep(1000);
		} catch(Exception e) {}
		    
		robot.stopMoving();
		
		if(distance < .1) {
			robot.turnRight(90);
		} else {
			robot.turnLeft(90);
		}
	}

	public void suppress() {
		robot.stopMoving();
	}

}
