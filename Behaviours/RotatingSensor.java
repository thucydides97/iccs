package Behaviours;

import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class RotatingSensor implements Behavior {

	public RobotController robot;
	
	private float dist = .35f;
	
	public RotatingSensor(RobotController robot) {
		this.robot = robot;
	}
	
	public boolean takeControl() {
		return robot.isCloseToObstacleUltrasonic();
	}

	public void action() {
		robot.moveBackward();
		
		try{
	    	Thread.sleep(500);
	    } catch(Exception e) {}
	    robot.stopMoving();
		
		if(checkRight()) {
			robot.turnRight(90);
		} else if (checkLeft()) {
			robot.turnLeft(90);
		} else {
			robot.turnLeft(180);
		}
	}

	public void suppress() {
		robot.stopMoving();
	}
	
	public boolean checkRight() {
		robot.rotateExtraMotor(-90);
		
		float sensorDist = robot.getSensorDistanceUltrasonic(); 
		
		robot.rotateExtraMotor(90);
		return sensorDist > dist;
	}
	
	public boolean checkLeft() {
		robot.rotateExtraMotor(90);
		
		float sensorDist = robot.getSensorDistanceUltrasonic(); 

		robot.rotateExtraMotor(-90);
		return sensorDist > dist;
	}
}
