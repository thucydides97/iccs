package Behaviours;

import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class AdvancedRotatingSensor implements Behavior {
	
	private RobotController robot;
	private int numChecks;
	private int rotatingAngle;
	
	public AdvancedRotatingSensor(RobotController robot, int numChecks) {
		this.robot = robot;
		
		this.numChecks = numChecks;
		this.rotatingAngle = 180 / (numChecks - 1);
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
	    
	    robot.rotateExtraMotor(-90);
	    
	    float longestDistance = robot.getSensorDistanceUltrasonic();;
	    int longestAngle = -90;
	    
	    for(int i = 1; i < numChecks; i++) {
	    	robot.rotateExtraMotor(rotatingAngle);
	    	float newDis = robot.getSensorDistanceUltrasonic();
	    	
	    	if(newDis > longestDistance) {
	    		longestDistance = newDis;
	    		longestAngle = (i * rotatingAngle) - 90;
	    	}
	    }
	    
	    robot.rotateExtraMotor(-90);
	    
		robot.turnLeft(longestAngle);
	}

	public void suppress() {
		robot.stopMoving();
	}

}
