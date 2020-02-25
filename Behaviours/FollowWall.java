package Behaviours;

import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class FollowWall implements Behavior{
	
	private RobotController robot;
	
	private float distance = 0;
	private double closeThreshold;
	private double farThreshold;
	private double findWallThreshold;
	
	private boolean suppressed = false;
	
	public FollowWall(RobotController robot, double closeThreshold, double farThreshold, double findWallThreshold) {
		this.robot = robot;
		this.closeThreshold = closeThreshold;
		this.farThreshold = farThreshold;
		this.findWallThreshold = findWallThreshold;
	}

	public boolean takeControl() {
		distance = robot.getSensorDistanceUltrasonic();
		
		return distance < closeThreshold || (distance > farThreshold && distance < findWallThreshold);
	}

	public void action() {
		
		suppressed = false;
		
		if(robot.getSensorDistanceUltrasonic() < closeThreshold) {
			robot.turnRight(30);
		}
		else if(robot.getSensorDistanceUltrasonic() > farThreshold) {
			robot.turnLeft(30);
		}
		
		float newDistance = robot.getSensorDistanceUltrasonic();
		
		while(newDistance < closeThreshold || (newDistance > farThreshold && newDistance < findWallThreshold) && !suppressed) {
			robot.moveForward();
		}
	
		robot.stopMoving();
	}

	public void suppress() {
		suppressed = true;
	}

}
