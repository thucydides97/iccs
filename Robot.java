import Behaviours.AdvancedRotatingSensor;
import Behaviours.BumpSense;
import Behaviours.Bumper;
import Behaviours.DriveForward;
import Behaviours.FollowWall;
import Behaviours.FrontSensing;
import Behaviours.RotatingSensor;
import lejos.hardware.*;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import main.RobotController;

public class Robot {
	private Arbitrator arbitrator = null;
	
	private Behavior forward;
	private Behavior rightBumper;
	private Behavior leftBumper;
	private Behavior frontSensing;
	private Behavior followWall;
	private Behavior bumpSense;
	private Behavior rotatingSensor;
	private Behavior advancedRotatingSensor;
	
	public Robot(RobotController controller) {
		double closeWallThreshold = .05;
		double farWallThreshold = .1;
		double findWallThreshold = .3;
		
		forward = new DriveForward(controller);
		rightBumper = new Bumper(controller, true);
		leftBumper = new Bumper(controller, false);
		frontSensing = new FrontSensing(controller);
		followWall = new FollowWall(controller, closeWallThreshold, farWallThreshold, findWallThreshold);
		bumpSense = new BumpSense(controller);
		rotatingSensor = new RotatingSensor(controller);
		advancedRotatingSensor = new AdvancedRotatingSensor(controller, 7);
	}
	
	public void start() {
		if(this.arbitrator != null) {
			this.arbitrator.go();
		}
	}
	
	public void stop() {
		if(this.arbitrator != null) {
			this.arbitrator.stop();
		}
	}
	
	/*
	 * Just bump sensor
	 */
	public void basicBumper() {
		Behavior [] bArray = {forward, rightBumper, leftBumper};
	    arbitrator = new Arbitrator(bArray);
	}
	
	/*
	 * Front sonic sensor and bumpers
	 */
	public void frontSensing() {
		Behavior [] bArray = {forward, rightBumper, leftBumper, frontSensing};
	    arbitrator = new Arbitrator(bArray);
	}
	
	/*
	 * Sonic sensor on side, attempts to follow wall
	 */
	public void followWall() {
		Behavior [] bArray = {forward, rightBumper, leftBumper, followWall};
	    arbitrator = new Arbitrator(bArray);
	}
	
	/*
	 * Gets bumped, then sensor on left side, if free space goes left else right
	 */
	public void bumpThenSense() {
		Behavior [] bArray = {forward, bumpSense};
	    arbitrator = new Arbitrator(bArray);
	}
	
	/*
	 * Sensor stops robot crashing, then scans right side, then left or goes backwards, goes to first space
	 */
	public void basicRotatingSensor() {
		Behavior [] bArray = {forward, rightBumper, leftBumper, rotatingSensor};
	    arbitrator = new Arbitrator(bArray);
	}
	
	/*
	 * Sensor stops robot crashing, then scans multiple locations in front of robot, moves to location with most space. 
	 */
	public void advancedRotatingSensor() {
		Behavior [] bArray = {forward, rightBumper, leftBumper, advancedRotatingSensor};
	    arbitrator = new Arbitrator(bArray);
	}

	public static void main(String[] args) {
		RobotController controller = new RobotController(Motor.A, Motor.D, SensorPort.S1, SensorPort.S4, SensorPort.S2, Motor.B);
		
		Robot robot = new Robot(controller);
		
		robot.advancedRotatingSensor();
		
		robot.start();
		
		while (!Button.ESCAPE.isDown()) {
			
		}
		
		robot.stop();
	}
}
