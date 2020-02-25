package main;
import Sensors.TouchSensor;
import Sensors.UltrasonicSensor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;

public class RobotController {
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private NXTRegulatedMotor extraMotor;
	
	private TouchSensor touchSensor1;
	private TouchSensor touchSensor2;
	private UltrasonicSensor ultrasonicSensor;
	
	public RobotController(NXTRegulatedMotor rightMotor, NXTRegulatedMotor leftMotor, Port touchSensor1Port, Port touchSensor2Port, Port ultrasonicSensorPort, NXTRegulatedMotor extraMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.extraMotor = extraMotor;
		
		if(touchSensor1Port != null) {
			this.touchSensor1 = new TouchSensor(touchSensor1Port);
		}
		
		if(touchSensor2Port != null) {
			this.touchSensor2 = new TouchSensor(touchSensor2Port);
		}
		
		if(ultrasonicSensorPort != null) {
			this.ultrasonicSensor = new UltrasonicSensor(ultrasonicSensorPort);
		}
	}
	
	public void setSpeed(float speed) {
		if(leftMotor != null && rightMotor != null) {
			leftMotor.setSpeed(speed);	
			rightMotor.setSpeed(speed);
		}
	}
	
	public int getSpeed() {
		return rightMotor.getSpeed();
	}
	
	public void moveForward() {
		if(leftMotor != null && rightMotor != null) {
			leftMotor.forward();
			rightMotor.forward();			
		}
	}
	
	public void moveBackward() {
		if(leftMotor != null && rightMotor != null) {
			leftMotor.backward();
			rightMotor.backward();			
		}
	}
	
	public void turnRight(int degrees) {
		if(leftMotor != null) {
			leftMotor.rotate(degrees * 4);
		}
	}
	
	public void turnLeft(int degrees) {
		if(rightMotor != null) {
			rightMotor.rotate(degrees * 4);
		}
	}
	
	public void stopMoving() {
		if(leftMotor != null && rightMotor != null) {
			rightMotor.stop(true);
			leftMotor.stop();
		}
	}
	
	public void rotateExtraMotor(int degrees) {
		if(extraMotor != null) {
			extraMotor.rotate(degrees);
		}
	}
	
	public boolean isTouchSensor1Pressed() {
		if(touchSensor1 == null) {
			return false;
		}
		
		return touchSensor1.isPressed();
	}
	
	public boolean isTouchSensor2Pressed() {
		if(touchSensor2 == null) {
			return false;
		}
		
		return touchSensor2.isPressed();
	}
	
	public boolean isCloseToObstacleUltrasonic() {
		if(ultrasonicSensor == null) {
			return false;
		}
		
		return ultrasonicSensor.isCloseToObstacle();
	}
	
	public float getSensorDistanceUltrasonic() {
		if(ultrasonicSensor == null) {
			return 0;
		}
		
		return ultrasonicSensor.getDistance();
	}
}