package Sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class UltrasonicSensor {

	EV3UltrasonicSensor sensor;
	private float[] sample;
	
	public UltrasonicSensor(Port port) {
		if(port == null) {
			throw new IllegalArgumentException("No port specified");
		}
		
		sensor = new EV3UltrasonicSensor(port);
		sample = new float[sensor.getDistanceMode().sampleSize()];
	}
	
	public boolean isCloseToObstacle() {
		sensor.getDistanceMode().fetchSample(sample, 0);
		
		return sample[0] < .1;
	}
	
	public float getDistance() {
		sensor.getDistanceMode().fetchSample(sample, 0);
		
		return sample[0];
	}
	
}
