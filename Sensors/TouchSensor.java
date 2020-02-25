package Sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;

public class TouchSensor {
	
	private EV3TouchSensor sensor;
	private float[] sample;
	
	public TouchSensor(Port port) {
		if(port == null) {
			throw new IllegalArgumentException("No port specified");
		}
		
		sensor = new EV3TouchSensor(port);
		sample = new float[sensor.getTouchMode().sampleSize()];
	}
	
	public boolean isPressed() {
		sensor.getTouchMode().fetchSample(sample, 0);
		
		return sample[0] == 1;
	}
	
	
}
