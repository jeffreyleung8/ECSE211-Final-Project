package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class implements the ultrasonic sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class UltrasonicSensorController{

	private SensorModes usSensor;
	private SampleProvider usDistance;
	private float[] usData;

	
	//private boolean isRunning;

	private TextLCD lcd;

	/**
	 * This is a constructor for this class
	 * @param usSensor
	 * @param lcd
	 */
	public UltrasonicSensorController(SensorModes usSensor, TextLCD lcd) {
		this.usSensor = usSensor;
		usDistance = this.usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];
		this.lcd = lcd;
	}

	/**
	 * This method fetches samples from the ultrasonic sensor 
	 * @return integer distance from sensor to object
	 */
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		return distance;
	}
}
