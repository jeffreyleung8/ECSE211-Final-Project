package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class implements the ultrasonic sensor controller
 * It is used in the usLocalizer which allows the robot to detect the walls so
 * that the robot can place itself parallel to the left wall
 *
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class UltrasonicSensorController{

	private SensorModes usSensor;
	private SampleProvider usDistance;
	private float[] usData;

	private TextLCD lcd;

	/**
	 * This is a constructor for this class
	 * @param usSensor ultrasonic sensor that is used
	 * @param lcd lcd screen of the ev3 block
	 */
	public UltrasonicSensorController(SensorModes usSensor, TextLCD lcd) {
		this.usSensor = usSensor;
		usDistance = this.usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];
		this.lcd = lcd;
	}

	/**
	 * This method fetches samples from the ultrasonic sensor
	 * 
	 * @return distance from sensor to object in cm
	 */
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		return distance;
	}
}
