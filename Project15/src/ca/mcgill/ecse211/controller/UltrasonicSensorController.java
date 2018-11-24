package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class implements the ultrasonic sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class UltrasonicSensorController{
	
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	//private SampleProvider average;
	private float[] usData;
	
	private int dist = 0;
	
	private TextLCD lcd;
	
	private int filterControl;
	private static final int FILTER_OUT = 20;
	
	/**
	 * This is a constructor for this class
	 * @param usSensor
	 * @param lcd
	 */
	public UltrasonicSensorController(EV3UltrasonicSensor usSensor, TextLCD lcd) {
		this.usSensor = usSensor;
		usDistance = this.usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];
		//average = new MeanFilter(usDistance,8);
		//usData = new float[average.sampleSize()];

		this.lcd = lcd;
	}
	
	/**
	 * This method fetches samples from the ultrasonic sensor 
	 * @return integer distance from sensor to object
	 */
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		//average.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		if (distance > 50) {
			distance = 255;
		}
//		lcd.clear();
//		lcd.drawString("Distance: " + distance, 0, 5);
		return distance;
	}
	
	
}
