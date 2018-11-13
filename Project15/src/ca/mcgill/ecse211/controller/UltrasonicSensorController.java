package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class implements the ultrasonic sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class UltrasonicSensorController{
	
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
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
		this.lcd = lcd;
	}
	
	/**
	 * This method fetches samples from the ultrasonic sensor 
	 * @return integer distance from sensor to object
	 */
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		if(distance == 2147483647) {
			distance = 100;
		}
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value: do not set the distance var, do increment the filter value
			this.filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			dist = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			this.filterControl = 0;
			dist = distance;
		}
		
		lcd.drawString("Distance: " + dist, 0, 5);
		return dist;
	}
	
	
}
