package ca.mcgill.ecse211.controller;


import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicSensorController{
	
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	private float[] usData;
	
	private int distance = 0;
	
	private TextLCD lcd;

	public UltrasonicSensorController(EV3UltrasonicSensor usSensor, TextLCD lcd) {
		this.usSensor = usSensor;
		usDistance = this.usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];
		this.lcd = lcd;
	}
	
	public int fetch() {
		usDistance.fetchSample(usData, 0);
		distance =  (int) (usData[0] * 100.0);
		lcd.drawString("Distance: " + distance, 0, 5);

		return distance;
	}
	
	
}
