package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class LightSensorController{
	
	private EV3ColorSensor lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	private float colorIntensity;
	private TextLCD lcd;
	
	public LightSensorController(EV3ColorSensor lightSensor, TextLCD lcd) {
		this.lightSensor = lightSensor;
		idColour = this.lightSensor.getRedMode();
		colorValue = new float[idColour.sampleSize()];
		this.lcd = lcd;
	}
	
	public float fetch() {
		idColour.fetchSample(colorValue, 0);
		colorIntensity = colorValue[0];
		lcd.drawString("Color intensity: " + colorIntensity, 0, 5);
		return colorIntensity;
	}
	
	
}
