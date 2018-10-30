package ca.mcgill.ecse211.controller;

import java.text.DecimalFormat;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroSensorController{
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroAngle;
	private float[] gyroData;
	private TextLCD lcd;
	
	int angle = 0;

	public GyroSensorController(EV3GyroSensor gyroSensor, TextLCD lcd) {
		this.gyroSensor = gyroSensor;
		gyroAngle = this.gyroSensor.getAngleMode();
		gyroData = new float[gyroAngle.sampleSize()];
		this.lcd = lcd;
	}
	
	public int fetch() {
		gyroAngle.fetchSample(gyroData, 0);
		//DecimalFormat numberFormat = new DecimalFormat("######0.00");
		angle = (int) (gyroData[0]);
		lcd.drawString("Angle: " + angle, 0, 5);
		return angle;
	}
	
	public void setAngle(int angle) {
		this.angle = angle;
	}
	
	public int getAngle() {
		return angle;
	}
	
}
