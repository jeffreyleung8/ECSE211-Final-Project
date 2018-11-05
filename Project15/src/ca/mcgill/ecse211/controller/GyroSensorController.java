package ca.mcgill.ecse211.controller;

import java.text.DecimalFormat;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * This class implements the gyro sensor controller
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class GyroSensorController {
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroAngle;
	private float[] gyroData;
	private TextLCD lcd;
	
	int angle = 0;

	/**
	 * This method implements the constructor of this class
	 * @param gyroSensor
	 * @param lcd
	 */
	public GyroSensorController(EV3GyroSensor gyroSensor, TextLCD lcd) {
		this.gyroSensor = gyroSensor;
		gyroAngle = this.gyroSensor.getAngleMode();
		gyroData = new float[gyroAngle.sampleSize()];
		this.lcd = lcd;
	}
	
	/**
	 * This method fetches samples from the gyro sensor a
	 * @return an angle (int)
	 */
	public int fetch() {
		gyroAngle.fetchSample(gyroData, 0);
		//DecimalFormat numberFormat = new DecimalFormat("######0.00");
		angle = (int) (gyroData[0]);
		lcd.drawString("Angle: " + angle, 0, 5);
		return angle;
	}
	
	/**
	 * This method sets the angle of the gyro sensor
	 * @param angle
	 */
	public void setAngle(int angle) {
		this.angle = angle;
	}
	
	/**
	 * This method gets the angle of the gyro sensor
	 * @return angle
	 */
	public int getAngle() {
		return angle;
	}

	
}
