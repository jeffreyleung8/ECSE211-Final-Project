package ca.mcgill.ecse211.gyroscope;

import java.text.DecimalFormat;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Gyroscope implements Runnable {
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroAngle;
	private float[] gyroData;
	private TextLCD lcd;
	
	float angle;

	public Gyroscope(EV3GyroSensor gyroSensor, TextLCD lcd) {
		this.gyroSensor = gyroSensor;
		gyroAngle = this.gyroSensor.getAngleMode();
		gyroData = new float[gyroAngle.sampleSize()];
		this.lcd = lcd;
	}
	
	public void run() {
		lcd.clear();
		while (true) {
			gyroAngle.fetchSample(gyroData, 0);
			//DecimalFormat numberFormat = new DecimalFormat("######0.00");
			angle = (gyroData[0]);
			lcd.drawString("Angle: " + angle, 0, 5);
			Delay.msDelay(500);
			
		}
	}
	
	public void setAngle(float angle) {
		this.angle = angle;
	}
	
	public float getAngle() {
		return angle;
	}
	
}
