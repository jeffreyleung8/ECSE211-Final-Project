package ca.mcgill.ecse211.tester;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Tester {
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	private EV3ColorSensor lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	
	private EV3ColorSensor colorSensor;
	private SampleProvider rgbValue;
	private float[] rgbData;


	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	private float[] usData;
	
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroAngle;
	private float[] gyroData;
	
	private TextLCD lcd;
	
	
	public Tester(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,EV3ColorSensor lightSensor,EV3ColorSensor colorSensor,
					EV3UltrasonicSensor usSensor,EV3GyroSensor gyroSensor,TextLCD lcd) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.lightSensor = lightSensor;
		idColour = this.lightSensor.getRedMode();
		colorValue = new float[idColour.sampleSize()];
		
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getRGBMode();
		rgbData = new float[rgbValue.sampleSize()];
		
		this.usSensor = usSensor;
		usDistance = usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];

		this.gyroSensor = gyroSensor;
		gyroAngle = gyroSensor.getAngleMode();
		gyroData = new float[gyroAngle.sampleSize()];
		
		this.lcd = lcd;
	}
	
	public void testGyro() {
		lcd.clear();
		while (true) {
			gyroAngle.fetchSample(gyroData, 0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("Angle: " + numberFormat.format(gyroData[0]), 0, 1);
			Delay.msDelay(500);
			
		}
	}
	public void testUS() {
		lcd.clear();
		while(true) {
			usDistance.fetchSample(usData, 0);
			//int distance =  (int) (usData[0] * 100.0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("Distance: " + numberFormat.format(usData[0]*100), 0, 1);
			Delay.msDelay(1000);
		}
		
	}
	
	public void testLS() {
		lcd.clear();
		while(true) {
			idColour.fetchSample(colorValue, 0);
			lcd.drawString("Color intensity: ", 0, 1);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("  " + numberFormat.format(colorValue[0]), 0, 2);
			Delay.msDelay(1000);
			
		}

	}
	
	public void testCS() {
		lcd.clear();
		while(true) {
			rgbValue.fetchSample(rgbData, 0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("R: " + numberFormat.format(rgbData[0]*1000), 0, 2);
			lcd.drawString("G: " + numberFormat.format(rgbData[1]*1000), 0, 3);
			lcd.drawString("B: " + numberFormat.format(rgbData[2]*1000), 0, 4);
			Delay.msDelay(1000);
		}

	}
	public void testTurn() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn 90 degrees to avoid obstacle
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		// move to the left by a tile size
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn again 90 back and move past the block
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn again 90 back and move past the block
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 20), false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn 90 degrees to avoid obstacle
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

	}
	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
