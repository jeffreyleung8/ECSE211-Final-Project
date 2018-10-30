/* This class serves to drive the robot to the 0 degreee axis
 * 
 * @author Jeffrey Leung
 * @author Douglas So
 * @author Lea Akkary
 * @author Yassine Douida
 * @author Tushar Agarwal
 * @author Babette Smith
 */
package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.main.*;

public class USLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double deltaTheta;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private UltrasonicSensorController usSensor;
//	private EV3UltrasonicSensor usSensor;
//	private float[] usData;
//	private SampleProvider usDistance;
//	int filterControl = 0;
//	int dist = 0;
//	private static final int FILTER_OUT = 20;

	// Create a navigation
	public Navigation navigation;

	private double d = 42.00;
	private double k = 15.00;

	/*
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param usSensor
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			UltrasonicSensorController usSensor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		navigation = new Navigation(odometer, leftMotor, rightMotor);

		this.usSensor = usSensor;
		//USsensor
//		this.usSensor = usSensor;
//		this.usDistance = this.usSensor.getMode("Distance");
//		this.usData = new float[usDistance.sampleSize()];
	}
//	/**
//	 * A method to get the distance from our sensor
//	 * 
//	 * @return
//	 */
//	private int usSensor.fetch() {
//		usDistance.fetchSample(usData, 0);
//		int distance =  (int) (usData[0] * 100.0);
//		// rudimentary filter - toss out invalid samples corresponding to null signal 
//		if (distance >= 255 && filterControl < FILTER_OUT) {
//			// bad value: do not set the distance var, do increment the filter value
//			this.filterControl++;
//		} else if (distance >= 255) {
//			// We have repeated large values, so there must actually be nothing
//			// there: leave the distance alone
//			this.dist = distance;
//		} else {
//			// distance went below 255: reset filter and leave
//			// distance alone.
//			this.filterControl = 0;
//			this.dist = distance;
//		}
//		if(dist == 2147483647) {
//			return 200;
//		}
//		LCD.drawString(""+ this.dist, 0, 5);
//		LCD.clear(5);
//
//		return this.dist;
//	}

	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (usSensor.fetch() < k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (usSensor.fetch() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate out of the wall range
		while (usSensor.fetch() < k) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (usSensor.fetch() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, turningAngle), true);
		rightMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, turningAngle), false);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * A method to localize position using the rising edge
	 * 
	 */
	public void localizeRisingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to the wall
		while (usSensor.fetch() > k ) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (usSensor.fetch() < d ) { //d+k
			leftMotor.backward();
			rightMotor.forward();
		}

		Sound.playNote(Sound.FLUTE, 880, 250);

		// record angle
		angleA = odometer.getXYT()[2];

		// rotate the other way all the way until it sees the wall
		while (usSensor.fetch() > k ) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate until it sees open space
		while (usSensor.fetch() < d) { //d+k
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.playNote(Sound.FLUTE, 880, 250);
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();


		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 using turning angle and we account for small
		// error
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, turningAngle), true); ///////
		rightMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, turningAngle), false);///////

		odometer.setXYT(0,0,0);
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
