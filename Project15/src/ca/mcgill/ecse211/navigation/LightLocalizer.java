package ca.mcgill.ecse211.navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the cart to the origin
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class LightLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 130;

	//TO CHANGE
	private double SENSOR_LENGTH = 12.5;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	public Navigation navigation;
	
	private LightSensorController lightSensor;

	private float sample;

	double[] lineData;

	/**
	 * This is a constructor for this class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param lightSensor
	 */
	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,LightSensorController lightSensor) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		//navigation = new Navigation(odometer, leftMotor, rightMotor);
		
		this.lightSensor = lightSensor;
		lineData = new double[5];
	}
	
	/**
	 * This method localizes the robot from the starting point
	 * @param x - coordinate
	 * @param y - coordinate
	 */
	public void initialLocalize(double x, double y) {
		
		moveToOrigin();
		
		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		//Detect the four lines and record the angle at which it is seen
		while (index < 4) {

			leftMotor.forward();
			rightMotor.backward();

			sample = lightSensor.fetch();

			if (sample < 0.20) {
				lineData[index] = odometer.getXYT()[2];
				Sound.playNote(Sound.FLUTE, 880, 250);
				index++;
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[4];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to origin to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		navigation.travelTo(0,0);

		leftMotor.setSpeed(ROTATION_SPEED / 2);
		rightMotor.setSpeed(ROTATION_SPEED / 2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, -odometer.getXYT()[2]), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, -odometer.getXYT()[2]), false);
		}

		leftMotor.stop(true);
		rightMotor.stop();
		odometer.setXYT(x, y, 0.0);
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 * @param x - coordinate
	 * @param y - coordinate
	 */
	public void localize(double x, double y) {
		
		//Turn until the robot is not on a black line
		while(lightSensor.fetch() < 20) {
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.backward();
			rightMotor.forward();
		}
		
		//Start localization
		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		//Detect the four lines and record the angle at which it is seen
		while (index < 4) {

			leftMotor.forward();
			rightMotor.backward();

			sample = lightSensor.fetch();

			if (sample < 0.20) {
				lineData[index] = odometer.getXYT()[2];
				Sound.playNote(Sound.FLUTE, 880, 250);
				index++;
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[4];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to origin to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		navigation.travelTo(0,0);

		leftMotor.setSpeed(ROTATION_SPEED / 2);
		rightMotor.setSpeed(ROTATION_SPEED / 2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, -odometer.getXYT()[2]), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, -odometer.getXYT()[2]), false);
		}

		leftMotor.stop(true);
		rightMotor.stop();
		odometer.setXYT(x, y, 0.0);
	}

	/**
	 * This method moves the robot towards the origin
	 */
	public void moveToOrigin() {


		if(odometer.getXYT()[2] <= 10.00 || odometer.getXYT()[2] >=350) {
			navigation.turnTo(Math.PI /4);
		}

		leftMotor.setSpeed(ROTATION_SPEED+60);
		rightMotor.setSpeed(ROTATION_SPEED+60);

		// get sample
		sample = lightSensor.fetch();

		// move forward past the origin until light sensor sees the line
		while (sample > 0.20) {

			sample = lightSensor.fetch();
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();

		// Move backwards so our origin is close to origin
		leftMotor.rotate(convertDistance(Main.WHEEL_RAD, -11), true); ////////////
		rightMotor.rotate(convertDistance(Main.WHEEL_RAD, -11), false);///////////

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
