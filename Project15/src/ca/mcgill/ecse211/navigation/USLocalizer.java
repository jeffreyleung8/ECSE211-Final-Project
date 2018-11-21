package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the robot to the 0 degrees axis
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class USLocalizer {

	// vehicle constants
	public int ROTATE_SPEED;
	private double deltaTheta;

	//Odometer
	private Odometer odometer;

	//Robot
	private RobotController robot;

	//Sensor
	private UltrasonicSensorController usSensor;


	//Constant
	private int OPEN_SPACE = 50;
	private int WALL = 30;

	/**
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param usSensor
	 */
	public USLocalizer(Odometer odometer, RobotController robot,UltrasonicSensorController usSensor) {
		this.odometer = odometer;
		this.robot = robot;
		this.usSensor =usSensor;
		this.ROTATE_SPEED = 125;

	}


	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void usLocalize() {

		double angleA, angleB, turningAngle;
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		// Rotate to open space
		while (usSensor.fetch() < OPEN_SPACE) {
			robot.rotate(false);
		}
		
		// Rotate to the first wall
		while (usSensor.fetch() > WALL) {
			robot.rotate(false);
		}

		robot.stopMoving();
		Sound.beep();
		angleA = odometer.getXYT()[2];
		
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		
		// rotate out of the wall range
		while (usSensor.fetch() < OPEN_SPACE ) {
			robot.rotate(true);
		}
		
		// rotate to the second wall
		while (usSensor.fetch() > WALL) {
			robot.rotate(true);
		}

		robot.stopMoving();
		Sound.beep();
		angleB = odometer.getXYT()[2];

		robot.stopMoving();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		robot.turnBy(turningAngle,false);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}



}
