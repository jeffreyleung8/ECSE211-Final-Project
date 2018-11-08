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
	public static int ROTATE_SPEED;
	private double deltaTheta;

	//Odometer
	private Odometer odometer;

	//Robot
	private RobotController robot;

	//Sensor
	private UltrasonicSensorController usSensor;


	//Constant
	private double d = 42.00;
	private double k = 15.00;

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
		this.usSensor = usSensor;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;

	}

	public void usLocalize() {
		//Choose when not facing a wall at first 
		double angleA=0.0, angleB=0.0, turningAngle=0.0;

		//Turn clockwise
		robot.setSpeeds(ROTATE_SPEED,ROTATE_SPEED);
		robot.rotate(true);
		int prevAvgDistance = usSensor.fetch();
		int deltaDistance;

		// If starting in front of wall
		while(usSensor.fetch() < d) {
			// Keep moving
		}

		// Check for wall 1
		while(robot.isMoving()) {
			int currAvgDistance = usSensor.fetch();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if(deltaDistance < 0 && currAvgDistance < d) {
				robot.setSpeeds(0,0);
				angleA = odometer.getXYT()[2];
			}
		}

		Sound.buzz();

		// Turn the other way (counterclockwise)
		robot.setSpeeds(ROTATE_SPEED,ROTATE_SPEED);
		robot.rotate(false);

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		while(usSensor.fetch() < 30) {
			// Keep moving
		}

		// Check for wall 2
		while(robot.isMoving()) {
			int currAvgDistance = usSensor.fetch();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if(deltaDistance < 0 && currAvgDistance < d) {
				robot.setSpeeds(0,0);
				angleB = odometer.getXYT()[2];

			}
		}
		
		Sound.buzz();

		//calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		robot.turnBy(turningAngle);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}
	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void usLocalize1() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (usSensor.fetch() < k) {
			robot.rotate(false);
		}
		// Rotate to the first wall
		while (usSensor.fetch() > d) {
			robot.rotate(false);
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate out of the wall range
		while (usSensor.fetch() < k) {
			robot.rotate(true);
		}

		// rotate to the second wall
		while (usSensor.fetch() > d) {
			robot.rotate(true);
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		robot.stopMoving();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		robot.turnBy(turningAngle);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}



}
