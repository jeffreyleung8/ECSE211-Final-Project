package ca.mcgill.ecse211.navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the cart to the origin
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class LightLocalizer {

	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final static double TILE_SIZE = Main.TILE_SIZE;
	private final static double SENSOR_LENGTH = Main.SENSOR_LENGTH;

	private Odometer odometer;

	private static RobotController robot;

	private static LightSensorController leftLS;
	private static LightSensorController rightLS;

	private double color = 0.30;
	/**
	 * This is a constructor for this class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param lightSensor
	 */
	public LightLocalizer(Odometer odometer,RobotController robot,LightSensorController leftLS,LightSensorController rightLS ) {
		this.odometer = odometer;
		this.robot = robot;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;		
		this.leftLS = leftLS;
		this.rightLS = rightLS;
	}

	/**
	 * This method localizes the robot from the starting point
	 */
	public void initialLocalize() {

		// Start moving the robot forward
		robot.setSpeeds(200,200);

		robot.moveForward();

		correct();

		robot.travelDist(SENSOR_LENGTH,200);
		robot.turnBy(90,true);

		robot.setSpeeds(200,200);
		robot.moveForward();

		correct();

		robot.travelDist(SENSOR_LENGTH,200);

		robot.turnBy(90, false); //90 is a bit too much

		Sound.beep();
		Sound.beep();
		Sound.beep();
	}

	/**
	 * This method serves to correct the orientation of the robot for the initial 
	 * light localization
	 */
	private void correct() {

		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected ) {
			if (rightLS.fetch() < color) {
				rightLineDetected = true;
				// Stop the right motor
				robot.stopMoving(false, true);

			} else if (leftLS.fetch() < color) {
				leftLineDetected = true;

				// Stop the left motor
				robot.stopMoving(true, false);
			}
		}

		// Get the odometer's reading 

		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected)) {
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLS.fetch() < color) {
				leftLineDetected = true;
				robot.stopMoving();
			} else if (leftLineDetected && rightLS.fetch() < color) {
				rightLineDetected = true;
				robot.stopMoving();
			}
		}

	}




}
