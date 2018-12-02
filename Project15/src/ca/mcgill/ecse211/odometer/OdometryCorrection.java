/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * This class allows the robot to correct itself (positiioning and odometer)
 *  by using two front light sensors(line detection)
 * 
 * @author Jeffrey Leung
 *
 */
public class OdometryCorrection {

	// Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_LENGTH;

	// Robot controller
	private RobotController robot;

	// Left and right light sensors
	private LightSensorController leftLS;
	private LightSensorController rightLS;

	// Odometer
	private Odometer odometer;

	private double color = 0.30;

	/**
	 * Construction of the odometryCorrection class
	 * @param odometer odometer of the robot (singleton)
	 * @param leftLS left front light sensor that is used
	 * @param rightLS right front light sensor that is used
	 * @param robot robot controller to control the motors
	 */
	public OdometryCorrection(Odometer odometer,RobotController robot,LightSensorController leftLS, LightSensorController rightLS) {
		this.odometer = odometer;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.TILE_SIZE = robot.TILE_SIZE;
		this.SENSOR_LENGTH = robot.SENSOR_LENGTH;
		this.robot = robot;
		this.leftLS = leftLS;
		this.rightLS = rightLS;
	}

	/**
	 * Correct position and odometer
	 * @param corrTheta orientation at which you want the robot to be/is
	 */
	public void correct(double corrTheta) {

		robot.travelDist(-7,150);

		robot.moveForward();

		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		while (!leftLineDetected && !rightLineDetected ) {
			//double rightSample = rightLS.fetch();
			//double leftSample = leftLS.fetch();
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

		correctOdo(corrTheta);

		robot.resetMotors();

		robot.setSpeeds(150, 150);

	}
	/**
	 * Correct odometer 
	 * @param theta orientation at which you want the robot to be/is
	 */
	private void correctOdo(double theta) {
		//Correction variables
		double corrX = 0;
		double corrY = 0;
		double corrTheta = roundTheta(theta);

		//Correction in X
		if (corrTheta == 90 || corrTheta == 270) {

			if (corrTheta == 90) {
				// Compute the sensors' X position in cm's
				double pos = odometer.getXYT()[0] + SENSOR_LENGTH;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrPos = (int) Math.round(pos / TILE_SIZE);

				// Get the correct X
				corrX = TILE_SIZE * corrPos - SENSOR_LENGTH;

			} else {
				// Compute the sensors' X position in cm's
				double pos = odometer.getXYT()[0] - SENSOR_LENGTH;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrPos = (int) Math.round(pos / TILE_SIZE);

				// Get the correct X
				corrX = TILE_SIZE * corrPos + SENSOR_LENGTH;

			}
			odometer.setX(corrX);

			//Correction in Y
		} else if (corrTheta == 0 || corrTheta == 180) {

			if (corrTheta == 0) {
				// Compute the sensors' Y position in cm's
				double pos = odometer.getXYT()[1] + SENSOR_LENGTH;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrPos = (int) Math.round(pos / TILE_SIZE);

				// Get the correct Y
				corrY = TILE_SIZE * corrPos - SENSOR_LENGTH;

				// Get the correct X
				//corrX = intermediateOdo[0] - (dTheta / Math.abs(dTheta) * offset);

			} else {
				// Compute the sensors' Y position in cm's
				double pos = odometer.getXYT()[1] - SENSOR_LENGTH;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrPos = (int) Math.round(pos / TILE_SIZE);

				// Get the correct Y
				corrY = TILE_SIZE * corrPos + SENSOR_LENGTH;

				// Get the correct X
				//corrX = intermediateOdo[0] + (dTheta / Math.abs(dTheta) * offset);
			}
			odometer.setY(corrY);
		}

		odometer.setTheta(corrTheta);

	}
	/**
	 * This method allows to round the angle received
	 * 
	 * @param theta angle to get rounded
	 * @return angle that is rounded to a general angle (integer)
	 */
	private double roundTheta(double theta){
		if(theta > 345 && theta < 15){
			return 0;
		}
		if(theta < 105 && theta > 75){
			return 90;
		}
		if(theta < 195 && theta > 165){
			return 180;
		}
		if(theta < 285 && theta > 255){
			return 270;
		}
		return 0;
	}
}
