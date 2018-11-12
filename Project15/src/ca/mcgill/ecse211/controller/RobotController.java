
package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.main.Main;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


/** This class serves to control the robot's motors
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class RobotController {

	// Motor objects
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;

	//Constants
	public final int FORWARD_SPEED = 200;
	public final int ROTATE_SPEED = 100;
	public final double TRACK = Main.TRACK;
	public final double WHEEL_RAD = Main.WHEEL_RAD;
	public final double TILE_SIZE = Main.TILE_SIZE;
	public final double SENSOR_LENGTH = Main.SENSOR_LENGTH;

	//Odometer
	private Odometer odometer;

	//Sensor
	private static GyroSensorController gyroSensor;
	/**
	 * This is a constructor for the RobotController class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RobotController(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, GyroSensorController gyroSensor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.gyroSensor = gyroSensor;
		this.odometer = odometer;
	}
	/*
	 * Get left motor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	/*
	 * Get right motor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x x-Coordinate
	 * @param y y-Coordinate
	 */
	public void travelTo(double x, double y) {

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];

		double deltax = x*TILE_SIZE - currx;
		double deltay = y*TILE_SIZE - curry;

		// Calculate the angle to turn around
		double currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltax, deltay) - currTheta;

		double hypot = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, hypot), false);

		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(true);

		Sound.beep();
	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {

		// ensures minimum angle for turning
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// rotate motors at set speed

		// if angle is negative, turn to the left
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, -(theta * 180) / Math.PI), false);
			//adjustTurn((theta * 180) / Math.PI);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, (theta * 180) / Math.PI), false);
			//adjustTurn((theta * 180) / Math.PI);
		}
	}

	/**
	 * Sets the speeds of the motors.
	 * 
	 * @param leftSpeed 
	 * @param rightSpeed 
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}

	/**
	 * Moves the robot forward
	 */
	public void moveForward() {
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Moves the robot backward 
	 */
	public void moveBackward() {
		leftMotor.backward();
		rightMotor.backward();

	}

	/**
	 * Stops the robot
	 */
	public void stopMoving() {
		leftMotor.stop(true);
		rightMotor.stop();

	}

	/**
	 * Turns the robot by the specified angle
	 * 
	 * @param dTheta 
	 */
	public void turnBy(double dTheta) {
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		adjustTurn((dTheta * 180) / Math.PI);
	}

	/**
	 * Moves the robot forward by the specified distance 
	 * 
	 * @param dist 
	 */
	public void travelDist(double distance) {
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}

	/**
	 * Rotate the robot clockwise or counterclockwise.
	 * 
	 * @param rotateClockwise 
	 * @param speed 
	 */
	public void rotate(boolean rotateClockwise) {
		if (rotateClockwise) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}
	/**
	 * Specifies whether the robot is current moving.
	 * 
	 * @return 
	 */
	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return 
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return the angle the robot needs to turn each wheel to rotate
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Adjust the turn of the robot using the gyroscope
	 * 
	 * @param angle  
	 */
	private static void adjustTurn(double angle){

		if(gyroSensor.getAngle() > (int) angle) {
			while(gyroSensor.fetch() >= (int)angle) {
				leftMotor.forward();
				rightMotor.backward();
			}
		}
		if(gyroSensor.getAngle() <(int) angle) {
			while(gyroSensor.fetch() <=(int) angle) {
				leftMotor.backward();
				rightMotor.forward(); 
			}
		}
	}

}