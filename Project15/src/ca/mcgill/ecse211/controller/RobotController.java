
package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.main.Main;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;


/** This class serves to control the robot's motors
 *  It allows the robot to perform basic operations such as move forward, change speed,
 *  navigate to a specific point, to stop etc.
 *  Since it controls the mobility of the robot, it is essentially used in all navigation classes
 *  namely in usLocalizer, lightLocalizer, navigation and ringSearcher
 * 
 * @author Jeffrey Leung
 */

public class RobotController {

	// Motor objects
	private  static EV3LargeRegulatedMotor leftMotor;
	private  static EV3LargeRegulatedMotor rightMotor;

	private static final EV3LargeRegulatedMotor leftSideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightSideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	//Constants
	public final int FORWARD_SPEED = 350;
	public final int ROTATE_SPEED = 150;
	public final double TRACK = Main.TRACK;
	public final double WHEEL_RAD = Main.WHEEL_RAD;
	public final double TILE_SIZE = Main.TILE_SIZE;
	public final double SENSOR_LENGTH = Main.SENSOR_LENGTH;

	//Odometer
	private Odometer odometer;

	//Odometry correction
	private OdometryCorrection odoCorr;
	/**
	 * This is a constructor for the RobotController class
	 * @param odometer odometer of the robot (singleton)
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 */
	public RobotController(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x x-coordinate of the board
	 * @param y y-coordinate of the board
	 */
	public void directTravelTo(int x, int y) {

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];

		double deltax = (x*TILE_SIZE) - currx;
		double deltay = (y*TILE_SIZE) - curry;

		double mTheta = Math.toDegrees(Math.atan2(deltax, deltay));

		double hypot = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);
		//checkAngle((int)mTheta);

		resetMotors();

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, hypot), false);

		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(false);

		Sound.beep();
	}
	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate using 
	 * odometry correction
	 * 
	 * @param x x-Coordinate of the board
	 * @param y y-Coordinate of the board
	 */
	public void travelTo(int x, int y) {

		int deltax = 0, deltay = 0;

		//Nearest waypoint
		int lastX = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE);
		int lastY = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE);


		// Angle to turn to to go to the next point
		double corrTheta = 0;

		resetMotors();

		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		//Check if x-component is different
		if(lastX - x != 0) {
			if (lastX > x) {
				corrTheta = 270;
				deltax = -1;
			} 
			else {
				corrTheta = 90;
				deltax = 1;
			}

			// Rotate to the proper angle
			if (lastX != x)
				checkAngle((int)corrTheta);
			//turnTo(corrTheta);

			// Number of tiles to move in X
			int tilesX = x - lastX;

			// Advance towards next point's x coordinate
			for (int i = 1; i <= Math.abs(tilesX); i++) {

				// Correction at first line
				if (i == 1) {
					odoCorr.correct(corrTheta);
					this.travelDist(SENSOR_LENGTH,150);
					//directTravelTo(lastX+deltax*Math.abs(tilesX),lastY);
				}
				
				directTravelTo(lastX + deltax*i, lastY);

//				//Correction at the line
//				odoCorr.correct(corrTheta);

				// Move back by sensor_length at the last tile
				if (i == Math.abs(tilesX)) {
					odoCorr.correct(corrTheta);
					this.travelDist(SENSOR_LENGTH,150);
				}

			}
		}
		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		if(lastY - y != 0) {
			if (lastY > y) {
				corrTheta = 180;
				deltay = -1;
			} 
			else {
				corrTheta = 0;
				deltay = 1;
			}

			// Rotate to the proper angle
			if (lastY != y)
				checkAngle((int)corrTheta);

			// Number of tiles to move in Y
			int tilesY = y - lastY;

			// Advance towards next point's y coordinate
			for (int i = 1; i <= Math.abs(tilesY); i++) {

				// Correction at first line
				if (i == 1) {
					odoCorr.correct(corrTheta);
					this.travelDist(SENSOR_LENGTH,150);
				//	directTravelTo(lastX,deltay*Math.abs(tilesY)+lastY);
				}

				// travelToDirect() to the next closest point
				directTravelTo(lastX, lastY + deltay*i);

				// Correction at the line
				//odoCorr.correct(corrTheta);

				// Move back by sensor_length at the last tile
				if (i == Math.abs(tilesY)) {
					odoCorr.correct(corrTheta);
					this.travelDist(SENSOR_LENGTH,150);
				}
			}
		}

	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta angle you want the robot to be
	 */
	public void turnTo(double theta) {

		// Get current theta
		double currTheta = odometer.getXYT()[2];

		// Set angle displacement
		double dTheta = theta - currTheta;

		// Set speed to turn speed

		resetMotors();

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Calculate smallest angle
		if (dTheta > 180) {
			dTheta -= 360;
		} else if (dTheta < -180) {
			dTheta += 360;
		}
		if (dTheta == 180 || dTheta == -180) {
			dTheta = 180;
		}

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);

	}

	/**
	 * Check if robot is at the correct angle
	 *
	 * @param angle angle you want the robot to be
	 */
	public void checkAngle(int angle) {
		int current_angle = roundTheta(odometer.getXYT()[2]);

		if(current_angle != angle) {
			turnTo(angle);
		}
	}
	/**
	 * Sets the speeds of the motors.
	 * 
	 * @param leftSpeed speed of left motor
	 * @param rightSpeed speed of right motor
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}

	/**
	 * Moves the robot forward with synchronized motors
	 */
	public void moveForward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * Moves the robot backward with synchronized motors
	 */
	public void moveBackward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.endSynchronization();	
	}

	/**
	 * Stops the motors of the robot namely stop the robot
	 */
	public void stopMoving() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	/**
	 * Stops the specified motors of the robot with synchronization
	 * true - stop , false - don't stop
	 * @param stopLeft boolean stop leftMotor
	 * @param stopRight boolean stop rightMotor
	 */
	public void stopMoving(boolean stopLeft, boolean stopRight) {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		if (stopLeft)
			leftMotor.stop();
		if (stopRight)
			rightMotor.stop();
		leftMotor.endSynchronization();
	}

	/**
	 * Starts the specified motors of the robot.
	 * true - start , false - don't start
	 * @param startLeft boolean start leftMotor
	 * @param startRight boolean start rightMotor 
	 */
	public void startMoving(boolean startLeft, boolean startRight) {
		if (startLeft)
			leftMotor.forward();
		if (startRight)
			rightMotor.forward();
	}
	/**
	 * Set the acceleration of the motors.
	 * 
	 * @param acceleration the acceleration to set the left and right motor to
	 */
	public void setAcceleration(int acceleration) {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}

	/**
	 * Turns the robot by the specified angle
	 * 
	 * @param dTheta angle to turn
	 */
	public void turnBy(double dTheta, boolean clockwise) {
		if(clockwise) {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		}
		else {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		}
	}

	/**
	 * Moves the robot forward by the specified distance in cm
	 * positive distance - forward , negative distance - backward
	 * @param dist distance to travel
	 * @param speed speed of the motors
	 */
	public void travelDist(double distance,int speed) {

		resetMotors();
		setSpeeds(speed,speed);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}

	/**
	 * Rotate the robot clockwise or counterclockwise.
	 * 
	 * @param rotateClockwise true for clockwise, false for counterclockwise
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
	 * @return boolean true for moving, false for not moving
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
	 * @param radius radius of wheel
	 * @param distance desired distance to move the robot
	 * @return angle the robot needs to rotate its wheels in order to travel forward by
	 *         the distance provided
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius radius of wheel 
	 * @param width track of robot
	 * @param angle angle desired to turn the robot by
	 * @return the angle the robot needs to turn each wheel to rotate
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method allows to round the angle received
	 * 
	 * @param theta angle to get rounded
	 * @return angle that is rounded to a general angle (integer)
	 */
	private int roundTheta(double theta){
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

	/**
	 * This method allows to reset the motors
	 * 
	 */
	public void resetMotors() {
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
	}

	/**
	 * This method allows to rotate the side motors in order to lift the two frames
	 */
	public void turnMotor() {
		leftSideMotor.stop(true);
		rightSideMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftSideMotor, rightSideMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftSideMotor.setSpeed(30);
		rightSideMotor.setSpeed(30);
		leftSideMotor.rotate(-18,true);
		rightSideMotor.rotate(-18,false);
	}

	/**
	 * This method allows to unload the rings from the two frames by rotating 
	 * quickly the two side motors
	 * 
	 */
	public void unload() {
		leftSideMotor.setSpeed(150);
		rightSideMotor.setSpeed(150);
		leftSideMotor.rotate(-85, true);
		rightSideMotor.rotate(-85, false);
		leftSideMotor.setSpeed(250);
		rightSideMotor.setSpeed(250);
		leftSideMotor.rotate(100, true);
		rightSideMotor.rotate(100);

	}
	/**
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}

}