package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.main.*;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used for navigating the robot 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class Navigation extends Thread {

	//Motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	//Sensor
	private GyroSensorController gyroSensor;

	//Change in x ,y
	private double deltax;
	private double deltay;

	// current location of the vehicle
	private double currx;
	private double curry;
	private double currTheta;

	//Constants
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 120;

	private boolean navigate = true;

	//Odometer class
	private Odometer odometer;

	//RingSearcher class
	private RingSearcher ringSearcher;

	//Wifi class
	//private Wifi wifi;

	//Starting corner
	//int startingCorner;

	//Tunnel coordinates
	//private int[][] tunnelZone;

	//Search zone
	//private int[][] searchZone;

	/**
	 * This is a constructor for the RobotController class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param ringSearcher
	 * @param gyroSensor
	 */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, RingSearcher ringSearcher,GyroSensorController gyroSensor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.ringSearcher = ringSearcher;
		this.gyroSensor = gyroSensor;
		//this.wifi = wifi;
		//this.startingCorner = wifi.getStartingCorner();
		//this.tunnelZone = wifi.getTunnelZone();
		//this.searchZone = wifi.getSearchZone();


	}
	
	/**
	 * A method to drive our vehicle to the tunnel
	 */
	public void travelToTunnel() {
		//determine coordinates depending on team color
		//use travelTo
	}

	/**
	 * A method to drive our vehicle pass the tunnel
	 */
	public void travelThroughTunnel() {
		//determine coordinates depending on team color
		//use travelTo
	}

	/**
	 * A method to drive our vehicle to the search zone
	 */
	public void travelToSearchZone() {
		//determine coordinates depending on team color
		//use travelTo
	}

	/**
	 * A method to drive our vehicle to the search zone
	 */
	public void searchRing() {
		//Thread RingSearcher
	}

	/**
	 * A method to drive our vehicle to the search zone
	 */
	public void travelToStartingPoint() {
		//determine coordinates depending on team color
		//use travelTo
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	public void travelTo(double x, double y) {

		currx = odometer.getXYT()[0];
		curry = odometer.getXYT()[1];

		deltax = x - currx;
		deltay = y - curry;

		// Calculate the angle to turn around
		currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltax, deltay) - currTheta;

		double hypot = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Main.WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(Main.WHEEL_RAD, hypot), false);

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
			leftMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * A method to determine whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return boolean
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigate;
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

	private static void adjustTurn(int supposedAngle){
		//adjust turn with gyroscope
	}

}
