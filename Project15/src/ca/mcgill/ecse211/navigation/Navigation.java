package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
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

	//Sensor
	private GyroSensorController gyroSensor;

	//Robot 
	private RobotController robot;
	
	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;

	private boolean navigate = true;

	//Odometer class
	private Odometer odometer;

	//RingSearcher class
	private RingSearcher ringSearcher;

	//Wifi class
	private WiFi wifi;

	//Starting corner
	int startingCorner;
	int[] startingCornerCoords;

	//Tunnel coordinates
	private int[][] tunnelZone;

	//Search zone
	private int[][] searchZone;
	
	//Team
	private Team team;

	/**
	 * This is a constructor for the RobotController class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param ringSearcher
	 * @param gyroSensor
	 */
	public Navigation(Odometer odometer,RobotController robot, RingSearcher ringSearcher,GyroSensorController gyroSensor, WiFi wifi) {
		this.odometer = odometer;
		this.robot = robot;
		this.ringSearcher = ringSearcher;
		this.gyroSensor = gyroSensor;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.startingCorner = wifi.getStartingCorner(team);
		this.startingCornerCoords = wifi.getStartingCornerCoords();
		this.tunnelZone = wifi.getTunnelZone(team);
		this.searchZone = wifi.getSearchZone(team);
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
	 * A method to determine whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return boolean
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigate;
	}


}
