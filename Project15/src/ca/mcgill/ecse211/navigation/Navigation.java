package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.*;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used for navigating the robot
 * FOR BETA DEMO 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class Navigation extends Thread {

	//Sensor
	//private LightSensorController lightSensor;

	//Robot 
	private RobotController robot;

	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_LENGTH;
	private boolean navigate = true;


	//Odometer class
	private Odometer odometer;
	
	private OdometryCorrection odoCorr;

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
	
	//Ring set
	private int[] ringSet;

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
	public Navigation(Odometer odometer,RobotController robot, RingSearcher ringSearcher, WiFi wifi) {
		this.odometer = odometer;
		this.robot = robot;
		//this.lightSensor = lightSensor;
		this.ringSearcher = ringSearcher;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.TILE_SIZE = robot.TILE_SIZE;
		this.SENSOR_LENGTH = robot.SENSOR_LENGTH;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.startingCorner = wifi.getStartingCorner(team);
		this.startingCornerCoords = wifi.getStartingCornerCoords();
		this.tunnelZone = wifi.getTunnelZone(team);
		//this.searchZone = wifi.getSearchZone(team);
		this.ringSet = wifi.getRingSet(team);
	}

	/**
	 * A method to drive our vehicle to the tunnel
	 */
	public void travelToTunnel() {
		if(wifi.isTunnelVertical(team)) {
			int[] tunnelLR = tunnelZone[1];
			//travel to the point under lower-right corner
			robot.travelTo(tunnelLR[0],startingCornerCoords[1]);
			robot.travelTo(tunnelLR[0], tunnelLR[1]-1);
			//robot.travelTo(tunnelLR[0], tunnelLR[1]-1); //direct way

		}
		else {
			int[] tunnelUR = tunnelZone[3];
			//travel to the point next to the upper-right corner
			robot.travelTo(startingCornerCoords[0],tunnelUR[1]);
			robot.travelTo(tunnelUR[0]+1, tunnelUR[1]);
			//robot.travelTo(tunnelUR[0]+1, tunnelUR[1]); //direct way
		}
	}

	/**
	 * A method to drive our vehicle pass the tunnel
	 */
	public void travelThroughTunnel() {
		robot.setSpeeds(200, 200);
		robot.setAcceleration(200);
		if(wifi.isTunnelVertical(team)) {
			//assure that the robot is pointing 0 axis
			while(odometer.getXYT()[2] >= 350 || odometer.getXYT()[2]<=10) {
				robot.rotate(true);
			}
		}
		//assure that the robot is pointing 270
		else {
			while(odometer.getXYT()[2] >= 260 || odometer.getXYT()[2]<=280) {
				robot.rotate(true);
			}
		}
		//turn 45 to the left
		robot.setSpeeds(200, 200);
		robot.setAcceleration(200);
		robot.turnBy(45,false); 
		
		//hypothenus of tile
		robot.travelDist(24.55);
		//turn 45 to the right
		robot.setSpeeds(200, 200);
		robot.setAcceleration(200);
		robot.turnBy(42,true); 
		//Increase speed to pass over bump
		//robot.setSpeeds(280, 280);
		
		//robot.setAcceleration(280);
		
		odoCorr.correct(0);
		
		robot.travelDist(1.5*TILE_SIZE);
		//Move to first black line
		odoCorr.correct(0);

		odoCorr.correct(0);
		////////////////////////
		robot.setAcceleration(0);
		robot.setSpeeds(200, 200);
		robot.moveForward();
		////////////////////////
//		
//		float sample = lightSensor.fetch();
//		while (sample > 0.30) {
//			sample = lightSensor.fetch();
//			robot.moveForward();
//
//		}
		robot.stopMoving();
		robot.setSpeeds(200, 200);
		//Update odometer
		
		robot.travelDist(-16);
		
		if(wifi.isTunnelVertical(team)) {
			odometer.setTheta(0);

		}
		else {
			//odometer.setY((tunnelZone[0][0]-1)*TILE_SIZE);
			odometer.setTheta(270);
		}
	}
	/**
	 * A method to travel to the ringSet
	 * 
	 */
	public void travelToRingSet() {
		int ulx = tunnelZone[2][0];
		int urx = tunnelZone[3][0];
		int ury = tunnelZone[3][1];
		//Check where is the ring set in respect to the tunnel
		//if the ringset is at the left of the tunnel
		//travel to the right side of the ringset
		if((ulx+urx)/2 > ringSet[0]) {
			robot.travelTo((int)(odometer.getXYT()[0]/TILE_SIZE), ringSet[1]);
			robot.travelTo(ringSet[0]+1,ringSet[1]);
			
		}
		//if the ringset is at the right of the tunnel
		//travel to the left side of the ringset
		else {
			robot.travelTo((int)(odometer.getXYT()[0]/TILE_SIZE), ringSet[1]);
			robot.travelTo(ringSet[0]-1,ringSet[1]);		}	
		
		
	}
//	/**
//	 * A method to drive our vehicle to the search zone
//	 */
//	public void searchRing() {
//		Thread ringSearchThread = new Thread(ringSearcher);
//		ringSearchThread.run();
//	}
	
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
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}


}
