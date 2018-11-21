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

	//Home zone
	private int[][] homeZone;

	//Ring set
	private int[] ringSet;

	//Team
	private Team team;

	//Position of the robot in respect to the tunnel
	private boolean upperLeft = false;
	private boolean upperRight = false;
	private boolean lowerLeft = false;
	private boolean lowerRight = false;

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
		this.ringSearcher = ringSearcher;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.TILE_SIZE = robot.TILE_SIZE;
		this.SENSOR_LENGTH = robot.SENSOR_LENGTH;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.startingCorner = wifi.getStartingCorner(team);
		this.startingCornerCoords = wifi.getStartingCornerCoords();
		this.homeZone = wifi.getHomeZone(team);
		this.tunnelZone = wifi.getTunnelZone(team);
		this.ringSet = wifi.getRingSet(team);
	}


	/**
	 * A method to drive our vehicle to the tunnel
	 */
	public void travelToTunnel() {
		switch(team){
		case GREEN:{
			//tunnel is along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {
				int[] tunnelLR = tunnelZone[1];
				int[] tunnelLL = tunnelZone[0];
				int[] tunnelUR = tunnelZone[2];
				int[] tunnelUL = tunnelZone[3];
				switch(startingCorner){
				case 0: // corner 0
					//travel to lower left corner of tunnel
					robot.travelTo(tunnelLL[0],startingCornerCoords[1]);
					robot.travelTo(tunnelLL[0], tunnelLL[1]-1);
					lowerLeft = true;
					robot.turnTo(0); //assure that robot is pointing 0
					break;
				case 1:	// corner 1
					//travel to the point under lower-right corner 
					robot.travelTo(tunnelLR[0],startingCornerCoords[1]);
					robot.travelTo(tunnelLR[0], tunnelLR[1]-1);
					lowerRight = true;
					robot.turnTo(0); // assure that robot is pointing 0
					break;
				case 2: // corner 2
					//travel to upper right corner
					robot.travelTo(tunnelUR[0],startingCornerCoords[1]);
					robot.travelTo(tunnelUR[0], tunnelUR[1]+1);
					upperRight = true;
					robot.turnTo(180);
					break;
				case 3: // corner 3
					//travel to upper left corner
					robot.travelTo(tunnelUL[0],startingCornerCoords[1]);
					robot.travelTo(tunnelUL[0], tunnelUL[1]+1);
					upperLeft = true;
					robot.turnTo(180);
					break;
				}	
				break;
			}
			//tunnel is along x-axis (horizontal)
			else {
				int[] tunnelLR = tunnelZone[1];
				int[] tunnelLL = tunnelZone[0];
				int[] tunnelUR = tunnelZone[2];
				int[] tunnelUL = tunnelZone[3];
				switch(startingCorner){
				case 0:
					//travel to the point under lower-left corner
					robot.travelTo(tunnelLL[1], startingCornerCoords[0]);
					robot.travelTo(tunnelLL[0]-1,tunnelLL[1]);
					lowerLeft = true;
					robot.turnTo(90); // assure that robot is pointing 
					break;
				case 1:
					//travel to the point under lower-right corner 
					robot.travelTo(startingCornerCoords[0], tunnelLR[1]);
					robot.travelTo(tunnelLR[0]+1,startingCornerCoords[1]);
					lowerRight = true;
					robot.turnTo(270); // assure that robot is pointing */
					break;				 
				case 2:
					break;
				case 3:
					break;
				}
			}
			break;
		}
		case RED:{
			//tunnel is along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {
				switch(startingCorner){
				case 0:
				case 1:
				case 2:
				case 3: 
				}					

			}
			//tunnel is along x-axis (horizontal)
			else {
				int[] tunnelUL = tunnelZone[3];
				switch(startingCorner){
				case 0:
				case 1:
				case 2:
				case 3:	//travel to the point to the left  of the upper-left corner
					robot.travelTo(startingCornerCoords[0],tunnelUL[1]);
					robot.travelTo(tunnelUL[0]-1, tunnelUL[1]);
					robot.turnTo(90); //assure that the robot's orientation is 90
					upperLeft = true;
				}
			}
		}
		break;
		}
	}


	public void travelThroughTunnel(){

		turnToTunnel();

		//Correct at initial line
		odoCorr.correct(odometer.getXYT()[2]);

		robot.travelDist(TILE_SIZE);

		//Correct at line at the tunnel entrance
		odoCorr.correct(odometer.getXYT()[2]);

		//Move in the tunnel
		robot.setSpeeds(200,200);
		robot.travelDist(2*TILE_SIZE);

		//Correct at line at the tunnel exit
		odoCorr.correct(odometer.getXYT()[2]);
		robot.travelDist(TILE_SIZE);

		//Correct at next line
		odoCorr.correct(odometer.getXYT()[2]);
		robot.travelDist(SENSOR_LENGTH);
	}

	/**
	 * A method to travel to the ringSet
	 * 
	 */
	public void travelToRingSet() {
		switch(team) {
		case GREEN:
			//Tunnel along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {
				int urx = tunnelZone[2][0];
				int llx = tunnelZone[3][0];
				//Check where is the ring set in respect to the tunnel
				//if the ringset is at the left of the tunnel
				//travel to the right side of the ringset
				if((llx+urx)/2 > ringSet[0]) {
					int deltay = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE)-ringSet[1];  
					robot.travelDist(deltay * TILE_SIZE);
					robot.travelTo(ringSet[0]+1,ringSet[1]);
				}
				//if the ringset is at the right of the tunnel
				//travel to the left side of the ringset
				else {
					int deltay = ringSet[1]-(int) Math.round(odometer.getXYT()[1] / TILE_SIZE);  
					robot.travelDist(deltay * TILE_SIZE);
					robot.travelTo(ringSet[0]-1,ringSet[1]);
				}
			}
			//Tunnel along x-axis (horizontal)
			else {
			}

		case RED:
			//Tunnel along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {

			}
			//Tunnel along x-axis (horizontal)
			else {
				int ury = tunnelZone[2][1];
				int lly = tunnelZone[1][1];
				//Check where is the ring set in respect to the tunnel
				//if the ringset is at the left of the tunnel
				//travel to the right side of the ringset
				if((lly+ury)/2 > ringSet[1]) {
					int deltax = (int) Math.round(odometer.getXYT()[0] / TILE_SIZE)-ringSet[0];  
					robot.travelDist(deltax * TILE_SIZE);
					robot.travelTo(ringSet[0],ringSet[1]+1);
				}
				//if the ringset is at the right of the tunnel
				//travel to the left side of the ringset
				else {
					int deltax = ringSet[0]-(int) Math.round(odometer.getXYT()[0] / TILE_SIZE);  
					robot.travelDist(deltax * TILE_SIZE);
					robot.travelTo(ringSet[0],ringSet[1]-1);
				}
			}
		}

	}
	/**
	 * A method to travel to tunnel exit
	 * 
	 */
	public void travelToTunnelExit() {
		switch(team){
		case GREEN:
			//tunnel is along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {
				int[] tunnelUR = tunnelZone[2];
				switch(startingCorner){
				case 0: 
				
				case 1:	//travel to the point above upper-right corner
					int curry = (int) Math.round(odometer.getXYT()[1]/TILE_SIZE);
					robot.travelTo(tunnelUR[0],curry);
					robot.travelTo(tunnelUR[0], tunnelUR[1]+1);
					upperRight = true;
				case 2: 
					
				case 3:
				}	
			}
			//tunnel is along x-axis (horizontal)
			else {
				int[] tunnelUR = tunnelZone[2];
				switch(startingCorner){
				case 0:
				case 1: //travel to the point next to lower-right corner
					lowerRight = true;
				case 2:
				case 3:
				}
			}
		case RED:
			//tunnel is along y-axis (vertical)
			if(wifi.isTunnelVertical(team)) {
				switch(startingCorner){
				case 0:
				case 1:
				case 2:
				case 3: 
				}					

			}
			//tunnel is along x-axis (horizontal)
			else {
				int[] tunnelUL = tunnelZone[3];
				switch(startingCorner){
				case 0:
				case 1: //x then y
				case 2:
				case 3:	
				}
			}
		}
	}
	/**
	 * A method to travel to the starting point
	 * 
	 */
	public void travelToStartingPoint() {
		switch(startingCorner) {
		case 0: 
		case 1: //y then x
			int deltay = (int) Math.round(odometer.getXYT()[1] / TILE_SIZE)-startingCornerCoords[1];  
			robot.travelDist(deltay * TILE_SIZE);
			robot.travelTo(startingCornerCoords[0],startingCornerCoords[1]);
		case 2:
		}
	}
	/**
	 * A method to turn correctly to the tunnel entrance/exit
	 * 
	 */
	public void turnToTunnel() {
		//Tunnel is vertical
		if(wifi.isTunnelVertical(team)) {
			if(upperLeft) {
				robot.turnBy(90,false);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,true);
				upperLeft = false; //reinitialize
			}
			//Red tunnel along y-axis
			else if (upperRight) {
				robot.turnBy(90,true);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,false);
				upperRight = false; //reinitialize
			}
			//Green tunnel along y-axis
			else if (lowerLeft) {
				robot.turnBy(90,true);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,false);
				lowerLeft = false; //reinitialize
			}
			//Green tunnel along x-axis
			else if(lowerRight) {
				robot.turnBy(90,false);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,true);
				lowerRight = false; //reinitialize
			}
		}
		else {
			if(upperLeft) {
				robot.turnBy(90,true);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,false);
				upperLeft = false; //reinitialize
			}
			//Red tunnel along y-axis
			else if (upperRight) {
				robot.turnBy(90,false);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,true);
				upperRight = false; //reinitialize
			}
			//Green tunnel along y-axis
			else if (lowerLeft) {
				robot.turnBy(90,false);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,true);
				lowerLeft = false; //reinitialize
			}
			//Green tunnel along x-axis
			else if(lowerRight) {
				robot.turnBy(90,true);
				robot.travelDist(TILE_SIZE/2);
				robot.turnBy(90,false);
				lowerRight = false; //reinitialize
			}
		}
		
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

