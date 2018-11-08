package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the ring searcher
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class RingSearcher implements Runnable {

	//Robot
	private RobotController robot;
	
	//Sensors
	private ColorSensorController colorSensor;
	private UltrasonicSensorController usSensor;
	private GyroSensorController gyroSensor;
	
	//Wifi class
	//private Wifi wifi = new Wifi();

	//Odometer
	private Odometer odometer;
	
	//Search state
	private SearchState searchState;
	
	//Constants
	private long START_TIME;
	
	/**
	 *  Constructor for ring searcher
	 * @param colorSensor
	 * @param gyroSensor
	 * @param usSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RingSearcher(Odometer odometer,ColorSensorController colorSensor, 	UltrasonicSensorController usSensor,GyroSensorController gyroSensor, RobotController robot) {
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.gyroSensor = gyroSensor;
		this.robot = robot;
		this.odometer = odometer;
	}

	@Override 
	public void run(){
		//Thread
	}
	/**
	 * This method searches the ring set
	 */
	public void searchRingSet() {
		
	}
	
	/**
	 * This method searches for the orange ring
	 * @return true if the ring is found
	 */
	public boolean detectRing() {
		return false;
	}
	
	/**
	 * This method moves the robot to the next ring
	 */
	public void moveToNextRing() {
	}
	
	/**
	 * This method grabs a ring
	 */
	public void grabRing() {
	}
	
	/**
	 * This method unloads the rings
	 */
	public void unloadRing() {
		
	}


	
	
}