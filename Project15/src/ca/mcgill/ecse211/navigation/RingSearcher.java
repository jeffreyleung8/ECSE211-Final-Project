package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the ring searcher
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class RingSearcher implements Runnable {

	//Motors
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	private EV3LargeRegulatedMotor sideMotor; 

	//Sensors
	private ColorSensorController colorSensor;
	private UltrasonicSensorController usSensor;
	private GyroSensorController gyroSensor;
	
	//Wifi class
	//private Wifi wifi = new Wifi();

	//Odometer
	//private Odometer odometer;

	/**
	 *  Constructor for ring searcher
	 * @param colorSensor
	 * @param gyroSensor
	 * @param usSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RingSearcher(ColorSensorController colorSensor, 	UltrasonicSensorController usSensor,GyroSensorController gyroSensor,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sideMotor) {
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.gyroSensor = gyroSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sideMotor = sideMotor;

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

	/**
	 * This method adjusts turns
	 */
	private static void adjustTurn(int supposedAngle){
		//adjust turn with gyroscope
	}

	
	
}