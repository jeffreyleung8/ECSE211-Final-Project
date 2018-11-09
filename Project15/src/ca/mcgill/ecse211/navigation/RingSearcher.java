package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.ev3.LocalEV3;
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
	
	//Tree Sides
	int count = 1;
	
	//Side motors
	private static final EV3LargeRegulatedMotor leftSideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	private static final EV3LargeRegulatedMotor rightSideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	//usdistance
	int usDistance = usSensor.fetch();
	 
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
		
		
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
				
			}
		if(detectRing()) {
			grabRing();
			}
		else if(!detectRing()) {
			moveToNextRing();
			searchRingSet();
		}
	}
	
	
	/**
	 * This method searches for the orange ring
	 * @return true if the ring is found
	 */
	public boolean detectRing() {
		
		//checking robot status
		//if(searchState == SearchState.IN_PROGRESS) {
			
/*------------- TODO:  check the int that represents the orange color ** -----------*/
		
		if(colorSensor.detect() == 1) {
			searchState = SearchState.RING_FOUND;
			return true;
		}			
		
		return false;
	}
	
	/*
	 * 
	 * 
 -Searching for ring set (supposed the ring set is at the center)
 -Call method searchRingSet(x,y) in RingSearcher
 -Stop robot when distance detected by US is less than d
 -Turn 45 left
 -Move forward by distance x
 -Turn 45 right
 -Move forward until distance detected by US is less than d
	 */
	
	/**
	 * This method searches the ring set
	 */
	public void searchRingSet() {
		if(usDistance<10) {
			robot.stopMoving();
			robot.turnBy(-45);
			robot.travelDist(20);
			robot.turnBy(45);
			while(usDistance>7) {
				robot.moveForward();
			}
		}
		
	}
	
	/**
	 * This method moves the robot to the next ring
	 */
	public void moveToNextRing() {

		//if(!detectRing()) {

/*-------- TODO: Test Correct Distance to Cover entire Tree , travel 25 cm for now ----------*/
		robot.travelDist(25);
		
		//turn 90 degrees
		robot.turnBy(90);
		
		//increment count
		count ++;
		
		if(count>4) {
			searchState = SearchState.TIME_OUT;
		}
			
		
		
		searchState = SearchState.IN_PROGRESS;
	}
	
	/**
	 * This method grabs a ring
	 */
	public void grabRing() {
		
		// if(searchState == SearchState.RING_FOUND)
		
			
/*---------- TODO : Needs Testing and tweaking the values --------------------------- */			
		robot.turnBy(90);
		
		//move up a little
		rightSideMotor.setSpeed(0);
		leftSideMotor.setSpeed(0);
		rightSideMotor.rotate(50);
		leftSideMotor.rotate(50);
		
		while(usDistance > 4){
			robot.moveForward();
		}
		
		//move up all the way
		rightSideMotor.setSpeed(0);
		leftSideMotor.setSpeed(0);
		rightSideMotor.rotate(40);
		leftSideMotor.rotate(40);
		
		//move back down
		rightSideMotor.setSpeed(0);
		leftSideMotor.setSpeed(0);
		rightSideMotor.rotate(-90);
		leftSideMotor.rotate(-90);
		
		//ring grabbed, move back from tree
		while(usDistance<10) {
			robot.moveBackward();
		}
			
		
	}
	
	/**
	 * This method unloads the rings
	 */
	public void unloadRing() {
		
		//move side motors down
		rightSideMotor.setSpeed(0);
		leftSideMotor.setSpeed(0);
		rightSideMotor.rotate(-90);
		leftSideMotor.rotate(-90);
		
	}


	
	
}