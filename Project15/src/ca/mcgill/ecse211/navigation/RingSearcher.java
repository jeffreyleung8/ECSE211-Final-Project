package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.main.Main;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
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

	public static final double SENSOR_LENGTH = 3.3;

	//Odometer
	private Odometer odometer;

	//Search state
	public SearchState searchState;

	//Constants
	private long START_TIME = Main.START_TIME;

	//Odometry correction
	private OdometryCorrection odoCorr;

	//Tree Sides
	private int targetRing = 4;

	//	//Side motors
	//	private static final EV3LargeRegulatedMotor leftSideMotor = 
	//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	//
	//	private static final EV3LargeRegulatedMotor rightSideMotor = 
	//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	//usdistance
	//	int usDistance = usSensor.fetch();

	/**
	 *  Constructor for ring searcher
	 * @param colorSensor
	 * @param gyroSensor
	 * @param usSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RingSearcher(Odometer odometer,ColorSensorController colorSensor, UltrasonicSensorController usSensor, RobotController robot) {
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.robot = robot;
		this.odometer = odometer;
		searchState = SearchState.IN_PROGRESS;
	}

	@Override 
	public void run(){
		while(searchState == SearchState.IN_PROGRESS) {
			
			long timeElapsed = System.currentTimeMillis() - START_TIME;
			//Time out at 4 min
			if(timeElapsed >= 240000) {
				searchState = SearchState.TIME_OUT;
			}
			
			if(colorSensor.findMatch(colorSensor.fetch()) != 4) {
				colorSensor.beep();
				searchState = SearchState.RING_FOUND;
			}
		}
	}

	public void detectRing() {

		int color = colorSensor.findMatch(colorSensor.fetch()) ;

		while(color == 4) {
			color = colorSensor.findMatch(colorSensor.fetch()) ;	
		}
		colorSensor.beep();
		
		searchState = SearchState.RING_FOUND;
		
	}
	
	/**
	 * This method serves to move forward and backward in order to detect the ring
	 */
	public void approachRing() {
		odoCorr.correct(odometer.getXYT()[2]);

		while(usSensor.fetch() > 13) {
			robot.setSpeeds(70, 70);
			robot.moveForward();
		}
		
		while(searchState == SearchState.IN_PROGRESS) {
			if(usSensor.fetch() > 12) {
				while(usSensor.fetch()>12) {
					robot.moveForward();
				}
				robot.stopMoving();
			}


			if(usSensor.fetch() < 18) {
				while(usSensor.fetch()<18) {
					robot.moveBackward();
				}
				robot.stopMoving();
			}
		}

		robot.stopMoving();

		while(usSensor.fetch() > 5) {
			robot.setSpeeds(80, 80);
			robot.moveForward();
		}
		
		robot.stopMoving();
		
		robot.travelDist(-15);

		odoCorr.correct(odometer.getXYT()[2]);

		robot.travelDist(SENSOR_LENGTH);

	}

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
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}




}