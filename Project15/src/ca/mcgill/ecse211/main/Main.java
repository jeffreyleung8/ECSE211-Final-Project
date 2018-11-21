
package ca.mcgill.ecse211.main;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.tester.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.controller.*;

/** This is the main class of the project. It is at the top level of the layered hierarchy.
 *  It serves to link all the classses together and make a sequence of methods classes from 
 *  the middle layer (navigation package) in order to execute the tasks of the game. 
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */


public class Main {

	//Constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 14.35; 
	public static final double TILE_SIZE = 30.48;
	public static final double SENSOR_LENGTH = 3.3;
	public static int[] startingCorner;

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//	private static final EV3LargeRegulatedMotor leftSideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
//	private static final EV3LargeRegulatedMotor rightSideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	//LCD Screen Object
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	//Sensor Object
	private static final EV3ColorSensor leftLight = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3UltrasonicSensor us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static final EV3ColorSensor color = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static final EV3ColorSensor rightLight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));


	//Odometer
	private static final Odometer odometer = Odometer.getOdometer(leftMotor,rightMotor);

	//Controllers
	private static ColorSensorController colorSensor = new ColorSensorController(color);
	private static UltrasonicSensorController usSensor = new UltrasonicSensorController(us,lcd);
	private static LightSensorController leftLS = new LightSensorController(leftLight,lcd);
	private static LightSensorController rightLS = new LightSensorController(rightLight,lcd);
	private static RobotController robot = new RobotController(odometer,leftMotor,rightMotor);
	private static OdometryCorrection odoCorr = new OdometryCorrection(odometer,robot,leftLS,rightLS);

	public static long START_TIME = System.currentTimeMillis();
	
	// WiFi class
	private static WiFi wifi = new WiFi();

	//Navigation
	private static USLocalizer usLocalizer = new USLocalizer(odometer,robot,usSensor);
	private static LightLocalizer lightLocalizer = new LightLocalizer(odometer,robot,leftLS, rightLS);
	private static RingSearcher ringSearcher = new RingSearcher(odometer,colorSensor, usSensor,robot);
	private static Navigation navigation = new Navigation(odometer,robot,ringSearcher,wifi);


	/**
	 * This is the central method in which it sequentically calls methods from the navigation package.
	 * It first localizes with the ultrasonic sensor, then localizes on the starting corner with the light 
	 * sensor. Then it navigates to and through the tunnel. Then it searches for the ring set and grabs a ring.
	 * Then it returns back to the starting corner by passing through the tunnel. 
	 * 
	 * @param args 
	 */
	public static void main(String[] args) throws OdometerExceptions {

		do {
			/*----------Test sensors----------*/	
			//	Tester test = new Tester(leftMotor, rightMotor,leftLight,color,us,lcd);
			//			//Gyro
			//			test.testGyro();
			//			//Color Sensor
			//			test.testCS();
			//			///Light Sensor
			//			test.testLS();
			//			//Ultrasonic sensor
			//			test.testUS();
			//			//Turns
			//			test.testTurn();
			/*--------------END---------------*/

			//Display
			Display odometryDisplay = new Display(lcd); 

			//Odometer thread
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			//Odometer display thread
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			//Set odometry correction
			robot.setOdoCorrection(odoCorr);
			navigation.setOdoCorrection(odoCorr);
			
			//localization
			usLocalizer.usLocalize();
			lightLocalizer.initialLocalize();
			
			//Initialize odometer
			odometer.initialize(wifi.getStartingCorner(wifi.getTeam()));
			
			//Navigation to tunnel entrance
			navigation.travelToTunnel(); 

			//Navigation through tunnel 
			navigation.travelThroughTunnel();

			//Navigation to ring set
			navigation.travelToRingSet();


			//Navigation to tunnel exit
			navigation.travelToTunnel();

			//Navigation through tunnel 
			navigation.travelThroughTunnel();

			//Navigation to starting point
			navigation.travelToStartingPoint();

			//Unload ring
			//ringSearcher.unload();
			
		}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 


		System.exit(0);

	}

}