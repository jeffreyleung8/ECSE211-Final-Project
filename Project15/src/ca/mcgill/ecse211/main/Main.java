
package ca.mcgill.ecse211.main;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.tester.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.controller.*;

/** This class serves to link all the classses together and display a menu when running the code  
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class Main {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  //private static final EV3LargeRegulatedMotor sideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	//LCD Screen Object
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	//Sensor Object
	private static final EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);
	//	private static final EV3UltrasonicSensor us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	//	private static final EV3ColorSensor light = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	//	private static final EV3ColorSensor color = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	//Controllers
	GyroSensorController gyroSensor = new GyroSensorController(gyro,lcd);
	// ColorSensorController colorSensor = new ColorSensorController(color);
	// UltrasonicSensorController usSensor = new UltrasonicSensorController(us,lcd);
	// LightSensorController lightSensor = new LightSensorController(light,lcd);

	//Constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 14.35; 
	public static final double TILE_SIZE = 30.48;

	static int startingCorner = 0;


	//Navigation
	// USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, true, usSensor);
	// LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor,lightSensor);
	// RingSearcher ringSearcher = new RingSearcher(colorSensor, usSensor,gyroSensor, leftMotor,rightMotor, sideMotor);
	// Navigation navigation = new Navigation(odometer, leftMotor, rightMotor,ringSearcher,gyroSensor);


	public static void main(String[] args) throws OdometerExceptions {

		do {
			/*----------Test sensors----------*/	
			//			Tester test = new Tester(leftMotor, rightMotor,light,color,us,gyro,lcd);
			//			//Gyro
			//			//test.testGyro();
			//			//Color Sensor
			//			//test.testCS();
			//			///Light Sensor
			//			//test.testLS();
			//			//Ultrasonic sensor
			//			//test.testUS();
			//			//Turns
			//			test.testTurn();
			/*--------------END---------------*/
			
			//Odometer
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor,lcd,startingCorner);
			
			//Display
			Display odometryDisplay = new Display(lcd); 
			
			//Odometer thread
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			//Odometer display thread
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			//Center to 0 axis with USLocalizer (true: Rising edge / false: Falling edge) 
			//USLocalizer.localizeRisingEdge();

			//Localize robot to origin with LightLocalizer
			//lightLocalizer.initialLocalize(1*TILE_SIZE,1*TILE_SIZE); //TO CHANGE ARGUMENTS
			//odometer.initialize(); //initialize odometer to startingCorner
			
			//Navigation to tunnel entrance
			//navigation.travelToTunnel(); 
			
			//Navigation through tunnel 
			//navigation.travelThroughTunnel(); 

			//Navigaton to search zone(TR_LL)
			//navigation.travelToSearchZone();

			//Localize on (TR_LL)
			//lightLocalizer.localize();

			//Search for ring
			//navigation.searchRing();

			//Navigation to tunnel exit
			//navigation.travelToTunnel();

			//Navigation through tunnel 
			//navigation.travelThroughTunnel();

			//Navigation to starting point
			//navigation.travelToStartingPoint();

			//Unload ring
			//ringSearcher.unload();
		}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 


		System.exit(0);

	}
}