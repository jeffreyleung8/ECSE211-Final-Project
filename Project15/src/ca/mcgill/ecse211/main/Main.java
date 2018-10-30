/* This class serves to link all the classses together and display a menu when running the code  
 * 
 * @author Jeffrey Leung
 * @author Douglas So
 * @author Lea Akkary
 * @author Yassine Douida
 * @author Tushar Agarwal
 * @author Babette Smith
 */
package ca.mcgill.ecse211.main;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.controller.GyroSensorController;
import ca.mcgill.ecse211.tester.*;
import ca.mcgill.ecse211.controller.*;


public class Main {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//	private static final EV3LargeRegulatedMotor sideMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	//LCD Screen Object
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	//Sensor Object
	private static final EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);
	private static final EV3UltrasonicSensor us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static final EV3ColorSensor light = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static final EV3ColorSensor color = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 22; //9.8

	public static final double TILE_SIZE = 30.48;

	// Array to store { LLx, LLy, UUx, UUy, TR, SC }
	public static int[] variable = new int[] {3, 3, 6, 6 , 2, 0};

	public static int startingCorner = 0;
	
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
			
			//Controller
			GyroSensorController gyroSensor = new GyroSensorController(gyro,lcd);
			ColorSensorController colorSensor = new ColorSensorController(color);
			UltrasonicSensorController usSensor = new UltrasonicSensorController(us,lcd);
			LightSensorController lightSensor = new LightSensorController(light,lcd);
			
//			//Odometer objects
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, lcd, startingCorner);
			Display odometryDisplay = new Display(lcd); // No need to change
//
//			//Odometer thread
			Thread odoThread = new Thread(odometer);
			odoThread.start();
//
//			//Odometer display thread
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
//
//
//			//Center to 0 axis with USLocalizer (true: Rising edge / false: Falling edge) 
//			USLocalizer USLocalizerr = new USLocalizer(odometer, leftMotor, rightMotor, true, usSensor);
//			USLocalizerr.localizeRisingEdge();
//
//			//LightLocalizer Object
//			LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor,lightSensor);
//
//			//Localize robot to origin with LightLocalizer
//			lightLocalizer.moveToOrigin();
//			lightLocalizer.localize(1*TILE_SIZE,1*TILE_SIZE);
//			odometer.initialize(); //initialize odometer to startingCorner
//
//			//Navigate object
//			Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
//
//			//Navigate to (LLx,LLy)
//			//navigation.travelTo(1.0*TILE_SIZE, (variable[1])*TILE_SIZE);

//
//			//Localize on (LLx,LLy)
//			lightLocalizer.moveToOrigin();
//			lightLocalizer.localize(variable[0]*TILE_SIZE,variable[1]*TILE_SIZE);
//			odometer.reset();

		}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 


		System.exit(0);

	}
}