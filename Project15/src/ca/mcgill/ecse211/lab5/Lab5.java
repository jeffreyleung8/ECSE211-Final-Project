/* This class serves to link all the classses together and display a menu when running the code  
 * 
 * @author Jeffrey Leung
 * @author Douglas So
 * @author Lea Akkary
 * @author Yassine Douida
 * @author Tushar Agarwal
 * @author Babette Smith
 */
package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.*;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.lab5.*;



public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");

	private static final EV3ColorSensor colosSamplerSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK =17; //9.8

	public static final double TILE_SIZE = 30.48;

	// Array to store { LLx, LLy, UUx, UUy, TR, SC }
	public static int[] variable = new int[] {3, 3, 6, 6 , 2, 0};


	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice=0;
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive with navigation or avoidance
			lcd.drawString(" Detect | Search  ", 0, 0);
			lcd.drawString("  Color |  Ring   ", 0, 1);
			lcd.drawString("        |         ", 0, 2);
			lcd.drawString("  Left  |  Right  ", 0, 3);
			lcd.drawString("        | 	      ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			do {
				lcd.clear();
				int targetRing = variable[4];
				ColorClassification colorClass = new ColorClassification(colosSamplerSensor);
				float[] rgb;
				int color = 5;
				while(true) {

					do {
						rgb = colorClass.fetch();
						color= ColorClassification.findMatch(rgb);
					} 
					while (color == 4);

					if (color== targetRing-1){
						Sound.playNote(Sound.FLUTE, 880, 250);
					} 
					else {
						Sound.twoBeeps();

					}
					String[] str= {"Blue  ", "Green  ", "Yellow  ", "Orange  "};
					lcd.drawString("Object Detected ", 0, 0);
					lcd.drawString(str[color], 0, 1);
					DecimalFormat numberFormat = new DecimalFormat("######0.00");
					lcd.drawString("R: " + numberFormat.format(rgb[0]*1000), 0, 2);
					lcd.drawString("G: " + numberFormat.format(rgb[1]*1000), 0, 3);
					lcd.drawString("B: " + numberFormat.format(rgb[2]*1000), 0, 4);
					//1 blue, 2 green 3 yellow 4 orange
				}

			}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 

		} else {
			// clear the display
			lcd.clear();
			do {
				// { LLx, LLy, UUx, UUy, TR, SC }
				//variable = new int[] {3, 3, 6, 6, 1, 0}; //temporary
				//variable = menu(variable);

				//Odometer objects
				Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD,variable[5]);
				Display odometryDisplay = new Display(lcd); // No need to change

				//Odometer thread
				Thread odoThread = new Thread(odometer);
				odoThread.start();

				//Odometer display thread
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();


//				//Ultrasonic objects
//				@SuppressWarnings("resource") //Because we don't bother to close this resource
//				SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
//				SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
//
//				//Center to 0 axis with USLocalizer (true: Rising edge / false: Falling edge) 
//				USLocalizer USLocalizerr = new USLocalizer(odometer, leftMotor, rightMotor, true, usDistance);
//				USLocalizerr.localizeRisingEdge();
//
//				//LightLocalizer Object
//				LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor);
//
//				//Localize robot to origin with LightLocalizer
//				lightLocalizer.moveToOrigin();
//				lightLocalizer.localize(1*TILE_SIZE,1*TILE_SIZE);
//				odometer.initialize(); //initialize odometer to startingCorner
//
				//Navigate object
				Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);

				//Navigate to (LLx,LLy)
				//	navigation.travelTo(1.0*TILE_SIZE, (variable[1])*TILE_SIZE);
				navigation.travelTo(0,60);
				navigation.travelTo(60,60);
				navigation.travelTo(60,0);
				navigation.travelTo(0,0);
//
//				//Localize on (LLx,LLy)
//				lightLocalizer.moveToOrigin();
//				lightLocalizer.localize(variable[0]*TILE_SIZE,variable[1]*TILE_SIZE);
//				odometer.reset();
//
//				//Obstacle Avoidance object
//				ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(odometer, leftMotor, rightMotor, usDistance, variable,lightLocalizer,colosSamplerSensor,lcd);
//
//				//Path thread
//				Thread snakeThread = new Thread(obstacleAvoidance);
//				snakeThread.run();
//
//				Sound.beep();
//
			}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 

		} while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		System.exit(0);

	}
	/**
	 * This method prints menu to choose llx,lly,urx,ury,targetRing,startingCenter
	 * @param label
	 * @param content
	 * @return (button)
	 */
	public static int printMenu(String label, int content) {
		int buttonChoice;
		do {
			// clear the display
			lcd.clear();

			lcd.drawString(label, 0, 0);
			lcd.drawString(""+content+"", 8, 2);
			lcd.drawString("<  Left |  Right >", 0, 4);
			lcd.drawString("Press Enter", 4, 7);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT 
				&& buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_ESCAPE);
		return buttonChoice;
	}

	/**
	 * This method allows to create label and update the menu
	 * @param variable array
	 * @return (variable array)
	 */
	public static int[] menu(int[] variable) {
		String tag[] = {"LLx","LLy","URx","URy","TR ","SC "};
		String label[] = new String[6];
		for (int i=0; i<label.length;i++) {
			label[i]="<   Choose "+tag[i]+"   >";
		}
		int maxRange[] = {8,8,8,8,4,3};
		int minRange[] = {0,0,0,0,1,0};
		int labelContent[]= {0,0,0,0,1,0};

		int i =0;
		while(i < 6) {
			int buttonChoice;
			buttonChoice = printMenu(label[i],labelContent[i]);
			if (buttonChoice == Button.ID_LEFT) {
				if(labelContent[i]!= minRange[i]) {
					labelContent[i]--;
				}
			}
			if (buttonChoice == Button.ID_RIGHT) {
				if(labelContent[i]!= maxRange[i]) {
					labelContent[i]++;
				}
			}
			if (buttonChoice == Button.ID_ENTER) {
				variable[i]=labelContent[i];
				i++; 
			}
			if(buttonChoice == Button.ID_ESCAPE) {
				System.exit(0);
			}
		}
		return variable;

	}


}