package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RingSearcher {

	ColorSensorController colorSensor;
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	
	// Constructor for navigation
	public RingSearcher(ColorSensorController colorSensor,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.colorSensor = colorSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}
	
}