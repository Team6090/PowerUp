/*********************************************************************************************************
 * FLOW MAP
 * 
 * Auton
 * |- Switch-Scale Position
 *    |- Go for switch
 *    	 |- Robot Start Position: Left
 *       |- Robot Start Position: Center
 *       |- Robot Start Position: Right
 *    |- Go for scale
 *    	 |- Robot Start Position: Left
 *       |- Robot Start Position: Center
 *       |- Robot Start Position: Right
 * |- Variables
 * |- INIT
 * 
 * Total Scenerios: 4 run methods x 6 scenerios = 24 Total Auton scenerios
 *********************************************************************************************************/


package org.usfirst.frc.team6090.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;				//import CAN bus Talon SRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX;				//import CAN bus Victor SRX


/*********************************************************************************************************
 * Auton scenerios based off of scale and switch positions.
 * The format is Switch-Scale, so LR means the switch closest to the bot at the beginning of that match
 * is in the left position and the scale is in the right position.
 *********************************************************************************************************/
@SuppressWarnings("unused") // Suppress warnings about unused variables - REMOVE BEFORE FINAL REVISION
public class Auton 
	{
	/**
	 * The position in which we start the patch. This will be set with a toggle button.
	 * This is set from the Shuffleboard interface from a drop-down
	 * -1 - No value set
	 * 0  - Left
	 * 1  - Center
	 * 2  - Right
	 */
	//static double matchStartPosition = SmartDashboard.getNumber("Auton/Match Start Positon", -1);
	static double matchStartPosition = -1;
	/**
	 * Whether or not we want to go for the scale. If false, just go for the switch. 
	 * This essentially doubles all the options we have to create.
	 */
	static double goForScale;
	
	
	
	static TalonSRX leftMotor;								//Reserve memory for variable for Left Talon CAN bus controller
	static TalonSRX rightMotor;								//Reserve memory for variable for Right Talon CAN bus controller
	static TalonSRX winchMotor;								//Reserve memory for variable for Winch Talon CAN bus controller
	static TalonSRX latchMotor;								//Reserve memory for variable for Latch Talon CAN bus controller
	static VictorSPX leftSlaveMotor;						//Reserve memory for variable for Left Victor CAN bus controller
	static VictorSPX rightSlaveMotor;						//Reserve memory for variable for Right Victor CAN bus controller
	static VictorSPX winchSlaveMotor;						//Reserve memory for variable for Winch Victor CAN bus controller
	
	
	public static void autonInit() {
		leftMotor = new TalonSRX(1);				//Create a new instance for Left Talon SRX CAN bus motor controller
    	leftSlaveMotor = new VictorSPX(2);			//Create a new instance for Left Victor SPX CAN bus motor controller
    	rightMotor = new TalonSRX(3);				//Create a new instance for Right Talon SRX CAN bus motor controller
    	rightSlaveMotor = new VictorSPX(4);			//Create a new instance for Right Victor SPX CAN bus motor controller
    	winchMotor = new TalonSRX(5);				//Create a new instance for Winch Talon SRX CAN bus motor controller
    	winchSlaveMotor = new VictorSPX(6);			//Create a new instance for Left Victor SPX CAN bus motor controller
    	latchMotor = new TalonSRX(7);				//Create a new instance for Latch Talon SRX CAN bus motor controller
    	
    	/*
    	 * Set slave motors to follow their masters
    	 */
    	leftSlaveMotor.follow(leftMotor);
    	rightSlaveMotor.follow(rightMotor);
    	winchSlaveMotor.follow(winchMotor);
    	
    	//resetButtons();
    	
    	
    	
		
		String autonChooser = SmartDashboard.getString("Auton Start Position/selected", "Nothing selected");
		
		/*switch (autonChooser) {
		case "Left Position":
			SmartDashboard.putNumber("Auton/Match Start Position", 0);
		case "Center Position":
			SmartDashboard.putNumber("Auton/Match Start Position", 1);
		case "Right Position":
			SmartDashboard.putNumber("Auton/Match Start Position", 2);
		case "Nothing selected":
			SmartDashboard.putNumber("Auton/Match Start Position", -3);
		default:
			SmartDashboard.putNumber("Auton/Match Start Position", -2);
		} */
		/***************************************************************************
		 * 
		 ***************************************************************************/
		if (autonChooser.equals("Left Position")) {
			SmartDashboard.putNumber("Auton/Match Start Position", 0);
		}
		if (autonChooser.equals("Center Position")) {
			SmartDashboard.putNumber("Auton/Match Start Position", 1);
		}
		if (autonChooser.equals("Right Position")) {
			SmartDashboard.putNumber("Auton/Match Start Position", 2);
		}
		
		matchStartPosition = SmartDashboard.getNumber("Auton/Match Start Position", -1);
		System.out.println(matchStartPosition);
		
		
		goForScale = SmartDashboard.getNumber("Auton/Go for Scale", -1);

		System.out.println(goForScale);
	}
	/**
	 * Switch Position: Left
	 * Scale Position: Left
	 */
	public static void runLL() {
		 autonInit();
		SmartDashboard.putString("Auton/Mode", "Left-Left");
		if (goForScale != 1.0) { // Go for switch
			if (matchStartPosition == 0) { 					// Robot Match Start Position: Left (from shuffleboard drop down)
				System.out.println("LEFT-LEFT-LEFT");
				long[] times = new long[] {
						0,								   // Start stage - Close arms to grab cube
						1000, 							   // Stage 1: Go forward until even with the switch
						2000, 							   // Stage 2: Rotate 90 degrees to the RIGHT
						2500,							   // Stage 3: Extend elevator upward SLIGHTLY
						3000,							   // Stage 4: Go forward A LITTLE
						3700							   // Stage 5: Eject cube onto switch	
				};
			} else if (matchStartPosition == 1) { // Robot Position: Center
				System.out.println("LEFT-LEFT-CENTER");
				long[] times = new long[] {
						0,								   // Start stage - Close arms to grab cube
						1000,							   // Stage 1: Go forward until IN FRONT OF switch
						2000, 							   // Stage 2: We have the left side of the switch, turn LEFT
						3000,							   // Stage 3: Go forward to edge of switch
						4000, 							   // Stage 4: Extend elevator up SLIGHTLY
						5000,							   // Stage 5: Rotate 90 degrees
						6000,							   // Stage 6: Go forward A REALLY TINY BIT // MAY NOT BE NECCESSARY
						6500, 							   // Stage 7: Drop cube onto switch
				};
			} else if (matchStartPosition == 2) { // Robot Position: Right
				System.out.println("LEFT-LEFT-RIGHT");
				long[] times = new long[] {
						0, 								   // Stage 0: Close arms to grab cube
						1000, 							   // Stage 1: Go forward until IN FRONT OF switch
						2000,							   // Stage 2: Turn LEFT
						3000, 							   // Stage 3: Go forward until even with end of switch
						4000, 							   // Stage 4: Extend elevator up SLIGHTLY
						5000,							   // Stage 5: Rotate 90 degrees
						6000,							   // Stage 6: Drop cube onto switch
				};
			}
		} else { // Go for scale.
			if (matchStartPosition == 0) { // Robot Position: Left
				long[] times = new long[] {
						0,								   // Start stage - Close arms to grab cube
						1000, 							   // Stage 1: Go forward until even with the scale
						2000, 							   // Stage 2: Rotate 90 degrees to the RIGHT
						2500,							   // Stage 3: Extend elevator upward ALL THE WAY
						3000,							   // Stage 4: Go forward A LITTLE
						3700							   // Stage 5: Eject cube onto scale	
				};
			} else if (matchStartPosition == 1) { // Robot Position: Center
				long[] times = new long[] {
						0,								   // Start stage - Close arms to grab cube
						1000,							   // Stage 1: Go forward until IN FRONT OF switch
						2000, 							   // Stage 2: We have the left side of the scale, turn LEFT
						3000,							   // Stage 3: Go forward past edge of switch
						4000, 							   // Stage 4: turn RIGHT
						5000, 							   // Stage 5: go forward to scale
						5000,							   // Stage 5: Rotate 90 degrees facing the scale
						6000,							   // Stage 6: Extend the elevator
						6500, 							   // Stage 7: Drop cube onto switch
				};
			} else if (matchStartPosition == 2) { // Robot Position: Right
				long[] times = new long[] {
						0, 								   // Stage 0: Close arms to grab cube
						1000, 							   // Stage 1: Go forward until IN FRONT OF switch
						2000,							   // Stage 2: Turn LEFT
						3000, 							   // Stage 3: Go forward until even with end of switch
						4000, 							   // Stage 4: Extend elevator up SLIGHTLY
						5000,							   // Stage 5: Rotate 90 degrees
						6000,							   // Stage 6: Drop cube onto switch
				};
			}
		}
		
	}
	
	public static void runRR() {
		autonInit();
		SmartDashboard.putString("Auton/Mode", "Right-Right");
		if (goForScale != 0) { // Go for switch
			if (matchStartPosition == 0) {

			} else if (matchStartPosition == 1) {
				
			} else if (matchStartPosition == 2) {
				
			}
		} else { // Go for scale
			if (matchStartPosition == 0) {

			} else if (matchStartPosition == 1) {
				
			} else if (matchStartPosition == 2) {
				
			}
		}
	}
	public static void runRL() {
		autonInit();
		SmartDashboard.putString("Auton/Mode", "Right-Left");
		if (matchStartPosition == 0) {

		} else if (matchStartPosition == 1) {
			
		} else if (matchStartPosition == 2) {
			
		}
	}
	public static  void runLR() {
		autonInit();
		SmartDashboard.putString("Auton/Mode", "Left-Right");
		if (matchStartPosition == 0) {

		} else if (matchStartPosition == 1) {
			
		} else if (matchStartPosition == 2) {
			
		}
	}
	
	
	
	
	public static void getButtonSelection() {
		boolean leftSelected = SmartDashboard.getBoolean("Auton/Left", false);
		boolean centerSelected = SmartDashboard.getBoolean("Auton/Center", false);
		boolean rightSelected = SmartDashboard.getBoolean("Auton/Right", false);
		if (leftSelected) {
			SmartDashboard.putBoolean("Auton/Center", false);
			SmartDashboard.putBoolean("Auton/Right", false);
			SmartDashboard.putNumber("Auton/Match Start Position", 0);
		} else
		if (centerSelected) {
			SmartDashboard.putBoolean("Auton/Left", false);
			SmartDashboard.putBoolean("Auton/Right", false);
			SmartDashboard.putNumber("Auton/Match Start Position", 1);
		} else
		if (rightSelected) {
			SmartDashboard.putBoolean("Auton/Center", false);
			SmartDashboard.putBoolean("Auton/Left", false);
			SmartDashboard.putNumber("Auton/Match Start Position", 2);
		}
		else {
			// No option selected
			SmartDashboard.putNumber("Auton/Match Start Position", -1);
		}
		matchStartPosition = SmartDashboard.getNumber("Auton/Match Start Position", -1);
	}
}
