package org.usfirst.frc.team6090.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick; 						//import Joystick method of WPIlib archive
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Victor; 						//import Victor (non-CAN bus) method of WPIlib archive

import com.ctre.phoenix.motorcontrol.ControlMode; 			//import CAN bus motor control mode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 		//import CAN bus Talon SRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; 	//import CAN bus Victor SRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice; 		//import CAN but Talon encoder FDBK
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.Solenoid; 						//import control for solenoids
import edu.wpi.first.wpilibj.Compressor; 					//import control for air compressor
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer; 					//import Camera method of WPIlib archive
import edu.wpi.first.wpilibj.PowerDistributionPanel; 		//import interface for Power Distribution Panel
import com.kauailabs.navx.frc.AHRS;

/*****************************************
 * Suppress warnings: - Unused variables and imports
 *****************************************/
@SuppressWarnings("unused")

/**********************************************************************************************************************
 * Start of main class Robot
 **********************************************************************************************************************/
public class Robot extends IterativeRobot {

	Joystick stick; 				// Reserve memory for variable
	Joystick xBox; 					// Reserve memory for variable
	Victor leftIntake; 				// Reserve memory for variable for Left Victor PWM controller
	Victor rightIntake; 			// Reserve memory for variable for Right Victor PWM controller

	/*****************************************
	 * CAN bus motors
	 *****************************************/
	WPI_TalonSRX leftMotor; 		// Reserve memory for variable for Left Talon CAN bus controller
	WPI_TalonSRX rightMotor; 		// Reserve memory for variable for Right Talon CAN bus controller
	WPI_TalonSRX winchMotor; 		// Reserve memory for variable for Winch Talon CAN bus controller
	WPI_VictorSPX leftSlaveMotor; 	// Reserve memory for variable for Left Victor CAN bus controller
	WPI_VictorSPX rightSlaveMotor; 	// Reserve memory for variable for Right Victor CAN bus controller
	WPI_VictorSPX winchSlaveMotor; 	// Reserve memory for variable for Winch Victor CAN bus controller

	PowerDistributionPanel ourPDP; 				// Reserve memory for variable for Power Distribution Panel
	double[] currentFdbkPDP = new double[16]; 	// Current feedback values from PDP

	double leftMotorVelocity = 0; 	// Left drive motor speed
	double leftMotorPosition = 0; 	// Left drive motor relative position
	double leftMotorPositionAut = 0; // Left drive motor relative position
	int leftKTimeoutMs = 10; 		// Reserve memory - 0=don't wait for confirmation, <>0 is wait
	double rightMotorVelocity = 0; 	// Right drive motor speed
	double rightMotorPosition = 0; 	// Right drive motor relative position
	int rightKTimeoutMs = 10; 		// Reserve memory - 0=don't wait for confirmation, <>0 is wait
	double latchMotorVelocity = 0; 	// Latch drive motor speed
	double latchMotorPosition = 0; 	// Latch drive motor relative position
	int latchKTimeoutMs = 10; 		// Reserve memory - 0=don't wait for confirmation, <>0 is wait
	double yAxisRef = 0; 			// F:-1.0 - R:1.0 Forward-to-Reverse motion

	int winchKTimeoutMs = 10; 		// Reserve memory - 0=don't wait for confirmation, <>0 is wait
	double winchFeedFwdFix = 0; 	// Feed forward gain messes with the final target position so adjust to get it back
	double winchManualOffset = 0; 	// Change fixed positions with manual adjustments from the left x-box joystick
	double winchMotorVelocity = 0; 	// Winch drive motor speed
	double winchMotorPosition = 0; 	// Winch drive motor relative position
	double winchMotorCurrent = 0; 	// Winch drive motor current
	double winchPositionErr = 0; 	// Winch position error
	double winchPresetPosition = 0; // Winch predefined position setpoints
	double winchTargetAbsolutePosition = 0; // Reserve memory - Absolute target position in encoder cnts (0=bottom)

	/*****************************************
	 * Gyro Data
	 *****************************************/
	AHRS navxGyro; 					// Navx Gryo module
	float gyroCompass;				// Gyro Compass heading as read from navx gyro
	float gyroPitch; 				// Gyro Pitch read from navx gyro
	float gyroRoll; 				// Gyro Roll read from navx gyro
	float gyroYaw; 					// Gyro Yaw read from navx gyro
	double gyroDriftTotal; 			// Total gyro drift while running
	final double gyroDriftIncrement = 0.0001688555; // Amout of drift correction per scan of gyro
	double gyroWithDrift = 0; 		// Gyro feedback value with drift added
	double firstGyroScan = 0; 		// Gyro angle on first scan of autonomous
	double gyroAngleOffset = 0; 	// Gyro angle with first scan angle added to it
	double gyroAngleFB; 			// Gyro Feedback scaled to 0-359 degrees
	double gyroAngleSP; 			// Gyro SetPoint scaled to 0-359 degrees
	double gyroAngleMod = 0.0; 		// gyro angle modified for drift
	double gyroAngleSync = 0.0; 	// New gyro angle sync reference
	int numRotations = 1; 			// number of whole rotations of feedback

	/*****************************************
	 * Solenoid valves
	 *****************************************/
	Compressor airCompressor; 		// Reserve memory for Compressor on CAN bus ID=60
	Solenoid leftMotorShiftHigh; 	// Reserve memory for variable for Left motor shift to High
	Solenoid leftMotorShiftLow; 	// Reserve memory for variable for Left motor shift to Low
	Solenoid winchMotorLock; 		// Reserve memory for variable for Winch motor shift to High
	Solenoid winchMotorRun; 		// Reserve memory for variable for Winch motor shift to Low
	Solenoid intakeArmClose; 		// Reserve memory for variable for Intake Arm Open
	Solenoid intakeArmOpen; 		// Reserve memory for variable for Intake Arm Close
	Solenoid intakeArmPivotDOWN; 	// Reserve memory for variable for Intake Arm Pivot DOWN
	Solenoid intakeArmPivotUP; 		// Reserve memory for variable for Intake Arm Pivot UP

	/*****************************************
	 * Robot wheel speed control by joystick
	 *****************************************/
	double xAxisMod = 0; 			// Modified x-Axis position command to include deadband
	double yAxisMod = 0; 			// Modified y-Axis position command to include deadband
	double zAxisMod = 0; 			// Modified z-Axis position command modified to include deadband
	double leftMotorSpdRef = 0; 	// X-Axis speed command
	double rightMotorSpdRef = 0; 	// Y-Axis speed command
	double leftMotorSpdRefAut = 0; 	// X-Axis speed command in Autonomous
	double rightMotorSpdRefAut = 0; // Y-Axis speed command in Autonomous
	double left_x_axisTurnMod = 0; 	// Turning Left xAxis mod
	double right_x_axisTurnMod = 0; // Turning Right xAxis mod
	double left_z_axisRotMod = 0; 	// Rotate Left zAxis mod
	double right_z_axisRotMod = 0; 	// Rotate Right zAxis mod
	final double deadBandPercentX = 0.1; 	// Dead band percent of X-Axis side-to-side from joystick
	final double deadBandPercentZ = 0.2; 	// Dead band percent of Z-Axis twist from joystick
	final double deadBandWinch = 0.2; 		// Dead band from x-box joystick for Winch manual speed control
	final double deadBandLatch = 0.2; 		// Dead band from x-box joystick for Latch manual speed control

	double winchMotorSpdRef = 0; 			// Winch motor manual mode speed reference
	double latchMotorSpdRef = 0; 			// Latch motor manual mode speed reference
	double cubePickUpMotorSpdRef = 0; 		// PWM motor speed reference for cube pick-up
	boolean intakeArmOpenCommand; 			// Latched command to move the intake arms Closed
	boolean intakeArmPivotDownCommand; 		// Latched command to move intake arm Pivot DOWN
	boolean winchRunCommand; 				// Winch motor gearbox engaged to run mode
	boolean enableClimbMode; 				// Climb mode started

	/*****************************************
	 * Digital Inputs
	 *****************************************/
	DigitalInput digitalIn0; 		// Reserve memory for Digital Input 0
	DigitalInput digitalIn1; 		// Reserve memory for Digital Input 1
	DigitalInput digitalIn2; 		// Reserve memory for Digital Input 2
	DigitalInput digitalIn3; 		// Reserve memory for Digital Input 3
	DigitalInput digitalIn4; 		// Reserve memory for Digital Input 4
	DigitalInput digitalIn5; 		// Reserve memory for Digital Input 5

	/*****************************************
	 * Position SETPOINTS
	 *****************************************/
	final double winchBottomRef 	= 400; 		// Winch Bottom position ref in encoder quad counts (old 300)
	final double winchScaleRef 		= 62000; 	// Winch Scale position ref in encoder quad counts (old 7500)
	final double winchScaleTopRef 	= 75000; 	// Winch Scale Top position ref in encoder quad counts (old 8400)
	final double winchSwitchRef 	= 29000; 	// Winch Switch position ref in encoder quad counts (old 3200)
	final double winchTopCubeRef 	= 26000; 	// Winch Pile Top Cube position ref in encoder quad counts (old 3200)
	final double winchManualModeInc = 500; 		// Winch manual mode increase per scan
	final double winchMaxHeight 	= 80000;	// Winch max travel distance

	boolean testStartLeftSwitchLeft = false;
	boolean testStartRightSwitchRight = false;

	/*****************************************
	 * XBox controller
	 *****************************************/
	boolean[] xBoxButtons = new boolean[10];	// Reserve memory for variable for XBox controller buttons
	double[] xBoxAnalog = new double[10]; 		// Reserve memory for variable for XBox controller analog pads
	boolean[] stickButtons = new boolean[20]; 	// Reserve memory for variable for Joystick controller buttons
	int xBoxPOV = 0; 							// Reserve memory for variable for XBox controller POV

	/*****************************************
	 * Time related variables
	 *****************************************/
	boolean autoFirstScan = false; 				// First scan OFF, On for rest of running
	boolean climbStartTime = false; 			// Snapshot time when climb mode started
	long currentSystemClock = 0; 				// System clock time read
	long autoSequenceTime = 0; 					// Autonomous mode sequence timer number
	long autoTimeStart = 0; 					// System clock time snapshot at start of autonomous
	long climbTimeStartSnapshot = 0; 			// System clock time snapshot at start of climb mode
	long climbTimeElapsed = 0; 					// Elapsed time to do climb
	boolean firstScan = false;
	long firstClockScan = 0;

	/*****************************************
	 * Camera setup variables
	 *****************************************/
	public UsbCamera cameraGear; 				// Reserve memory for gear camera video stream
	final int kVideoWidth = 160; 				// set dashboard default camera image width
	final int kVideoHeight = 120; 				// set dashboard default camera image height
	final int kVideoFramesPerSecond = 30; 		// set dashboard default camera frames per second(fps)
	private int videoWidth; 					// Store value of actual width of video stream
	private int videoHeight; 					// Store value of actual height of video stream
	private int videoFPS; 						// Store value of actual fps of video stream

	/*****************************************
	 * Read Game information
	 *****************************************/
	String gameData; 							// 3 letters from FMS; Our Switch side nearest us, Our Scale side, Enemy Switch side farthest away
	String ourGameData; 						// Strip off the first 2 characters from the FMS string
	boolean robotStartLeft = false; 			// Robot starts at left position
	boolean robotStartCenter = false; 			// Robot starts at center position
	boolean robotStartRight = false; 			// Robot starts at right position
	double matchStartPosition = -1; 			// Robot Start position: 0=Left, 1=Center, 2=Right
	boolean goForScale = false; 				// Team override: True = Force going for scale instead of switch
	boolean goOpposite = true; 					// Go for opposite sides (such as opposite switch or scale)
	boolean runDefaultAuton = false; 			// Run default Autonomous mode of just driving forward
	String autonomousSelected;					//String defining the starting autonomous mode
	
	

	/*****************************************
	 * Autonomous mode
	 *****************************************/
	int startLeftSwitchLeftSeq = 0; 				// Start on Left, Switch on Left Autonomous Sequence number
	long[] startLeftSwitchLeftTime = new long[20]; 	// Create array of time points when each sequence starts in autonomous
	int startLeftSwitchRightSeq = 0; 				// Start on Left, Switch on Right Autonomous Sequence number
	long[] startLeftSwitchRightTime = new long[20]; // Create array of time points when each sequence starts in autonomous
	int startLeftScaleLeftSeq = 0; 					// Start on Left, Scale on Left Autonomous Sequence number
	long[] startLeftScaleLeftTime = new long[20]; 	// Create array of time points when each sequence starts in autonomous
	int startLeftScaleRightSeq = 0; 				// Start on Left, Scale on Right Autonomous Sequence number
	long[] startLeftScaleRightTime = new long[20]; 	// Create array of time points when each sequence starts in autonomous

	int startRightSwitchLeftSeq = 0; 				// Start on Right, Switch on Right Autonomous Sequence number
	long[] startRightSwitchLeftTime = new long[20]; // Create array of time points when each sequence starts in autonomous
	int startRightSwitchRightSeq = 0; 				// Start on Right, Switch on Right Autonomous Sequence number
	long[] startRightSwitchRightTime = new long[20];// Create array of time points when each sequence starts in autonomous
	int startRightScaleLeftSeq = 0; 				// Start on Right, Switch on Right Autonomous Sequence number
	long[] startRightScaleLeftTime = new long[20]; 	// Create array of time points when each sequence starts inautonomous
	int startRightScaleRightSeq = 0; 				// Start on Right, Scale on Right Autonomous Sequence number
	long[] startRightScaleRightTime = new long[20]; // Create array of time points when each sequence starts in autonomous

	int startCenterSwitchLeftSeq = 0; 				// Start on Center, Switch on Left Autonomous Sequence number
	long[] startCenterSwitchLeftTime = new long[20];// Create array of time points when each sequence starts in autonomous
	int startCenterSwitchRightSeq = 0; 				// Start on Center, Switch on Right Autonomous Sequence number
	long[] startCenterSwitchRightTime = new long[20]; // Create array of time points when each sequence starts in autonomous

	int startAnyForwardSeq = 0; 					// Start on Any, Switch on Left Autonomous Sequence number
	long[] startAnyForwardTime = new long[20]; 		// Create array of time points when each sequence starts in autonomous
	double autonMovementStartPosition = 0; 			// Snapshot current position to get starting point for calibrated movement
	
	int startCenterDefaultRightSeq = 0; 				// Start on Center, Switch on Default Right Autonomous Sequence number
	long[] startCenterDefaultRightTime = new long[20];// Create array of time points when each sequence starts in autonomous


	public void robotInit() 
		{
			stick = new Joystick(0); 				// Create a new instance of Joystick called stick
			xBox = new Joystick(1); 				// Create a new instance of Joystick called xBox
			leftIntake = new Victor(0); 			// Create a new instance for Left Victor PWM motor controller
			rightIntake = new Victor(1); 			// Create a new instance for Right Victor PWM motor controller
	
			leftMotor = new WPI_TalonSRX(1); 		// Create a new instance for Left Talon SRX CAN bus motor controller
			leftSlaveMotor = new WPI_VictorSPX(2); 	// Create a new instance for Left Victor SPX CAN bus motor controller
			rightMotor = new WPI_TalonSRX(3); 		// Create a new instance for Right Talon SRX CAN bus motor controller
			rightSlaveMotor = new WPI_VictorSPX(4); // Create a new instance for Right Victor SPX CAN bus motor controller
			winchMotor = new WPI_TalonSRX(5); 		// Create a new instance for Winch Talon SRX CAN bus motor controller
			winchSlaveMotor = new WPI_VictorSPX(6); // Create a new instance for Left Victor SPX CAN bus motor controller
			ourPDP = new PowerDistributionPanel(59);// Create a new instance for reading PDP values from CAN ID=59
			airCompressor = new Compressor(60); 	// Create a new instance for Compressor on CAN bus ID=60
			leftMotorShiftHigh = new Solenoid(60, 0);// Create a new instance for Left motor High Speed Sol shifter on Can ID=60
			leftMotorShiftLow = new Solenoid(60, 1); // Create a new instance for Left motor Low Speed Sol shifter on Can ID=60
			intakeArmClose = new Solenoid(60, 2); 	// Create a new instance for Intake Arms Open Sol on CAN ID=60
			intakeArmOpen = new Solenoid(60, 3); 	// Create a new instance for Intake Arms Close Sol on CAN ID=60
			intakeArmPivotDOWN = new Solenoid(60, 4);// Create a new instance for Intake Arms Pivot DOWN Sol on CAN ID=60
			intakeArmPivotUP = new Solenoid(60, 5); // Create a new instance for Intake Arms Pivot UP Sol on CAN ID=60
	
			leftMotor.setSensorPhase(true); 											// Make sure talons blink green when position increasing
			leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); 	// Left motor has a Quadrature encoder hooked up to talon
			leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 					// Zero the position encoder
	
			rightMotor.setSensorPhase(true); 											// Make sure talons blink green when position increasing
			rightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); 	// Right motor has a Quadrature encoder hooked up to talon
			rightMotor.setSelectedSensorPosition(0, 0, rightKTimeoutMs); 				// Zero the position encoder
	
			/**********************************************************************************************
			 * Setup the closed loop parameters for the winch Talon. For the
			 * SelectProfileSlot: First parameter is setpoint slot (0-4) Second parameter is
			 * PID loop number - usually 0 for primary loop
			 * 
			 * kFeed forward=(Percent motor load to achieve max speed) * 1023/max spd in
			 * encoder cnts/100ms To set-run full speed in manual joystick mode and read
			 * talon speed feedback and output percent from the Self-Test web interface
			 * roborio-6090-frc.local
			 **********************************************************************************************/
			/***************************************************************************
			 * Slot 0 is wench coarse movement UP and DOWN during Teleop
			 ***************************************************************************/
			winchMotor.selectProfileSlot(0, 0); 						// Slot 0, PID loop number=0
	
			winchMotor.config_kF(0, .40, winchKTimeoutMs); 				// Feed Forward gain - PID loop num, value, timout
			winchMotor.config_kP(0, 0.3, winchKTimeoutMs); 				// Proportional gain - PID loop num, value, timout
			winchMotor.config_kI(0, 0.0001, winchKTimeoutMs); 			// Integral gain - PID loop num, value, timout
			winchMotor.config_kD(0, 0, winchKTimeoutMs); 				// Differential gain - PID loop num, value, timout
			winchMotor.configAllowableClosedloopError(0, 50, winchKTimeoutMs); // This doesn't appear to do anything useful
			winchMotor.config_IntegralZone(0, 5, winchKTimeoutMs);		// Enable integral component when error gets below this threshold
			winchMotor.configPeakCurrentDuration(50, winchKTimeoutMs); 	// Number of msec to allow peak current
			winchMotor.configContinuousCurrentLimit(80, winchKTimeoutMs); // Continuous allow current in Amps
			winchMotor.configPeakCurrentLimit(68, winchKTimeoutMs); 	// Peak current in Amps
			winchMotor.configNeutralDeadband(0.03, winchKTimeoutMs); 	// If error is below threshold, consider to be "in position" (0.25=25%)
	
			/***************************************************************************
			 * Set accel and cruise velocity to use to make trapezoid profile Cruise speed =
			 * 95% of max speed = 3800*0.95 = 3610 Acc rate = Cruise spd in 0.5 sec =
			 * 3610/0.5 = 7220
			 ***************************************************************************/
			winchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); // Winch motor has a magnetic pickup encoder hooked to talon
			winchMotor.setSensorPhase(true); 							// Make sure talons blink green when position increasing
			winchMotor.setInverted(true); 								// Invert the speed reference
			winchSlaveMotor.setInverted(true); 							// Invert the speed reference (Slave must have same boolean value as master)
	
			/* Set relevant frame periods to be at least as fast as periodic rate */
			winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, winchKTimeoutMs);
			winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, winchKTimeoutMs);
			winchMotor.configMotionCruiseVelocity(3800, winchKTimeoutMs); 	// Cruize velociy in encoder/100msec
			winchMotor.configMotionAcceleration(5500, winchKTimeoutMs); 	// Accel
			winchMotor.setSelectedSensorPosition(0, 0, winchKTimeoutMs); 	// Zero the position encoder
	
			/*************************************************************************
			 * Set Open and Close Loop ramp rate -- Higher values give slower ramp--
			 * 
			 * Rate is change in output / 10msec chunks Max Talon speed reference = 1023
			 * counts internally so speed change = end speed - start speed
			 *************************************************************************/
			winchMotor.configOpenloopRamp(0.5, winchKTimeoutMs); 			// Rate = [1023-0] / [1000 msec] * 10 msec = 10
			winchMotor.configClosedloopRamp(0.5, winchKTimeoutMs); 			// Rate = [1023-0] / [500 msec] * 10 msec = 20
			winchManualOffset = 0; 											// Reset manual ref to zero on startup
	
			/*****************************************
			 * NavX MXP am-3060b plugged in roborio MXP port declaration. Plug in with
			 * micro-USB to verify operation and update firmware: hold cal button then plug
			 * in USB to update.
			 *****************************************/
			navxGyro = new AHRS(SPI.Port.kMXP);
	
			digitalIn0 = new DigitalInput(0); 		// Assign digital input to variable
			digitalIn1 = new DigitalInput(1); 		// Assign digital input to variable
			digitalIn2 = new DigitalInput(2); 		// Assign digital input to variable
			digitalIn3 = new DigitalInput(3); 		// Assign digital input to variable
			digitalIn4 = new DigitalInput(4); 		// Assign digital input to variable
			digitalIn5 = new DigitalInput(5); 		// Assign digital input to variable
	
			cameraGear = CameraServer.getInstance().startAutomaticCapture("lift Camera(0)", 0);
			cameraGear.setFPS(kVideoFramesPerSecond); 				// set Frames per second
			cameraGear.setResolution(kVideoWidth, kVideoHeight); 	// Set details camera resolution
			videoHeight = cameraGear.getVideoMode().height; 		// Get values of video height. Needed for vision calculations
			videoWidth = cameraGear.getVideoMode().width; 			// Get values of video width. Needed for vision calculations
			videoFPS = cameraGear.getVideoMode().fps;
	
			/**************************************************************
			 * Read selector switch to tell where the robot starts from in autonomous All
			 * inputs are normally ON, selecting it will make it go OFF
			 **************************************************************/
			if (!digitalIn0.get())
				matchStartPosition = 0; 	// 0 = Robot Start on LEFT
			if (!digitalIn1.get())
				matchStartPosition = 1; 	// 1 = Robot Start on CENTER
			if (!digitalIn2.get())
				matchStartPosition = 2; 	// 2 = Robot Start on RIGHT
			if (digitalIn0.get() && digitalIn1.get() && digitalIn2.get())
				matchStartPosition = 3; 	// 3 = Robot just drives forward for auton
	
			/**
			 * This block will add a network table value for the match start position.
			 */
			if (matchStartPosition == 0) {
				SmartDashboard.putString("Match Start Position Description", "Left");
			} else if (matchStartPosition == 1) {
				SmartDashboard.putString("Match Start Position Description", "Center");
			} else if (matchStartPosition == 2) {
				SmartDashboard.putString("Match Start Position Description", "Right");
			} else {
				SmartDashboard.putString("Match Start Position Description", "Unknown");
			}
	
			SmartDashboard.putBoolean("GFS", true);
			goForScale = SmartDashboard.getBoolean("GFS", true);
	
			System.out.println("go for scale=" + goForScale);
	
			SmartDashboard.putBoolean("GoOpposite", true);
			
			updateNetworkTables();
	
		} // End of RobotInit

	public void autonomousInit() 
		{
			updateNetworkTables();
			/**********************************************************************************************************************
			 * Variable Declarations for Autonomous Initialization
			 **********************************************************************************************************************/
	
			winchMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 		// Zero the position encoder
			autoFirstScan = false; 												// Reset first scan latch bit
			runDefaultAuton = false; 											// Reset Run default Autonomous mode of just driving forward
	
			/*****************************************
			 * The FMS gives a 3 letter setup message to indicate the position of your
			 * alliance colored Our Switch side nearest us, Our Scale side, Enemy Switch
			 * side farthest away
			 * 
			 * LRL for example: (near switch, scale, far switch) (1st L) Our alliance switch
			 * nearest us is on the Left (2nd R) Our alliance scale is on the right (3rd L)
			 * Our alliance switch color farthest us is on the Left
			 *****************************************/
			
			try 
				{
					gameData = DriverStation.getInstance().getGameSpecificMessage();  // Read from the driver station panel from the Field Management System
					ourGameData = gameData.substring(0, 2); // Strip off the first 2 characters from the FMS string
				} 
			catch (Throwable e) 
				{
					// Well, FMS screwed up, we have no data.
					gameData = "nul";
					ourGameData = "nul";
				}
			
			if (ourGameData.equals("nul") || gameData.equals("nul")) 
				{
					autonomousSelected = "default";
				}
			
			System.out.println( "Extracted Game Data: " + ourGameData + "\nAutonomous Selected: " + autonomousSelected);
	
			/*****************************************
			 * Turn on the air compressor
			 *****************************************/
			airCompressor.setClosedLoopControl(true); 							// Enable Air Compressor to be controlled by pressure switch input
	
			/**********************************************************************************************************************
			 * Action Code for Autonomous Initialization
			 **********************************************************************************************************************/
			navxGyro.reset();
			leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 			// Zero the position encoder
			rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 		// Zero the position encoder
			winchMotor.setSelectedSensorPosition(0, 0, winchKTimeoutMs); 		// Zero the position encoder
			
			startLeftSwitchLeftSeq = 0; 				// Zero Start on Left, Switch on Left sequence number
			startLeftSwitchRightSeq = 0; 				// Zero Start on Left, Switch on Right sequence number
			startLeftScaleLeftSeq = 0; 					// Zero Start on Left, Scale on Left sequence number
			startLeftScaleRightSeq = 0; 				// Zero Start on Left, Scale on Right sequence number
			
			startRightSwitchRightSeq = 0; 				// Zero Start on Right, Switch on Right sequence number
			startRightSwitchLeftSeq = 0; 				// Zero Start on Right, Switch on Right sequence number
			startRightScaleRightSeq = 0; 				// Zero Start on Right, Scale on Right sequence number
			startRightScaleLeftSeq = 0; 				// Zero Start on Right, Scale on Right sequence number
			
			startCenterSwitchLeftSeq = 0; 				// Zero Start on Center, Scale on Left sequence number
			startCenterSwitchRightSeq = 0; 				// Zero Start on Center, Scale on Right sequence number
			startAnyForwardSeq = 0; 					// Zero Start on Any, Just go forward sequence number
	
			//autonomousSelected = "   ";
			
			goForScale = SmartDashboard.getBoolean("GFS", true);
			System.out.println("[AUTON] Go for scale: " + goForScale);
			
			goOpposite = SmartDashboard.getBoolean("GoOpposite", true);
			System.out.println("[AUTON] Go Opposite: " + goOpposite);
	
			/**************************************************************
			 * Read selector switch to tell where the robot starts from in autonomous All
			 * inputs are normally ON, selecting it will make it go OFF
			 **************************************************************/
			if (!digitalIn0.get())
				matchStartPosition = 0; 					// 0 = Robot Start on LEFT
			if (!digitalIn1.get())
				matchStartPosition = 1; 					// 1 = Robot Start on CENTER
			if (!digitalIn2.get())
				matchStartPosition = 2; 					// 2 = Robot Start on RIGHT
			if (digitalIn0.get() && digitalIn1.get() && digitalIn2.get())
				matchStartPosition = 3; 					// 3 = Robot just drives forward for auton
	
			System.out.println("Match Pos = " + matchStartPosition);
			
			
			if (matchStartPosition == 0)	// Robot start position is left (left bumper is aligned with left point of straight part of back wall
				{
					if (goForScale)																		//goForScale=TRUE for LEFT Side
						{
							if (goOpposite)																//goOpposite=TRUE, goForScale=TRUE for LEFT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("RL") )
										autonomousSelected = "startLeftScaleLeft";
										
									else if ( ourGameData.equals("LR") || ourGameData.equals("RR") )
										autonomousSelected = "startLeftScaleRight";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=TRUE, goOpposite=TRUE for LEFT Side
							else																		//goForScale=TRUE, goOpposite=FALSE for LEFT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("RL") )
										autonomousSelected = "startLeftScaleLeft";
										
									else if ( ourGameData.equals("LR") )
										autonomousSelected = "startLeftSwitchLeft";
										
									else if ( ourGameData.equals("RR") )
										autonomousSelected = "default";	
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=TRUE, goOpposite=FALSE for LEFT Side
		
						}	//End of goForScale=TRUE for LEFT Side
					else																				//goForScale=FALSE for LEFT Side
						{
							if (goOpposite)																//goForScale=FALSE, goOpposite=TRUE for LEFT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("LR") )
										autonomousSelected = "startLeftSwitchLeft";
										
									else if ( ourGameData.equals("RL") || ourGameData.equals("RR") )
										autonomousSelected = "startLeftSwitchRight";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=FALSE, goOpposite=TRUE for LEFT Side
							else																		//goForScale=FALSE, goOpposite=FALSE for LEFT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("LR") )
										autonomousSelected = "startLeftSwitchLeft";
										
									else if ( ourGameData.equals("RL") )
										autonomousSelected = "startLeftScaleLeft";
										
									else if ( ourGameData.equals("RR") )
										autonomousSelected = "default";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=FALSE, goOpposite=FALSE for LEFT Side
						}	//End of goForScale=TRUE for LEFT Side
				}	//End of LEFT Side
			
	
			else if (matchStartPosition == 1) // Robot start position is center (left bumper is aligned with right wall of exchange zone box of back wall
				{
					if ( ourGameData.equals("LL") || ourGameData.equals("LR") )
						autonomousSelected = "startCenterSwitchLeft";
										
					else if ( ourGameData.equals("RL") || ourGameData.equals("RR") )
						autonomousSelected = "startCenterSwitchRight";
										
					else
						autonomousSelected = "default";
				}
			
	
			else if (matchStartPosition == 2) // Robot start position is right (right bumper is aligned with right point of straight part of back wall
				{
					if (goForScale)																													//goForScale=TRUE for RIGHT Side
						{
							if (goOpposite)																											//goOpposite=TRUE, goForScale=TRUE for RIGHT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("RL") )
										autonomousSelected = "startRightScaleLeft";
										
									else if ( ourGameData.equals("LR") || ourGameData.equals("RR") )
										autonomousSelected = "startRightScaleRight";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=TRUE, goOpposite=TRUE for RIGHT Side
							else																																//goForScale=TRUE, goOpposite=FALSE for RIGHT Side
								{
									if ( ourGameData.equals("LR") || ourGameData.equals("RR") )
										autonomousSelected = "startRightScaleRight";
										
									else if ( ourGameData.equals("RL") )
										autonomousSelected = "startRightSwitchRight";
										
									else if ( ourGameData.equals("LL") )
										autonomousSelected = "default";	
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=TRUE, goOpposite=FALSE for RIGHT Side
		
						}	//End of goForScale=TRUE for RIGHT Side
					else																																	//goForScale=FALSE for RIGHT Side
						{
							if (goOpposite)																										//goForScale=FALSE, goOpposite=TRUE for RIGHT Side
								{
									if ( ourGameData.equals("LL") || ourGameData.equals("LR") )
										autonomousSelected = "startRightSwitchLeft";
										
									else if ( ourGameData.equals("RL") || ourGameData.equals("RR") )
										autonomousSelected = "startRightSwitchRight";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=FALSE, goOpposite=TRUE for RIGHT Side
							else																														//goForScale=FALSE, goOpposite=FALSE for RIGHT Side
								{
									if ( ourGameData.equals("RL") || ourGameData.equals("RR") )
										autonomousSelected = "startRightSwitchRight";
										
									else if ( ourGameData.equals("LR") )
										autonomousSelected = "startRightScaleRight";
										
									else if ( ourGameData.equals("LL") )
										autonomousSelected = "default";
										
									else
										autonomousSelected = "default";
								
								}	//End of goForScale=FALSE, goOpposite=FALSE for RIGHT Side
						}	//End of goForScale=TRUE for RIGHT Side
				}	//End of RIGHT Side
			else
				{
					autonomousSelected = "default";
					runDefaultAuton = true; // Run default Autonomous mode of just driving forward
					System.out.println("Run Default Code");
				}
			

		System.out.println("autonomousSelected = " + autonomousSelected);
	
			// matchStartPosition = 2; //TEST MODE ONLY 0=left, 1=center, 2=right
			// goForScale = true; //TEST MODE ONLY
	
		} // End of Autonomous Initialization

	public void autonomousPeriodic() // Action routine: code called periodically during Autonomous
		{
	
			/**********************************************************************************************************************
			 * Variable Declarations for Autonomous Periodic
			 **********************************************************************************************************************/
	
			long[] startLeftSwitchLeft = new long[30]; 		// Create array of time points for autonomous mode for:start Left side, go for switch on Left side
			double startLeftSwitchLeftFirstPosition = 0; 	// First movement position point for:start Left side, go for switch on Left side
			double startLeftSwitchLeftFirstAngle = 0; 		// First movement angle for:start Left side, go for switch on Left side
			double startLeftSwitchLeftSecondPosition = 0; 	// Second movement position point for:start Right side, go for switch on Left side
			double startLeftSwitchLeftSecondAngle = 0; 		// Second movement angle for:start Left side, go for switch on Left side
			double startLeftSwitchLeftIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startLeftSwitchLeftThirdPosition = 0; 	// Third movement position point for:start Left side, go for switch on Left side
			double startLeftSwitchLeftThirdAngle = 0; 		// Third movement angle for:start Left side, go for switch on Left side
	
			long[] startLeftScaleLeft = new long[30]; 				// Create array of time points for autonomous mode for:start Left side, go for Scale on Left side
			double[] startLeftScaleLeftPosition = new double[30];	//Autonomous mode position points
			double[] startLeftScaleLeftAngle = new double[30];		//Autonomous mode angle points
			
			double startLeftScaleLeftFirstPosition = 0; 	// First movement position point for:start Left side, go for Scale on Left side
			double startLeftScaleLeftFirstAngle = 0; 		// First movement angle for:start Left side, go for Scale on Left side
			double startLeftScaleLeftSecondPosition = 0; 	// Second movement position point for:start Right side, go for Scale on Right side
			double startLeftScaleLeftSecondAngle = 0; 		// Second movement angle for:start Left side, go for Scale on Left side
			double startLeftScaleLeftIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startLeftScaleLeftIntakeGrabRef = 0.0; 	// Intake speed ref to grab cube in
			double startLeftSwitchLeftIntakeHoldRef = 0.0; 	// To make sure we hold the cube.
			double startLeftScaleLeftThirdPosition = 0; 	// Third movement position point for:start Left side, go for Scale on Left side
			double startLeftScaleLeftThirdAngle = 0; 		// Third movement angle for:start Left side, go for Scale on Left side
			double startLeftScaleLeftFourthAngle = 0; 		// Third movement angle for:start Left side, go for Scale on Left side
			double startLeftScaleLeftFourthPosition = 0; 	// Third movement angle for:start Left side, go for Scale on Left side
	
			long[] startLeftSwitchRight = new long[30]; 	// Create array of time points for autonomous mode for:start Left side, go for switch on Right side
			double startLeftSwitchRightFirstPosition = 0; 	// First movement position point for:start Left side, go for switch on Right side
			double startLeftSwitchRightFirstAngle = 0; 		// First movement angle for:start Left side, go for switch on Right side
			double startLeftSwitchRightSecondPosition = 0; 	// Second movement position point for:start Right side, go for switch on Right side
			double startLeftSwitchRightSecondAngle = 0; 	// Second movement angle for:start Left side, go for switch on Right side
			double startLeftSwitchRightIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startLeftSwitchRightThirdPosition = 0; 	// Third movement position point for:start Left side, go for switch on Right side
			double startLeftSwitchRightThirdAngle = 0; 		// Third movement angle for:start Left side, go for switch on Right side
			double startLeftSwitchRightForthPosition = 0; 	// Fourth movement position point for:start Left side, go for switch on Right side
	
			long[] startLeftScaleRight = new long[30]; 		// Create array of time points for autonomous mode for:start Left side, go for scale on Right side
			double startLeftScaleRightFirstPosition = 0; 	// First movement position point for:start Left side, go for scale on Right side
			double startLeftScaleRightFirstAngle = 0; 		// First movement angle for:start Left side, go for scale on Right side
			double startLeftScaleRightSecondPosition = 0; 	// Second movement position point for:start Right side, go for scale on Right side
			double startLeftScaleRightSecondAngle = 0; 		// Second movement angle for:start Left side, go for scale on Right side
			double startLeftScaleRightIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startLeftScaleRightThirdPosition = 0; 	// Third movement position point for:start Left side, go for scale on Right side
			double startLeftScaleRightThirdAngle = 0; 		// Third movement angle for:start Left side, go for scale on Right side
			double startLeftScaleRightForthPosition = 0; 	// Fourth movement position point for:start Left side, go for scale on Right side
			double startLeftScaleRightFourthAngle = 0; 		// Fourth movement angle for:start Left side, go for scale on Right side
	
			long[] startRightSwitchRight = new long[30]; 	// Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startRightSwitchRightFirstPosition = 0; 	// First movement position point for:start Right side, go for switch on Right side
			double startRightSwitchRightFirstAngle = 0; 	// First movement angle for:start Right side, go for switch on Right side
			double startRightSwitchRightSecondPosition = 0; // Second movement position point for:start Right side, go for switch on Right side
			double startRightSwitchRightSecondAngle = 0; 	// Second movement angle for:start Right side, go for switch on Right side
			double startRightSwitchRightIntakeSpdRef = 0.0; // Intake speed ref to spit out cube
			double startRightSwitchRightThirdPosition = 0; 	// Third movement position point for:start Right side, go for switch on Right side
			double startRightSwitchRightThirdAngle = 0; 	// Third movement angle for:start Right side, go for switch on Right side
	
			long[] startRightSwitchLeft = new long[30]; 	// Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startRightSwitchLeftFirstPosition = 0; 	// First movement position point for:start Right side, go for switch on Right side
			double startRightSwitchLeftFirstAngle = 0; 		// First movement angle for:start Right side, go for switch on Right side
			double startRightSwitchLeftSecondPosition = 0; 	// Second movement position point for:start Right side, go for switch on Right side
			double startRightSwitchLeftSecondAngle = 0; 	// Second movement angle for:start Right side, go for switch on Right side
			double startRightSwitchLeftIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startRightSwitchLeftThirdPosition = 0; 	// Third movement position point for:start Right side, go for switch on Right side
			double startRightSwitchLeftThirdAngle = 0; 		// Third movement angle for:start Right side, go for switch on Right side
			double startRightSwitchLeftForthPosition = 0; 	// Fourth movement position point for:start Right side, go for switch on Left side
	
			long[] startRightScaleRight = new long[30]; 				// Create array of time points for autonomous mode for:start Right side, go for scale on Right side
			double[] startRightScaleRightPosition = new double[30];		//Autonomous mode position points
			double[] startRightScaleRightAngle = new double[30];		//Autonomous mode angle points
			
			double startRightScaleRightFirstPosition = 0; 	// First movement position point for:start Right side, go for scale on Right side
			double startRightScaleRightFirstAngle = 0; 		// First movement angle for:start Right side, go for scale on Right side
			double startRightScaleRightSecondPosition = 0; 	// Second movement position point for:start Right side, go for switch on Right side
			double startRightScaleRightSecondAngle = 0; 	// Second movement angle for:start Right side, go for scale on Right side
			double startRightScaleRightIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startRightScaleRightIntakeGrabRef = 0.0; // Intake speed ref to grab cube in
			double startRightScaleRightThirdPosition = 0; 	// Third movement position point for:start Right side, go for scale on Right side
			double startRightScaleRightThirdAngle = 0; 		// Third movement angle for:start Right side, go for scale on Right side
			double startRightScaleRightFourthPosition = 0; 	// Fourth movement position point for:start Right side, go for scale on Right side
			double startRightScaleRightFourthAngle = 0; 	// Fourth movement angle for:start Right side, go for scale on Right side
	
			long[] startRightScaleLeft = new long[30]; 				// Create array of time points for autonomous mode for:start Right side, go for scale on Left side
			double[] startRightScaleLeftPosition = new double[30];	//Autonomous mode position points
			double[] startRightScaleLeftAngle = new double[30];		//Autonomous mode angle points
			
			double startRightScaleLeftFirstPosition = 0; 	// First movement position point for:start Right side, go for scale on Left side
			double startRightScaleLeftFirstAngle = 0; 		// First movement angle for:start Right side, go for scale on Left side
			double startRightScaleLeftSecondPosition = 0; 	// Second movement position point for:start Right side, go for scale on Left side
			double startRightScaleLeftSecondAngle = 0; 		// Second movement angle for:start Right side, go for scale on Left side
			double startRightScaleLeftIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startRightScaleLeftThirdPosition = 0; 	// Third movement position point for:start Right side, go for scale on Left side
			double startRightScaleLeftThirdAngle = 0; 		// Third movement angle for:start Right side, go for scale on Left side
			double startRightScaleLeftForthPosition = 0; 	// Fourth movement position point for:start Right side, go for scale on Left side
			double startRightScaleLeftFourthAngle = 0; 		// Fourth movement angle for:start Right side, go for scale on Left side
	
			long[] startCenterSwitchLeft = new long[30]; 	// Create array of time points for autonomous mode for:start Left side, go for switch on Left side
			double startCenterSwitchLeftFirstPosition = 0; 	// First movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftFirstAngle = 0; 	// First movement angle for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSecondPosition = 0; // Second movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSecondAngle = 0; 	// Second movement angle for:start Center side, go for switch on Left side
			double startCenterSwitchLeftIntakeSpdRef = 0.0; // Intake speed ref to spit out cube
			double startCenterSwitchLeftIntakeGrabRef = 0.0; // Intake speed ref to grab cube in
			double startCenterSwitchLeftThirdPosition = 0; 	// Third movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftThirdAngle = 0; 	// Third movement angle for:start Center side, go for switch on Left side
			double startCenterSwitchLeftForthPosition = 0; 	// Fourth movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftForthAngle = 0; 	// Fourth movement angle for:start Center side, go for switch on Left side
			double startCenterSwitchLeftFifthPosition = 0; 	// Fifth movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftFifthAngle = 0; 	// Fifth movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchLeftSixthPosition = 0; 	// Sixth movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSixthAngle = 0;		// Sixth movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchLeftSeventhPosition = 0; // Seventh movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSeventhAngle = 0; 	// Seventh movement angle for:start Center side, go for switch on Left side
			double startCenterSwitchLeftEighthAngle = 0; 	// Gyro angle for eighth turn to move toward switch
			double startCenterSwitchLeftEighthPosition = 0;	// Seventh movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchLeftNinthAngle = 0; 	// Gyro angle for ninth turn to move toward switch
			double startCenterSwitchLeftNinthPosition = 0;	// Ninth movement position point for:start Center side, go for switch on Left side
	
			long[] startCenterSwitchRight = new long[30]; 		// Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startCenterSwitchRightFirstPosition = 0; 	// First movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightFirstAngle = 0; 		// First movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightSecondPosition = 0; 	// Second movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightSecondAngle = 0; 		// Second movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightIntakeSpdRef = 0.0; 	// Intake speed ref to spit out cube
			double startCenterSwitchRightIntakeGrabRef = 0.0; 	// Intake speed ref to grab cube in
			double startCenterSwitchRightThirdPosition = 0; 	// Third movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightThirdAngle = 0; 		// Third movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightForthPosition = 0; 	// Forth movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightForthAngle = 0; 		// Forth movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightFifthPosition = 0; 	// Fifth movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightFifthAngle = 0; 		// Fifth movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightSixthPosition = 0; 	// Sixth movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightSixthAngle = 0; 		// Sixth movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightSeventhPosition = 0;	// Seventh movement position point for:start Center side, go for switch on Right side
			double startCenterSwitchRightSeventhAngle = 0; 		// Seventh movement angle for:start Center side, go for switch on Right side
			double startCenterSwitchRightEighthAngle = 0; 		// Gyro angle for eighth turn to move toward switch
			double startCenterSwitchRightEighthPosition = 0;	// Eigth movement position point for:start Center side, go for switch on Left side
			double startCenterSwitchRightNinthAngle = 0; 		// Gyro angle for eighth turn to move toward switch
			double startCenterSwitchRightNinthPosition = 0;		// Eigth movement position point for:start Center side, go for switch on Left side
			
			long[] startAnyForward = new long[30]; 				// Create array of time points for autonomous mode for:start any side, go forward
			double startAnyForwardFirstPosition = 0; 			// First movement position point for:start Any side, go forward
			double startAnyForwardFirstAngle = 0; 				// First movement angle for:start Any side, go forward
			double startAnyForwardSecondPosition = 0; 			// Second movement position point for:start Any side, go forward
			double startAnyForwardSecondAngle = 0; 				// Second movement angle for:start Any side, go forward
			double startAnyForwardIntakeSpdRef = 0.0; 			// Intake speed ref to spit out cube
			double startAnyForwardThirdPosition = 0; 			// Third movement position point for:start Any side, go forward
			double startAnyForwardThirdAngle = 0; 				// Third movement angle for:start Any side, go forward
			double startAnyForwardForthPosition = 0; 			// Forth movement position point for:start Any side, go forward
			double startAnyForwardForthAngle = 0; 				// Forth movement angle for:start Any side, go forward
			
			long[] startCenterDefaultRight = new long[30]; 		// Create array of time points for autonomous mode for:start Right side, go for default
			double startCenterDefaultRightFirstPosition = 0; 	// First movement position point for:start Center side, go for switch on default
			double startCenterDefaultRightFirstAngle = 0; 		// First movement angle for:start Center side, go for default
			double startCenterDefaultRightSecondPosition = 0; 	// Second movement position point for:start Center side, go for default
			double startCenterDefaultRightSecondAngle = 0; 		// Second movement angle for:start Center side, go for default
			double startCenterDefaultRightThirdPosition = 0; 	// Third movement position point for:start Center side, go for default

			
			/**********************************************************************************************************************
			 * Action Code for Autonomous Periodic
			 **********************************************************************************************************************/
	
			/**************************************************************
			 * Snapshot system clock value on first scan of robot running Then: Current
			 * system clock - Start system clock will give autonomous clock starting at zero
			 **************************************************************/
			if (!autoFirstScan) 
				{
					autoTimeStart = System.currentTimeMillis(); 		// Snapshot system time at start of autonomous
					gyroYaw = navxGyro.getYaw(); 						// Read gyro yaw value from navx board
					if (gyroYaw < 0) 									// Left half of compass
						firstGyroScan = 180 + (180 + gyroYaw); 			// gyro raw -179 to 0 = gyro compass angle +181 to 359
					else 												// Right half of compass
						firstGyroScan = gyroWithDrift; 					// gyro raw 0 to 179 = gyro compass angle 0 to 179
					autoFirstScan = true; 								// turn on first scan flag
				}
			currentSystemClock = System.currentTimeMillis(); 			// Read System clock
			autoSequenceTime = currentSystemClock - autoTimeStart; 		// Calc system time elapsed
			// System.out.println("clock=" + autoSequenceTime);
			// System.out.println("Our data auton= " + ourGameData);
	
			/****************************************************************************************************
			 ** Read raw gyro angle from the input mod. Compensate for gyro drift by adding
			 * an offset value to Compensate for gyro drift by adding an offset value to
			 * summation gyroDriftTotal Scale +/- gyro values and compensate for rollover so
			 * scalled to 0-360 degrees and
			 ****************************************************************************************************/
			gyroPitch = navxGyro.getPitch(); 							// Get gyro pitch because roboRIO is mounted on side
			gyroYaw = navxGyro.getYaw(); 								// Read gyro yaw value from navx board
			gyroDriftTotal = gyroDriftTotal + gyroDriftIncrement; 		// Keep track of total gyro angle drift
			gyroWithDrift = gyroYaw + gyroDriftTotal; 					// Real angle is read gyro angle plus drift compensation
	
			if (gyroWithDrift < 0)
				gyroAngleMod = 180 + (180 + gyroWithDrift); 			// gyro raw -179 to 0 = gyro compass angle +181 to 359
			else
				gyroAngleMod = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
			gyroAngleOffset = gyroAngleMod - firstGyroScan; 			// Normalize gyro first scan to be zero
	
			if (gyroAngleOffset < 0)
				gyroAngleFB = gyroAngleOffset + 360; 					// assign 0 to -179 shifts to 181 to 359
			else
				gyroAngleFB = gyroAngleOffset; 							// assign 0 to 180
	
			/****************************************************************************************************
			 ** Read Right Drive and Winch motor encoder
			 ****************************************************************************************************/
			rightMotorPosition = rightMotor.getSelectedSensorPosition(0); // Get Right drive motor Position from Talon SRX
			winchMotorPosition = winchMotor.getSelectedSensorPosition(0); // Get Winch drive motor Position from Talon SRX
	
			/***************************************************************************************************************
			 * 
			 * LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT
			 * 
			 ***************************************************************************************************************/
	
			if (matchStartPosition == 0) // Robot start position is left (left bumper is aligned with left point of straight part of back wall
				{
					/**************************************************************
					 * Robot Start=Left Side, Go for Left Switch system clock is in milliseconds
					 **************************************************************/
					if (autonomousSelected.equals("startLeftSwitchLeft")) // Go for Left switch starting from Left side
						{
							startLeftSwitchLeft[0] = 0; 				// Step 0: Start of sequence
							startLeftSwitchLeft[1] = 50; 				// Step 1: Close intake arms solenoid
							startLeftSwitchLeft[2] = 2850; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startLeftSwitchLeft[3] = 2550; 				// Step 3: Start Decel to min speed, Start rotate CW to 90 Deg during decel
							startLeftSwitchLeft[4] = 400; 				// Step 4: Use gyro correction to move to correct angle
							startLeftSwitchLeft[5] = 100; 				// Step 5: Reset Drive encoders to zero
							startLeftSwitchLeft[6] = 2800; 				// Step 6: Start moving forward toward switch
							startLeftSwitchLeft[7] = 600; 				// Step 7: Stop forward movement and Open Arms
							startLeftSwitchLeft[8] = 1200; 				// Step 8: Back up robot away from switch 1200
							startLeftSwitchLeft[9] = 500; 				// Step 9: Move elevator down, close intake arms
							startLeftSwitchLeft[10] = 0; 				// Autonomous movement
							startLeftSwitchLeft[11] = 0; 				// Autonomous movement
							startLeftSwitchLeft[12] = 0; 				// Autonomous movement
							startLeftSwitchLeft[13] = 0; 				// Autonomous movement
							startLeftSwitchLeft[14] = 14900; 			// Autonomous movement is done
							startLeftSwitchLeftFirstPosition = 37500; 	// Encoder counts - first move forward
							startLeftSwitchLeftFirstAngle = 84; 		// Gyro angle for first turn toward switch
							startLeftSwitchLeftSecondPosition = 5700; 	// Encoder counts - second move forward
							startLeftSwitchLeftIntakeSpdRef = -0.5; 	// Intake motor speed ref to spit cube out
							startLeftSwitchLeftThirdPosition = -3400; 	// Encoder counts - third move backward
							startLeftSwitchLeftSecondAngle = 120; 		// Gyro angle for second turn toward switch
							/****************************************************
							 * Step 0 of start on Left, Switch on Left INIT
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startLeftSwitchLeftSeq = 1; 								// Begin step 1
									startLeftSwitchLeftTime[1] = autoSequenceTime; 				// Start time of step 1 when this step exits
								} // End of Step 0
				
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 9: Move elevator down, close intake arms
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 9) 									// Step 9: Move elevator down, close intake arms
								{
									System.out.println(
											"Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[9]));
									leftMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									intakeArmOpenCommand = false; 								// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand);
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	// Start magic motion for absolute position to move to SWITCH
				
									startLeftSwitchLeftTime[10] = autoSequenceTime; 			// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftSwitchLeftTime[9]) > startLeftSwitchLeft[9])) 
										{
											startLeftSwitchLeftSeq = 10; // Move to step 10
										}
								} // End of Step 9: Move elevator down, close intake arms
				
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 8: Back up robot away from switch
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 8) // Step 8: Back up robot away from switch
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", "
											+ (autoSequenceTime - startLeftSwitchLeftTime[8]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.30; 										// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchLeftFirstAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB);	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 			// Right side motor speed reference
				
									leftIntake.set(0.0); 									// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 									// Assign speed ref to Right PWM Talon
				
									startLeftSwitchLeftTime[9] = autoSequenceTime; 			// Start time of step 9 when this step exits
									if (((autoSequenceTime - startLeftSwitchLeftTime[8]) > startLeftSwitchLeft[8])
									|| (rightMotorPosition < startLeftSwitchLeftThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startLeftSwitchLeftSeq = 9; 					// Move to step 9
										}
								} // End of Step 8: Back up robot away from switch
			
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 7: Stop forward movement and Open Arms
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 7) 								// Step 7: Stop forward movement and Open Arms
								{
									System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[7]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 								// Stop motion
									rightMotorSpdRefAut = 0; 								// Stop motion
				
									leftIntake.set(startLeftSwitchLeftIntakeSpdRef); 		// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftSwitchLeftIntakeSpdRef); 		// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 							// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 				// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand);
				
									startLeftSwitchLeftTime[8] = autoSequenceTime; 			// Start time of step 8 when this step exits
									if (((autoSequenceTime - startLeftSwitchLeftTime[7]) > startLeftSwitchLeft[7])) {
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startLeftSwitchLeftSeq = 8; 						// Move to step 8
									}
				
								} // End of Step 7: Stop forward movement and Open Arms
			
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 6: Start moving forward toward switch
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 6) 							// Step 5: Start moving forward toward switch
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.35; 									// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchLeftFirstAngle;	 	// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); // Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 		// Right side motor speed reference
				
									startLeftSwitchLeftTime[7] = autoSequenceTime; 		// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftSwitchLeftTime[6]) > startLeftSwitchLeft[6])
									|| (rightMotorPosition > startLeftSwitchLeftSecondPosition)) 
										{
											leftMotorSpdRefAut = 0; // Stop motion
											rightMotorSpdRefAut = 0; // Stop motion
											startLeftSwitchLeftSeq = 7; // Move to step 7
										}
								} // End of Step 6: Start moving forward toward switch
			
							/****************************************************
							 * START LEFT - SWITCH LEFT Step 5: Reset Drive encoders to zero
							 ****************************************************/
							if ((startLeftSwitchLeftSeq == 5)) 									// Step 5: Reset Drive encoders to zero
								{
									System.out.println("Step 5 : " + (autoSequenceTime - startLeftSwitchLeftTime[5]));
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
				
									startLeftSwitchLeftTime[6] = autoSequenceTime;
									if ((autoSequenceTime - startLeftSwitchLeftTime[5]) > startLeftSwitchLeft[5]) 
										{
											startLeftSwitchLeftSeq = 6; 						// Move to step 6
										}
								} // End of Step5 Reset Drive encoders to zero
				
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 4: Use gyro correction to move to correct angle
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 4) // Step 4: Use gyro correction to move to correct angle
								{
									System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startLeftSwitchLeftTime[4]));
				
									yAxisRef = 0.0; 											// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startLeftSwitchLeftTime[5] = autoSequenceTime; 				// Start time of step 5 when this step exits
									if ( (autoSequenceTime - startLeftSwitchLeftTime[4]) > startLeftSwitchLeft[4]
									|| ((gyroAngleFB > startLeftSwitchLeftFirstAngle) && (gyroAngleFB < 100) ) ) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startLeftSwitchLeftSeq = 5; 						// Move to step 5
										}
								}	//End of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 3: Start Decel to min speed, 
							 * Start rotate CW to 270 Deg during decel
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
								{
									System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftSwitchLeftTime[3]));
									yAxisRef = 0.00; 											// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.17)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.17); 				// Right side motor speed reference
				
									startLeftSwitchLeftTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startLeftSwitchLeftTime[3]) > startLeftSwitchLeft[3])
									|| ((gyroAngleFB > (startLeftSwitchLeftFirstAngle - 5)) && (gyroAngleFB < 100))) 
										{
											startLeftSwitchLeftSeq = 4; 						// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 2: Drop intake arms, start moving forward to
							 * position left of scale,start raising elevator
							 ****************************************************/
							if (startLeftSwitchLeftSeq == 2)									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false); 								// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 								// Disable high speed gear on Left/Right Drive motors
				
									yAxisRef = 0.64; 											// Assign open loop speed reference
									gyroAngleSP = 0.0; 											// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									if (rightMotorPosition > 12000)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); // Start magic motion for absolute position to move to SWITCH
				
									startLeftSwitchLeftTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startLeftSwitchLeftTime[2]) > startLeftSwitchLeft[2]
									|| (rightMotorPosition > startLeftSwitchLeftFirstPosition)) 
										{
											startLeftSwitchLeftSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START LEFT - SWITCH LEFT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startLeftSwitchLeftSeq == 1)) 						// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
									intakeArmOpenCommand = false; 					// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 		// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand);
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 								// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 		// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else // Right half of compass
										firstGyroScan = gyroWithDrift; 				// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startLeftSwitchLeftTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startLeftSwitchLeftTime[1]) > startLeftSwitchLeft[1]) 
										{
											startLeftSwitchLeftSeq = 2; // Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
				
							} // End of Go for Left switch starting from Left side
		
					
					/**************************************************************
					 * Robot START=LEFT Side, Go for LEFT SCALE 
					 * system clock is in milliseconds
					 **************************************************************/
					else if (autonomousSelected.equals("startLeftScaleLeft")) // Go for Left scale starting from Left side
						{
			
							startLeftScaleLeft[0] = 0; 					// Step 0: Start of sequence
							startLeftScaleLeft[1] = 50; 				// Step 1: Close intake arms solenoid
							startLeftScaleLeft[2] = 3500; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startLeftScaleLeft[3] = 2550; 				// Step 3: Start Decel to min speed, Start rotate CW to 90 Deg during decel
							startLeftScaleLeft[4] = 400; 				// Step 4: Use gyro correction to move to correct angle
							startLeftScaleLeft[5] = 12800; 				// Step 5: Wait for Elevator to get up to height
							startLeftScaleLeft[6] = 2800; 				// Step 6: Start moving forward toward scale 1050
							startLeftScaleLeft[7] = 600; 				// Step 7: not used
							startLeftScaleLeft[8] = 300; 				// Step 8: Stop forward movement and Open Arms
							startLeftScaleLeft[9] = 800; 				// Step 9: Back up robot away from scale
							startLeftScaleLeft[10] = 3200; 				// Step 10: Move elevator down, close intake arms
							startLeftScaleLeft[11] = 2600; 				// Step 11: Move to 2nd cube
							startLeftScaleLeft[12] = 800; 				// Step 12: Back up with second cube
							startLeftScaleLeft[13] = 2000; 				// Step 13: Raise elevator up, rotate toward scale
							startLeftScaleLeft[14] = 1700; 				// Step 14: Start moving forward toward scale again
							startLeftScaleLeft[15] = 300; 				// Step 15: Stop forward movement and Open Arms
							startLeftScaleLeft[16] = 1800; 				// Step 16: Back up robot away from scale
							startLeftScaleLeft[17] = 1000; 				// Step 17: Drop Elevator to bottom
							startLeftScaleLeft[18] = 100; 				// Autonomous movement is done
							
							startLeftScaleLeftPosition[1] 	= 0;		//Position for step 1
							startLeftScaleLeftAngle[1] 		= 0;		//Angle for step 1
							startLeftScaleLeftPosition[2] 	= 52500;	//Position for step 2
							startLeftScaleLeftAngle[2] 		= 0.0;		//Angle for step 2 - used
							startLeftScaleLeftPosition[3] 	= 0;		//Position for step 3
							startLeftScaleLeftAngle[3] 		= 45;		//Angle for step 3
							startLeftScaleLeftPosition[4] 	= 0;		//Position for step 4
							startLeftScaleLeftAngle[4] 		= 45;		//Angle for step 4
							startLeftScaleLeftPosition[5] 	= 0;		//Position for 	Step 5: Wait for Elevator to get up to height
							startLeftScaleLeftAngle[5] 		= 0;		//Angle for 	Step 5: Wait for Elevator to get up to height
							startLeftScaleLeftPosition[6] 	= 1000;		//Position for step 6
							startLeftScaleLeftAngle[6] 		= 45;		//Angle for step 6
							startLeftScaleLeftPosition[7] 	= 0;		//Position for 	Step 7: Stop forward movement and Open Arms 400
							startLeftScaleLeftAngle[7] 		= 0.0;		//Angle for 	Step 7: Stop forward movement and Open Arms 400
							startLeftScaleLeftPosition[8] 	= 0;		//Position for 	Step 8: Stop forward movement and Open Arms
							startLeftScaleLeftAngle[8] 		= 0;		//Angle for 	Step 8: Stop forward movement and Open Arms
							startLeftScaleLeftPosition[9] 	= -3200;	//Position for 	Step 9: Back up robot away from scale
							startLeftScaleLeftAngle[9] 		= 80;		//Angle for 	Step 9: Back up robot away from scale
							startLeftScaleLeftPosition[10] 	= 0;		//Position for 	Step 10: Move elevator down, close intake arms
							startLeftScaleLeftAngle[10] 	= 160;		//Angle for 	Step 10: Move elevator down, close intake arms
							startLeftScaleLeftPosition[11] 	= 14600;	//Position for 	Step 11: Move to 2nd cube
							startLeftScaleLeftAngle[11] 	= 160;		//Angle for 	Step 11: Move to 2nd cube
							startLeftScaleLeftPosition[12] 	= 10000;	//Position for 	Step 12: Back up with second cube
							startLeftScaleLeftAngle[12] 	= 110;		//Angle for 	Step 12: Back up with second cube
							startLeftScaleLeftPosition[13] 	= 0;		//Position for 	Step 13: Raise elevator up, rotate toward scale
							startLeftScaleLeftAngle[13] 	= 30;		//Angle for 	Step 13: Raise elevator up, rotate toward scale
							startLeftScaleLeftPosition[14] 	= 8600;		//Position for 	Step 14: Start moving forward toward scale again
							startLeftScaleLeftAngle[14] 	= 25;		//Angle for 	Step 14: Start moving forward toward scale again
							startLeftScaleLeftPosition[15] 	= 0;		//Position for 	Step 15: Stop forward movement and Open Arms
							startLeftScaleLeftAngle[15] 	= 0;		//Angle for 	Step 15: Stop forward movement and Open Arms
							startLeftScaleLeftPosition[16] 	= -3300;	//Position for 	Step 16: Back up robot away from scale
							startLeftScaleLeftAngle[16] 	= 95;		//Angle for 	Step 16: Back up robot away from scale
							startLeftScaleLeftPosition[17] 	= 0;		//Position for 	Step 17: Drop Elevator to bottom
							startLeftScaleLeftAngle[17] 	= 0;		//Angle for 	Step 17: Drop Elevator to bottom
							
							startLeftScaleLeftIntakeSpdRef = 0.4; 		// Intake motor speed ref to spit cube out
							startLeftScaleLeftIntakeGrabRef = -0.9; 	// Intake motor speed ref to grab cube in	
							startLeftSwitchLeftIntakeHoldRef = -0.55; 	// To make sure we hold the cube.
							
							//startLeftScaleLeftFirstPosition = 52500; 	// Encoder counts - first move forward
							//startLeftScaleLeftFirstAngle = 45; 		// Gyro angle for first turn toward scale
							//startLeftScaleLeftSecondPosition = 4300; 	// Encoder counts - second move forward
							//startLeftScaleLeftThirdPosition = -2100; 	// Encoder counts - third move backward
							//startLeftScaleLeftSecondAngle = 30; 		// Gyro angle for second turn toward scale
							//startLeftScaleLeftThirdAngle = 130; 		// Gyro angle for third turn toward cube Scale
							//startLeftScaleLeftFourthPosition = 10400; // Encoder counts - third move backward
			
							/****************************************************
							 * Step 0 of start on Left, SCALE on Left INIT
							 ****************************************************/
							if (startLeftScaleLeftSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startLeftScaleLeftSeq = 1; 									// Begin step 1
									startLeftScaleLeftTime[1] = autoSequenceTime; 				// Start time of step 1 when this step exits
								} // End of Step 0
			
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 17: Drop Elevator to bottom
							 ****************************************************/
							if (startLeftScaleLeftSeq == 17) 										// Step 17: Drop Elevator to bottom
								{
									System.out.println("Step 17 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[17]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[17]; 						// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);		// Start magic motion for absolute position to move to floor
									
									startLeftScaleLeftTime[18] = autoSequenceTime; 					// Start time of step 18 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[17]) > startLeftScaleLeft[17])
									|| ((gyroAngleFB > (gyroAngleSP - 2)) && (gyroAngleFB < 200)) )
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 18; 							// Move to step 18
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 17: Drop Elevator to bottom
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 16: Back up robot away from scale
							 ****************************************************/
							if (startLeftScaleLeftSeq == 16) 										// Step 16: Back up robot away from scale
								{
									System.out.println("Step 16 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[16]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.35; 												// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[9]; 						// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 						// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.25); 					// Right side motor speed reference
				
									leftIntake.set(0.0); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 											// Assign speed ref to Right PWM Talon
				
									startLeftScaleLeftTime[17] = autoSequenceTime; 					// Start time of step 17 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[16]) > startLeftScaleLeft[16])
									|| (rightMotorPosition < startLeftScaleLeftPosition[16])) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 17; 							// Move to step 17
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 16: Back up robot away from scale
							
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 15: Stop forward movement and Open Arms
							 ****************************************************/
							if (startLeftScaleLeftSeq == 15) 										// Step 15: Stop forward movement and Open Arms
								{
									System.out.println("Step 15 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[15]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startLeftScaleLeftIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftScaleLeftIntakeSpdRef); 				// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startLeftScaleLeftTime[16] = autoSequenceTime; 					// Start time of step 8 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[15]) > startLeftScaleLeft[15])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startLeftScaleLeftSeq = 16; 							// Move to step 16
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 15: Stop forward movement and Open Arms
							
							
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 14: Start moving forward toward scale again
							 ****************************************************/
							if (startLeftScaleLeftSeq == 14) 									// Step 14: Start moving forward toward scale again
								{
									System.out.println("Step 14 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[14]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.37; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[14]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									startLeftScaleLeftTime[15] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[14]) > startLeftScaleLeft[14])
									|| (rightMotorPosition > startLeftScaleLeftPosition[14])) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startLeftScaleLeftSeq = 15; 						// Move to step 15
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 14: Start moving forward toward scale again
							
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 13: Raise elevator up, rotate toward scale
							 ****************************************************/
							if (startLeftScaleLeftSeq == 13) 										// Step 13: Raise elevator up, rotate toward scale
								{
									System.out.println("Step 13 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[13]) + " , Gyro: " + gyroAngleFB);
				
									winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);		// Start magic motion for absolute position to move to scale

									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[13]; 						// Assign gyro angle setpoint in 0-359 degrees
				
									
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.22)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.22); 					// Right side motor speed reference
				
									startLeftScaleLeftTime[14] = autoSequenceTime; 					// Start time of step 10 when this step exits
									if ( ((autoSequenceTime - startLeftScaleLeftTime[13]) > startLeftScaleLeft[13])
									|| ( (winchMotorPosition > (winchScaleTopRef - 5000)) 
									&& ( (gyroAngleFB < (gyroAngleSP + 5)) || (gyroAngleFB > 358) ) ) )
										{
											leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 14; 							// Move to step 14
											System.out.println("total time= " + autoSequenceTime);
										}
				
								} // End of Step 13: Raise elevator up, rotate toward scale
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 12: Back up with second cube
							 ****************************************************/
							if (startLeftScaleLeftSeq == 12) // Step 12: Back up with second cube
								{
									System.out.println("Step 12 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[12]) + " , Gyro: " + gyroAngleFB);
									
									yAxisRef = -0.30; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[12]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									leftIntake.set(startLeftSwitchLeftIntakeHoldRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftSwitchLeftIntakeHoldRef); 			// Assign speed ref to Right PWM Talon
				
									startLeftScaleLeftTime[13] = autoSequenceTime; 				// Start time of step 9 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[12]) > startLeftScaleLeft[12])
									|| (rightMotorPosition < startLeftScaleLeftPosition[12]))  
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startLeftScaleLeftSeq = 13; 						// Move to step 10
										}
								} // End of Step 12: Back up with second cube
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 11: Move to 2nd cube
							 ****************************************************/
							if (startLeftScaleLeftSeq == 11) // Step 11: Move to 2nd cube
								{
									System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[11]) + " , Gyro: " + gyroAngleFB);
									
									yAxisRef = 0.65; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[11]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									leftIntake.set(startLeftScaleLeftIntakeGrabRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftScaleLeftIntakeGrabRef); 			// Assign speed ref to Right PWM Talon
				
									startLeftScaleLeftTime[12] = autoSequenceTime; 				// Start time of step 12 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[11]) > startLeftScaleLeft[11])
									|| (rightMotorPosition > startLeftScaleLeftPosition[11])) 
										{
											leftMotorSpdRefAut = 0.0; 							// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 							// Assign open loop speed reference
											startLeftScaleLeftSeq = 12; 						// Move to step 12
										}
				
								} // End of Step 11: Move to 2nd cube
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 10: Move elevator down, close intake arms
							 ****************************************************/
							if (startLeftScaleLeftSeq == 10) 										// Step 10: Move elevator down, close intake arms
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[10]) + " , Gyro: " + gyroAngleFB);
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);		// Start magic motion for absolute position to move to scale

									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[10]; 						// Assign gyro angle setpoint in 0-359 degrees
									
				
									intakeArmOpenCommand = false; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.20)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.20); 					// Right side motor speed reference
				
									startLeftScaleLeftTime[11] = autoSequenceTime; 					// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[10]) > startLeftScaleLeft[10])
									|| ((gyroAngleFB > (gyroAngleSP - 2)) && (gyroAngleFB < 200)) )
										{
											leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 11; 							// Move to step 11
										}
				
								} // End of Step 10: Move elevator down, close intake arms
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 9: Back up robot away from scale
							 ****************************************************/
							if (startLeftScaleLeftSeq == 9) 										// Step 9: Back up robot away from scale
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[9]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.40; 												// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[9]; 						// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 						// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.25); 					// Right side motor speed reference
				
									leftIntake.set(0.0); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 											// Assign speed ref to Right PWM Talon
				
									startLeftScaleLeftTime[10] = autoSequenceTime; 					// Start time of step 9 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[9]) > startLeftScaleLeft[9])
									|| (rightMotorPosition < startLeftScaleLeftPosition[9])) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 10; 							// Move to step 10
					
										}
								} // End of Step 9: Back up robot away from scale
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 8: Stop forward movement and Open Arms
							 ****************************************************/
							if (startLeftScaleLeftSeq == 8) 									// Step 8: Stop forward movement and Open Arms
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[8]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startLeftScaleLeftIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftScaleLeftIntakeSpdRef); 				// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startLeftScaleLeftTime[9] = autoSequenceTime; 					// Start time of step 8 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[8]) > startLeftScaleLeft[8])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleLeftSeq = 9; 								// Move to step 9
										}
								} // End of Step 8: Stop forward movement and Open Arms
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 7: Start rotate CCW to 0 Deg during decel
							 ****************************************************/
							if (startLeftScaleLeftSeq == 7) 									// Step 7: Start Decel to min speed, Start rotate CW to 30 Deg during decel
								{
									System.out.println("Step 7 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftScaleLeftTime[3]));
									yAxisRef = 0.15; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[7]; 											// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25); 				// Right side motor speed reference
				
									startLeftScaleLeftTime[8] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[7]) > startLeftScaleLeft[7])
									|| ((gyroAngleFB < (gyroAngleSP + 2)) && (gyroAngleFB < 100))) 
										{
											// startLeftScaleLeftSeq = 8; 						//Move to step 8
											startLeftScaleLeftSeq = 18; 						// Move to step 18
										}
								} // End of Step 7: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 6: Start moving forward toward scale
							 ****************************************************/
							if (startLeftScaleLeftSeq == 6) 									// Step 6: Start moving forward toward scale
								{
									System.out.println("Step 6LC : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.37; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[6]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									startLeftScaleLeftTime[8] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[6]) > startLeftScaleLeft[6])
									|| (rightMotorPosition > startLeftScaleLeftPosition[6])) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startLeftScaleLeftSeq = 8; 							// Move to step 8
										}
								} // End of Step 6: Start moving forward toward scale
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 5: Wait for Elevator to get up to height
							 ****************************************************/
							if ((startLeftScaleLeftSeq == 5)) 									// Step 5: Wait for Elevator to get up to height
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[5]) + " , Gyro: " + gyroAngleFB);
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
				
									startLeftScaleLeftTime[6] = autoSequenceTime;
									if (((autoSequenceTime - startLeftScaleLeftTime[5]) > startLeftScaleLeft[5])
									|| (winchMotorPosition > (winchScaleTopRef - 100))) 
										{
											startLeftScaleLeftSeq = 6; // Move to step 6
										}
								} // End of Step 5: Wait for Elevator to get up to height
			
							
							
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 4: Use gyro correction to move to correct angle
							 ****************************************************/
							if (startLeftScaleLeftSeq == 4) 									// Step 4: Use gyro correction to move to correct angle
								{
									System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startLeftScaleLeftTime[4]));
				
									yAxisRef = 0.35; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[4]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startLeftScaleLeftTime[5] = autoSequenceTime; 				// Start time of step 5 when this step exits
									if ((autoSequenceTime - startLeftScaleLeftTime[4]) > startLeftScaleLeft[4]
									|| ((gyroAngleFB > gyroAngleSP) && (gyroAngleFB < 100))) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startLeftScaleLeftSeq = 5; 							// Move to step 5
										}
								} // END of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 3: Start Decel to min speed, Start rotate CW to
							 * 30 Deg during decel
							 ****************************************************/
							if (startLeftScaleLeftSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CW to 30 Deg during decel
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[3]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.37; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[3]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.20)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0); 				// Right side motor speed reference
				
									startLeftScaleLeftTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startLeftScaleLeftTime[3]) > startLeftScaleLeft[3])
									|| ((gyroAngleFB > (gyroAngleSP - 2)) && (gyroAngleFB < 100))) 
										{
											startLeftScaleLeftSeq = 4; 							// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 2: Drop intake arms, start moving forward to
							 * position left of SCALE
							 ****************************************************/
							if (startLeftScaleLeftSeq == 2) 									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + " , " + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
				
									yAxisRef = 0.75; 											// Assign open loop speed reference
									gyroAngleSP = startLeftScaleLeftAngle[2]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									if (rightMotorPosition > (startLeftScaleLeftPosition[2] - 13000)) 
										{
											leftMotorShiftHigh.set(false); 						// Enable high speed gear on Left/Right Drive motors
											leftMotorShiftLow.set(true); 						// Enable high speed gear on Left/Right Drive motor
										}
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									if (rightMotorPosition > 4000)
										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef); // Start magic motion for absolute position to move to SCALE
				
									startLeftScaleLeftTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startLeftScaleLeftTime[2]) > startLeftScaleLeft[2]
									|| (rightMotorPosition > startLeftScaleLeftPosition[2])) 
										{
											startLeftScaleLeftSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START LEFT - SCALE LEFT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startLeftScaleLeftSeq == 1)) 						// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
				
									intakeArmOpenCommand = false; 					// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 		// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 		// Intake arms CLOSE solenoid
				
									leftMotorShiftHigh.set(true); 					// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(false); 					// Enable high speed gear on Left/Right Drive motor
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 								// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 		// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 											// Right half of compass
										firstGyroScan = gyroWithDrift; 				// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startLeftScaleLeftTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startLeftScaleLeftTime[1]) > startLeftScaleLeft[1]) 
										{
											startLeftScaleLeftSeq = 2; 				// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
			
						} // End of START LEFT, go for SCALE LEFT
		
					
					/**************************************************************
					 * Robot START LEFT Side, Go for SWITCH RIGHT
					 * system clock is in milliseconds
					 **************************************************************/
					//else if (ourGameData.equals("RR") && goOpposite) // Go for Right switch starting from Left side
					else if (autonomousSelected.equals("startLeftSwitchRight") ) // Go for Right switch starting from Left side
						{
							startLeftSwitchRight[0] = 0; 				// Step 0: Start of sequence
							startLeftSwitchRight[1] = 50; 				// Step 1: Close intake arms solenoid
							startLeftSwitchRight[2] = 3500; 			// Step 2: Move past Switch frame
							startLeftSwitchRight[3] = 1500; 			// Step 3: Rotate CW 90 degrees to get behind switch
							startLeftSwitchRight[4] = 100; 				// Step 4: Reset Drive encoders to zero
							startLeftSwitchRight[5] = 3500; 			// Step 5: Drive forward, drop arms, raise elevator
							startLeftSwitchRight[6] = 1500; 			// Step 6: Rotate CW to 180 degrees to face switch
							startLeftSwitchRight[7] = 1600; 			// Step 7: Start moving forward toward switch
							startLeftSwitchRight[8] = 600; 				// Step 8: Stop forward movement and Open Arms
							startLeftSwitchRight[9] = 2000; 			// Step 9: Back up robot away from switch 1200
							startLeftSwitchRight[10] = 500; 			// Step 10: Move elevator down, close intake arms
							startLeftSwitchRight[11] = 0; 				// Autonomous movement
							startLeftSwitchRight[12] = 0;				// Autonomous movement
							startLeftSwitchRight[13] = 0; 				// Autonomous movement
							startLeftSwitchRight[14] = 0; 				// Autonomous movement is done
							startLeftSwitchRightFirstPosition = 43200; 	// Encoder counts - first move forward
							startLeftSwitchRightFirstAngle = 89; 		// Gyro angle for first turn toward switch
							startLeftSwitchRightSecondPosition = 45500; // Encoder counts - second move forward
							startLeftSwitchRightSecondAngle = 149; 		// Gyro angle for second turn toward switch
							startLeftSwitchRightThirdPosition = 52000; 	// Encoder counts - third move forward
							startLeftSwitchRightIntakeSpdRef = 0.5; 	// Intake motor speed ref to spit cube out
							startLeftSwitchRightForthPosition = -4800; 	// Encoder counts - forth move backward
			
							/****************************************************
							 * Step 0 of start on Left, Switch on RIGHT INIT
							 ****************************************************/
							if (startLeftSwitchRightSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startLeftSwitchRightSeq = 1; 								// Begin step 1
									startLeftSwitchRightTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
				
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 10: Move elevator down, close intake arms
							 ****************************************************/
							if (startLeftSwitchRightSeq == 10) // Step 10: Move elevator down, close intake arms
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchRightTime[10]));
									leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
									leftIntake.set(0.0); 									// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 									// Assign speed ref to Right PWM Talon
									intakeArmOpen.set(false); 								// Intake arms OPEN solenoid
									intakeArmClose.set(true); 								// Intake arms CLOSE solenoid
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);// Start magic motion for absolute position
																							// to move to SWITCH
				
									startLeftSwitchRightTime[11] = autoSequenceTime; 		// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[10]) > startLeftSwitchRight[10])) {
										startLeftSwitchRightSeq = 11; 						// Move to step 11
									}
				
								} // End of Step 10: Move elevator down, close intake arms
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 9: Back up robot away from switch
							 ****************************************************/
							if (startLeftSwitchRightSeq == 9) // Step 8: Back up robot away from switch
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchRightTime[9]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.25; 												// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchRightSecondAngle; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.15); 					// Right side motor speed reference
				
									startLeftSwitchRightTime[10] = autoSequenceTime; 				// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[9]) > startLeftSwitchRight[9])
									|| (rightMotorPosition < startLeftSwitchRightForthPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; // Stop motion
											startLeftSwitchRightSeq = 10; // Move to step 10
										}
								} // End of Step 9: Back up robot away from switch
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 8: Stop forward movement and Open Arms
							 ****************************************************/
							if (startLeftSwitchRightSeq == 8) // Step 8: Stop forward movement and Open Arms
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchRightTime[8]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startLeftSwitchRightIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftSwitchRightIntakeSpdRef); 				// Assign speed ref to Right PWM Talon
				
									intakeArmOpen.set(true); 										// Intake arms OPEN solenoid
									intakeArmClose.set(false); 										// Intake arms CLOSE solenoid
				
									startLeftSwitchRightTime[9] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[8]) > startLeftSwitchRight[8])) {
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startLeftSwitchRightSeq = 9; 								// Move to step 8
									}
				
								} // End of Step 8: Stop forward movement and Open Arms
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 7: Start moving forward toward switch
							 ****************************************************/
							if (startLeftSwitchRightSeq == 7) // Step 7: Start moving forward toward switch
								{
									System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchRightTime[7]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.20; 												// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchRightSecondAngle; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 					// Right side motor speed reference
				
									startLeftSwitchRightTime[9] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[7]) > startLeftSwitchRight[7])
									|| (rightMotorPosition > startLeftSwitchRightThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startLeftSwitchRightSeq = 8; 							// Move to step 8
										}
								} // End of Step 7: Start moving forward toward switch
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 6: Rotate CW to 180 degrees to face switch
							 ****************************************************/
							if (startLeftSwitchRightSeq == 6) 										// Step 6: Rotate CW to 180 degrees to face switch
								{
									System.out.println("Step 6 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftSwitchRightTime[6]));
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchRightSecondAngle; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.25)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.25); 					// Right side motor speed reference
				
									startLeftSwitchRightTime[7] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[6]) > startLeftSwitchRight[6])
									|| ((gyroAngleFB > (startLeftSwitchRightSecondAngle - 3)) && (gyroAngleFB < 200))) 
										{
											startLeftSwitchRightSeq = 7; 							// Move to step 7
										}
								} // End of Step 6: Rotate CW to 180 degrees to face switch
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 5: Drive forward, drop arms, raise elevator
							 ****************************************************/
							if (startLeftSwitchRightSeq == 5) 										// Step 5: Drive forward, drop arms, raise elevator
							{
								System.out.println("Step 5 pos: " + gyroAngleFB + "," + (autoSequenceTime - startLeftSwitchRightTime[5]));
			
								intakeArmPivotDownCommand = true; 									// Intake Arm to pivot Down command
								intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 					// Enable Intake Arm to pivot DOWN
								intakeArmPivotUP.set(!intakeArmPivotDownCommand); 					// Enable Intake Arm to pivot UP
			
								yAxisRef = 0.55; 													// Assign open loop speed reference
								gyroAngleSP = startLeftSwitchRightFirstAngle; 						// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 				// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 						// Right side motor speed reference
			
								if (rightMotorPosition > 10000)
									winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); 		// Start magic motion for absolute position to move to SWITCH
			
								startLeftSwitchRightTime[6] = autoSequenceTime; 					// Start time of step 6 when this step exits
								if (((autoSequenceTime - startLeftSwitchRightTime[5]) > startLeftSwitchRight[5])
								|| (rightMotorPosition > startLeftSwitchRightSecondPosition)) 
									{
										startLeftSwitchRightSeq = 6; 								// Move to step 6
									}
							} // END of Step 5: Drive forward, drop arms, raise elevator
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 4: Reset Drive encoders to zero
							 ****************************************************/
							if (startLeftSwitchRightSeq == 4) 										// Step 4: Reset Drive encoders to zero
								{
									System.out.println("Step 4 : " + (autoSequenceTime - startLeftSwitchRightTime[4]));
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 		// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
				
									startLeftSwitchRightTime[5] = autoSequenceTime;
									if ((autoSequenceTime - startLeftSwitchRightTime[4]) > startLeftSwitchRight[4]) 
										{
											startLeftSwitchRightSeq = 5; 							// Move to step 5
										}
								} // End of Step 4 Reset Drive encoders to zero
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 3: Rotate CW 90 degrees to get behind switch
							 ****************************************************/
							if (startLeftSwitchRightSeq == 3) 										// Step 3: Rotate CW 90 degrees to get behind switch
								{
									System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftSwitchRightTime[3]));
									
									yAxisRef = 0.30; 												// Assign open loop speed reference
									gyroAngleSP = startLeftSwitchRightFirstAngle; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.27)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startLeftSwitchRightTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startLeftSwitchRightTime[3]) > startLeftSwitchRight[3])
									|| ((gyroAngleFB > (startLeftSwitchRightFirstAngle - 3)) && (gyroAngleFB < 110))) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftSwitchRightSeq = 4; 							// Move to step 4
										}
								} // End of Step 3: Rotate CW 90 degrees to get behind switch
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 2: Move past Switch frame
							 ****************************************************/
							if (startLeftSwitchRightSeq == 2) 										// Step 2: Move past Switch frame
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									leftMotorShiftHigh.set(false); 									// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 									// Disable high speed gear on Left/Right Drive motors
				
									yAxisRef = 0.85; 												// Assign open loop speed reference
									gyroAngleSP = 0.0; 												// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 					// Right side motor speed reference
				
									startLeftSwitchRightTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startLeftSwitchRightTime[2]) > startLeftSwitchRight[2]
											|| (rightMotorPosition > startLeftSwitchRightFirstPosition)) 
										{
											startLeftSwitchRightSeq = 3;
										}
								} // End of Step 2: Move past Switch frame
			
							/****************************************************
							 * START LEFT - SWITCH RIGHT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startLeftSwitchRightSeq == 1)) 								// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
									intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
									intakeArmClose.set(true); 									// Intake arms CLOSE solenoid
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 											// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 					// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 														// Right half of compass
										firstGyroScan = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startLeftSwitchRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startLeftSwitchRightTime[1]) > startLeftSwitchRight[1]) 
										{
											startLeftSwitchRightSeq = 2; 						// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
			
						} // End of Go for RIGHT SWITCH starting from LEFT SIDE
		
					
					/**************************************************************
					 * Robot START=LEFT Side, Go for RIGHT SCALE game data="RR" system clock is in
					 * milliseconds
					 **************************************************************/
					//else if (ourGameData.equals("TT") && goForScale && goOpposite) // Go for Right Scale starting from Left side
					else if (autonomousSelected.equals("startLeftScaleRight") ) // Go for Right Scale starting from Left side						
						{
							startLeftScaleRight[0] = 0; 				// Step 0: Start of sequence
							startLeftScaleRight[1] = 50; 				// Step 1: Close intake arms solenoid, shift to high speed
							startLeftScaleRight[2] = 3500; 				// Step 2: Move past Switch frame
							startLeftScaleRight[3] = 1500; 				// Step 3: Rotate CW 90 degrees to get behind switch
							startLeftScaleRight[4] = 100; 				// Step 4: Reset Drive encoders to zero
							startLeftScaleRight[5] = 2400; 				// Step 5: Drive forward, drop arms, raise elevator
							startLeftScaleRight[6] = 2300; 				// Step 6: Rotate CCW to 0 degrees to face Scale
							startLeftScaleRight[7] = 20000; 			// Step 7: Start moving forward toward switch
							startLeftScaleRight[8] = 600; 				// Step 8: Stop forward movement and Open Arms
							startLeftScaleRight[9] = 2300; 				// Step 9: Back up robot away from switch
							startLeftScaleRight[10] = 2200; 			// Step 10: Move elevator down, close intake arms, rotate to face switch
							startLeftScaleRight[11] = 50; 				// Step 11: Stop motion
							startLeftScaleRight[12] = 0; 				// Autonomous movement
							startLeftScaleRight[13] = 0; 				// Autonomous movement
							startLeftScaleRight[14] = 0;				// Autonomous movement is done
							startLeftScaleRightFirstPosition = 44000; 	// Encoder counts - first move forward
							startLeftScaleRightFirstAngle = 90; 		// Gyro angle for first turn toward scale
							startLeftScaleRightSecondPosition = 45500; 	// Encoder counts - second move forward
							startLeftScaleRightSecondAngle = 356; 		// Gyro angle for second turn toward scale
							startLeftScaleRightThirdPosition = 71200; 	// Encoder counts - third move forward toward scale
							startLeftScaleRightThirdAngle = 200; 		// Gyro angle for third turn away from scale
							startLeftScaleRightIntakeSpdRef = -0.5; 	// Intake motor speed ref to spit cube out
							startLeftScaleRightForthPosition = -6000; 	// Encoder counts - forth move backward
							startLeftScaleRightFourthAngle = 160; 		// Gyro angle for fourth turn away from scale
			
							/****************************************************
							 * Step 0 of start on RIGHT, SCALE on LEFT INIT
							 ****************************************************/
							if (startLeftScaleRightSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startLeftScaleRightSeq = 1; 								// Begin step 1
									startLeftScaleRightTime[1] = autoSequenceTime; 				// Start time of step 1 when this step exits
								} // End of Step 0
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 11: Stop motion
							 ****************************************************/
							if (startLeftScaleRightSeq == 11) 								// Step 11: Stop motion
								{
									System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[11]));
				
									leftMotorSpdRefAut = 0; 								// Stop motion
									rightMotorSpdRefAut = 0; 								// Stop motion
				
									startLeftScaleRightTime[12] = autoSequenceTime; 		// Start time of step 12 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[11]) > startLeftScaleRight[11])) 
										{
											startLeftScaleRightSeq = 12; // Move to step 12
										}
				
								} // End of Step 11: Stop motion
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 10: Move elevator down, close intake arms, rotate to face switch
							 ****************************************************/
							if (startLeftScaleRightSeq == 10) // Step 10: Move elevator down, close intake arms, rotate to face switch
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[10]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.0; 										// Assign open loop speed reference
									gyroAngleSP = startLeftScaleRightThirdAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.21)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.21); 			// Right side motor speed reference
				
									leftIntake.set(0.0); 									// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 									// Assign speed ref to Right PWM Talon
									intakeArmOpen.set(false); 								// Intake arms OPEN solenoid
									intakeArmClose.set(true); 								// Intake arms CLOSE solenoid
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);// Start magic motion for absolute position to move to SWITCH
				
									startLeftScaleRightTime[11] = autoSequenceTime; 		// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[10]) > startLeftScaleRight[10])
									|| ((gyroAngleFB < (gyroAngleSP + 3)) && (gyroAngleFB > 100) ) ) 
										{
											startLeftScaleRightSeq = 11; // Move to step 11
										}
				
								} // End of Step 10: Move elevator down, close intake arms, rotate to face switch
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 9: Back up robot away from switch
							 ****************************************************/
							if (startLeftScaleRightSeq == 9) // Step 8: Back up robot away from switch
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[9]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.27; 										// Assign open loop speed reference
									gyroAngleSP = startLeftScaleRightThirdAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 			// Right side motor speed reference
				
									startLeftScaleRightTime[10] = autoSequenceTime; 		// Start time of step 10 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[9]) > startLeftScaleRight[9])
									|| (rightMotorPosition < startLeftScaleRightForthPosition)) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; // Stop motion
											startLeftScaleRightSeq = 10; // Move to step 10
										}
								} // End of Step 9: Back up robot away from switch
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 8: Stop forward movement and Open Arms
							 ****************************************************/
							if (startLeftScaleRightSeq == 8) // Step 8: Stop forward movement and Open Arms
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[8]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0;					 					// Stop motion
				
									leftIntake.set(startLeftScaleRightIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftScaleRightIntakeSpdRef); 				// Assign speed ref to Right PWM Talon
				
									intakeArmOpen.set(true); 										// Intake arms OPEN solenoid
									intakeArmClose.set(false); 										// Intake arms CLOSE solenoid
				
									startLeftScaleRightTime[9] = autoSequenceTime; 					// Start time of step 8 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[8]) > startLeftScaleRight[8])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleRightSeq = 9; 							// Move to step 8
										}
				
								} // End of Step 8: Stop forward movement and Open Arms
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 7: Start moving forward toward scale
							 ****************************************************/
							if (startLeftScaleRightSeq == 7) // Step 7: Start moving forward toward switch
							{
								System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[7]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.30; 											// Assign open loop speed reference
								gyroAngleSP = startLeftScaleRightSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
			
								startLeftScaleRightTime[8] = autoSequenceTime; 				// Start time of step 7 when this step exits
								if (((autoSequenceTime - startLeftScaleRightTime[7]) > startLeftScaleRight[7])
								|| (rightMotorPosition > startLeftScaleRightThirdPosition)) 
									{
										startLeftScaleRightSeq = 8; // Move to step 8
									}
							} // End of Step 7: Start moving forward toward switch
			
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 6: Rotate CCW to 0 degrees to face Scale
							 ****************************************************/
							if (startLeftScaleRightSeq == 6) 								// Step 6: Rotate CCW to 0 degrees to face Scale
								{
									System.out.println("Step 6 : " + rightMotorPosition + " , " + (autoSequenceTime - startLeftScaleRightTime[6]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.15; 										// Assign open loop speed reference
									gyroAngleSP = startLeftScaleRightSecondAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.0); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.31); 			// Right side motor speed reference
				
									startLeftScaleRightTime[7] = autoSequenceTime; 			// Start time of step 7 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[6]) > startLeftScaleRight[6])
									|| ( (winchMotorPosition > winchScaleRef) && (gyroAngleFB < gyroAngleSP) && (gyroAngleFB > 200) ) ) 
										{
											startLeftScaleRightSeq = 7; // Move to step 7
										}
								} // End of Step 6: Rotate CCW to 0 degrees to face Scale
			
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 5: Drive forward, drop arms, raise elevator
							 ****************************************************/
							if (startLeftScaleRightSeq == 5) 								// Step 5: Drive forward, drop arms, raise elevator
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[5]) + " , Gyro: " + gyroAngleFB);
				
//									intakeArmPivotDownCommand = true; 						// Intake Arm to pivot Down command
//									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot DOWN
//									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot UP
				
									if (rightMotorPosition > (startLeftScaleRightSecondPosition - 15000)) 
										{
											leftMotorShiftHigh.set(false); 					// Enable high speed gear on Left/Right Drive motors
											leftMotorShiftLow.set(true); 					// Enable high speed gear on Left/Right Drive motor
										}
				
									yAxisRef = 0.75; 										// Assign open loop speed reference
									gyroAngleSP = startLeftScaleRightFirstAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 			// Right side motor speed reference
				
									if (rightMotorPosition > 1000)
										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef); // Start magic motion for absolute position to move to SWITCH
				
									startLeftScaleRightTime[6] = autoSequenceTime; 			// Start time of step 6 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[5]) > startLeftScaleRight[5])
									|| (rightMotorPosition > startLeftScaleRightSecondPosition)) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startLeftScaleRightSeq = 6; 					// Move to step 6
										}
								} // END of Step 5: Drive forward, drop arms, raise elevator
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 4: Reset Drive encoders to zero
							 ****************************************************/
							if (startLeftScaleRightSeq == 4) // Step 4: Reset Drive encoders to zero
								{
									System.out.println("Step 4 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[4]) + " , Gyro: " + gyroAngleFB);
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 			// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);			// Zero the position encoder
				
				
									startLeftScaleRightTime[5] = autoSequenceTime;
									if ((autoSequenceTime - startLeftScaleRightTime[4]) > startLeftScaleRight[4]) 
										{
											startLeftScaleRightSeq = 5; 								// Move to step 5
										}
								} // End of Step 4 Reset Drive encoders to zero
			
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 3: Rotate CW 90 degrees to get behind switch
							 ****************************************************/
							if (startLeftScaleRightSeq == 3) 								// Step 3: Rotate CW 90 degrees to get behind switch
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[3]) + " , Gyro: " + gyroAngleFB);
				
									intakeArmPivotDownCommand = true; 						// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false); 							// Disable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 							// Disable high speed gear on Left/Right Drive motor
				
									yAxisRef = 0.30; 										// Assign open loop speed reference
									gyroAngleSP = startLeftScaleRightFirstAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.27); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									startLeftScaleRightTime[4] = autoSequenceTime; 			// Start time of step 4 when this step exits
									if (((autoSequenceTime - startLeftScaleRightTime[3]) > startLeftScaleRight[3])
									|| ((gyroAngleFB > (startLeftScaleRightFirstAngle - 3)) && (gyroAngleFB < 110))) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startLeftScaleRightSeq = 4; 					// Move to step 4
										}
								} // End of Step 3: Rotate CW 90 degrees to get behind switch
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 2: Move past Switch frame
							 ****************************************************/
							if (startLeftScaleRightSeq == 2) 									// Step 2: Move past Switch frame
								{
									System.out.println("Step 2 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleRightTime[2]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.85; 											// Assign open loop speed reference
									gyroAngleSP = 0.0; 											// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									if (rightMotorPosition > (startLeftScaleRightFirstPosition - 12000)) 
										{
											leftMotorShiftHigh.set(false); 						// Enable high speed gear on Left/Right Drive motors
											leftMotorShiftLow.set(true); 						// Enable high speed gear on Left/Right Drive motor
										}
				
									startLeftScaleRightTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startLeftScaleRightTime[2]) > startLeftScaleRight[2]
									|| (rightMotorPosition > startLeftScaleRightFirstPosition)) 
										{
											startLeftScaleRightSeq = 3;
										}
								} // End of Step 2: Move past Switch frame
				
							/****************************************************
							 * START LEFT - SCALE RIGHT 
							 * Step 1: Close intake arms solenoid, shift to high speed
							 ****************************************************/
							if ((startLeftScaleRightSeq == 1)) 					// Step 1: Close intake arms solenoid, shift to high speed
								{
									System.out.println("Step 1 : " + autoSequenceTime);
									
									intakeArmOpenCommand = false; 				// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 	// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand);	// Intake arms CLOSE solenoid
									
									leftMotorShiftHigh.set(true); 				// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(false); 				// Enable high speed gear on Left/Right Drive motor
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 							// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 	// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 										// Right half of compass
										firstGyroScan = gyroWithDrift; 			// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startLeftScaleRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startLeftScaleRightTime[1]) > startLeftScaleRight[1]) 
										{
											startLeftScaleRightSeq = 2; // Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid, shift to high speed
				
							} // End of Go for RIGHT SCALE starting from LEFT SIDE
			
					else 
						{
							autonomousSelected = "default"; // Run default Autonomous mode of just driving forward
						}
		
				} // End of robot starts on left
		
			/***************************************************************************************************************
			 * 
			 * CENTER CENTER CENTER CENTER CENTER CENTER CENTER CENTER CENTER CENTER
			 * 
			 ***************************************************************************************************************/
			if (matchStartPosition == 1) // Robot start position is center (left bumper is aligned with right wall of exchange zone box of back wall
				{
					/**************************************************************
					 * Robot START=CENTER Side, Go for LEFT SWITCH 
					 * system clock is in milliseconds
					 **************************************************************/
					if (autonomousSelected.equals("startCenterSwitchLeft")) // Go for Left switch starting from Center
						{
							startCenterSwitchLeft[0] = 0; 					// Step 0: Start of sequence
							startCenterSwitchLeft[1] = 50; 					// Step 1: Close intake arms solenoid
							startCenterSwitchLeft[2] = 950; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startCenterSwitchLeft[3] = 1000; 				// Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
							startCenterSwitchLeft[4] = 1400; 				// Step 4: Use gyro correction to move to correct angle
							startCenterSwitchLeft[5] = 3000; 				// Step 5 Rotate CW to 0 degrees toward Switch
							startCenterSwitchLeft[6] = 2800; 				// Step 6: Start moving forward toward switch 1050
							startCenterSwitchLeft[7] = 600; 				// Step 7: Stop forward movement and Open Arms 400
							startCenterSwitchLeft[8] = 2600; 				// Step 8: Back up robot away from switch 1200
							startCenterSwitchLeft[9] = 500; 				// Step 9: Move elevator down
							startCenterSwitchLeft[10] = 2600; 				// Step 10: Start moving forward toward cube in pile
							startCenterSwitchLeft[11] = 100; 				// Step 11: Close intake arms
							startCenterSwitchLeft[12] = 2500; 				// Step 12: Back up with cube to switch drop off point
							startCenterSwitchLeft[13] = 0; 					// Step 13: Rotate CW to 0 degrees to face switch
							startCenterSwitchLeft[14] = 2200; 				// Step 14: Start moving forward toward switch
							startCenterSwitchLeft[15] = 600; 				// Step 15: Stop forward movement and Open Arms
							startCenterSwitchLeft[16] = 2500; 				// Step 16: Back up robot away from switch
							startCenterSwitchLeft[17] = 14900; 				// Autonomous movement is done
							startCenterSwitchLeft[18] = 14900; 				// Autonomous movement is done
							startCenterSwitchLeftFirstPosition = 3500; 		// Encoder counts - first move forward
							startCenterSwitchLeftFirstAngle = 305; 			// Gyro angle for first turn toward switch
							startCenterSwitchLeftSecondPosition = 32000; 	// Encoder counts - second move forward
							startCenterSwitchLeftIntakeSpdRef = 0.4; 		// Intake motor speed ref to spit cube out
							startCenterSwitchLeftThirdPosition = 36000; 	// Encoder counts - third move forward
							startCenterSwitchLeftSecondAngle = 0; 			// Gyro angle for second turn toward switch
							startCenterSwitchLeftForthPosition = -14500; 	// Encoder counts - forth move backwards
							startCenterSwitchLeftForthAngle = 50; 			// Gyro angle for forth turn away switch
							startCenterSwitchLeftIntakeGrabRef = -0.8; 		// Intake motor speed ref to grab cube in
							double startCenterSwitchLeftIntakeHoldRef = -0.35; // To make sure we hold the cube.
							startCenterSwitchLeftFifthPosition = -7000; 	// Encoder counts - fifth move forward to get cube
							startCenterSwitchLeftFifthAngle = 43; 			// Gyro angle for fifth turn to move along back of switch
							startCenterSwitchLeftSixthPosition = -13500; 	// Encoder counts - sixth move backwards
							startCenterSwitchLeftSixthAngle = 0; 			// Gyro angle for sixth turn to move along back of switch
							startCenterSwitchLeftSeventhAngle = 0; 			// Gyro angle for seventh turn to move toward switch
							startCenterSwitchLeftSeventhPosition = 1000; 	// Encoder counts - reverse from getting the cube
							startCenterSwitchLeftEighthAngle = 0; 			// Gyro angle for eighth turn to move toward switch
							startCenterSwitchLeftEighthPosition = -8000; 	// Encoder counts - eighth move forward toward switch
							startCenterSwitchLeftNinthAngle = 75; 			// Gyro angle for ninth turn to move toward switch
							startCenterSwitchLeftNinthPosition = 0; 		// Encoder counts - ninth move forward toward switch
							/****************************************************
							 * Step 0 of start on CENTER, SWITCH on Left INIT
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startCenterSwitchLeftSeq = 1; 								// Begin step 1
									startCenterSwitchLeftTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 17: Move elevator down, close intake arms
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 17) 								// Step 17: Move elevator down, close intake arms
								{
									System.out.println("Step 17 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[17]));
									
									leftMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
									intakeArmClose.set(true); 									// Intake arms CLOSE solenoid
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef); 	// Start magic motion for absolute position  to move to SWITCH
				
									startCenterSwitchLeftTime[18] = autoSequenceTime; 			// Start time of step 10 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[17]) > startCenterSwitchLeft[17])) 
										{
											startCenterSwitchLeftSeq = 18; 						// Move to step 18
										}
				
								} // End of Step 17: Move elevator down, close intake arms
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 16: Back up robot away from switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 16) // Step 8: Back up robot away from switch
								{
									System.out.println("Step 16 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[16]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition);
									yAxisRef = -0.45; 											// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftForthAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.18)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0); 				// Right side motor speed reference
				
									leftIntake.set(0.0); 										// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 										// Assign speed ref to Right PWM Talon
				
									startCenterSwitchLeftTime[17] = autoSequenceTime; 			// Start time of step 17 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[16]) > startCenterSwitchLeft[16]) // end on time
									|| ((rightMotorPosition < startCenterSwitchLeftEighthPosition)))	// end on position
									//&& (gyroAngleFB < (gyroAngleSP + 5)) && (gyroAngleFB < 100))) //end on angle
									{
										leftMotorSpdRefAut = 0; 								// Stop motion
										rightMotorSpdRefAut = 0; 								// Stop motion
										leftIntake.set(0.0);
										rightIntake.set(0.0);
										startCenterSwitchLeftSeq = 17; 							// Move to step 17
									}
								} // End of Step 16: Back up robot away from switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 15: Stop forward movement and Open Arms
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 15) // Step 15: Stop forward movement and Open Arms
								{
									System.out.println("Step 15 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[15]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startCenterSwitchLeftIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchLeftIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startCenterSwitchLeftTime[16] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[15]) > startCenterSwitchLeft[15])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startCenterSwitchLeftSeq = 16; 								// Move to step 16
										}
				
								} // End of Step 15: Stop forward movement and Open Arms
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 14: Start moving forward toward switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 14) // Step 14: Start moving forward toward switch
								{
									System.out.println("Step 14 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[14]) + " , Gyro: " + gyroAngleFB + "Pos: " + rightMotorPosition);
									yAxisRef = 0.38; 											// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftSeventhAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.05); 				// Right side motor speed reference
				
									leftIntake.set(0.0);
									rightIntake.set(0.0);
									
									startCenterSwitchLeftTime[15] = autoSequenceTime; 			// Start time of step 15 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[14]) > startCenterSwitchLeft[14])
									|| (rightMotorPosition > startCenterSwitchLeftSeventhPosition)) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startCenterSwitchLeftSeq = 15; 						// Move to step 15
										}
								} // End of Step 14: Start moving forward toward switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 13: Rotate CCW to 0 degrees to face switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 13) 									// Step 13: Rotate CW to 0 degrees to face switch
								{
									System.out.println("Step 13 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[13]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition);
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftSeventhAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.22); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.22); 					// Right side motor speed reference
				
									startCenterSwitchLeftTime[14] = autoSequenceTime; 				// Start time of step 14 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[13]) > startCenterSwitchLeft[13]) // end on time
									|| (gyroAngleFB < (gyroAngleSP + 3)) || (gyroAngleFB > 357)) // end on angle //end on angle
										{
											startCenterSwitchLeftSeq = 14; 								// Move to step 14
										}
								} // End of Step 13: Rotate CW to 0 degrees to face switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 12: Back up with cube to switch drop off point
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 12) 							// Step 12: Back up with cube to switch drop off point
								{
									System.out.println("Step 12 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[12]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition); 
									yAxisRef = -0.6; 										// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftNinthAngle;			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									leftIntake.set(startCenterSwitchLeftIntakeHoldRef); 	// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchLeftIntakeHoldRef); 	// Assign speed ref to Right PWM Talon
				
									if (rightMotorPosition < -2000)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);// Start magic motion for absolute position to move to SWITCH
				
									startCenterSwitchLeftTime[13] = autoSequenceTime; 		// Start time of step 13 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[12]) > startCenterSwitchLeft[12]) // end on time
									|| (rightMotorPosition < startCenterSwitchLeftSixthPosition)) // end on position
										{
											startCenterSwitchLeftSeq = 13; 					// Move to step 13
										}
								} // End of Step 12: Back up with cube to switch drop off point
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 11: Close intake arms
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 11) 							// Step 11: Close intake arms
								{
									System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[11]));
									yAxisRef = 0.0;
									gyroAngleSP = startCenterSwitchLeftSixthAngle;
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									//intakeArmOpenCommand = false; 						// Intake arms Closed command
									//intakeArmOpen.set(intakeArmOpenCommand); 				// Intake arms OPEN solenoid
									//intakeArmClose.set(!intakeArmOpenCommand); 			// Intake arms CLOSE solenoid
									
									leftIntake.set(startCenterSwitchLeftIntakeHoldRef); 	// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchLeftIntakeHoldRef); 	// Assign speed ref to Left PWM Talon
									
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
				
									startCenterSwitchLeftTime[12] = autoSequenceTime; 		// Start time of step 12 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[11]) > startCenterSwitchLeft[11])) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startCenterSwitchLeftSeq = 12; 					// Move to step 12
										}
				
								} // End of Step 11: Close intake arms
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 10: Start moving forward toward cube in pile
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 10) 							// Step 10: Start moving forward toward cube in pile
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[10]) + " , Gyro: " + gyroAngleFB + "Pos: " + rightMotorPosition);
				
									leftIntake.set(startCenterSwitchLeftIntakeGrabRef); 	// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchLeftIntakeGrabRef); 	// Assign speed ref to Left PWM Talon
				
									yAxisRef = 0.30; 										// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftFifthAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.15 +  gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									startCenterSwitchLeftTime[11] = autoSequenceTime; 		// Start time of step 11 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[10]) > startCenterSwitchLeft[10])
									|| (rightMotorPosition > startCenterSwitchLeftFifthPosition)) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											leftIntake.set(startCenterSwitchLeftIntakeHoldRef); // Assign speed ref to Left PWM Talon
											rightIntake.set(startCenterSwitchLeftIntakeHoldRef); // Assign speed ref to Left PWM Talon
											startCenterSwitchLeftSeq = 11; 					// Move to step 11
										}
								} // End of Step 10: Start moving forward toward cube in pile
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 9: Move elevator down
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 9) 								// Step 9: Move elevator down
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[9]));
									leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
									
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef); // Start magic motion for absolute position to move to SWITCH
				
									startCenterSwitchLeftTime[10] = autoSequenceTime; 		// Start time of step 10 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[9]) > startCenterSwitchLeft[9])) 
										{
											startCenterSwitchLeftSeq = 10; 					// Move to step 10
										}
				
								} // End of Step 9: Move elevator down
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 8: Back up robot away from switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 8) 								// Step 8: Back up robot away from switch
								{
									System.out.println("Step 8CL : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[8]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.65; 										// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftForthAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB);	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0); 			// Right side motor speed reference
				
									intakeArmOpenCommand = false; 							// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 				// Intake arms CLOSE solenoid
									intakeArmClose.set(!intakeArmOpenCommand);
									
									leftIntake.set(0.0); 									// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 									// Assign speed ref to Right PWM Talon
				
									startCenterSwitchLeftTime[9] = autoSequenceTime; 		// Start time of step 9 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[8]) > startCenterSwitchLeft[8]) // end on time
									|| (rightMotorPosition < startCenterSwitchLeftForthPosition)) // end on position
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startCenterSwitchLeftSeq = 9; 					// Move to step 9
										}
								} // End of Step 8: Back up robot away from switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 7: Stop forward movement and Open Arms
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 7) 								// Step 7: Stop forward movement and Open Arms
								{
									System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[7]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 								// Stop motion
									rightMotorSpdRefAut = 0; 								// Stop motion
				
									leftIntake.set(startCenterSwitchLeftIntakeSpdRef); 		// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchLeftIntakeSpdRef); 	// Assign speed ref to Right PWM Talon
				
									intakeArmOpen.set(true); 								// Intake arms OPEN solenoid
									intakeArmClose.set(false); 								// Intake arms CLOSE solenoid
				
									startCenterSwitchLeftTime[8] = autoSequenceTime; 		// Start time of step 8 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[7]) > startCenterSwitchLeft[7])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startCenterSwitchLeftSeq = 8; 					// Move to step 8
										}
				
								} // End of Step 7: Stop forward movement and Open Arms
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 6: Start moving forward toward switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 6) 								// Step 5: Start moving forward toward switch
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.40; 										// Assign open loop speed reference
									gyroAngleSP = 0.0; 										// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB);	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									startCenterSwitchLeftTime[7] = autoSequenceTime; 		// Start time of step 7 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[6]) > startCenterSwitchLeft[6])
									|| (rightMotorPosition > startCenterSwitchLeftThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startCenterSwitchLeftSeq = 7; 					// Move to step 7
										}
								} // End of Step 6: Start moving forward toward switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 5: Rotate CW to 0 degrees toward Switch
							 ****************************************************/
							if ((startCenterSwitchLeftSeq == 5)) 							// Step 5: Rotate CW to 0 degrees toward Switch
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[5]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.0; 										// Assign open loop speed reference
									gyroAngleSP = 0.0; 										// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.32); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.22); 			// Right side motor speed reference
				
									startCenterSwitchLeftTime[6] = autoSequenceTime;
									if ((autoSequenceTime - startCenterSwitchLeftTime[5]) > startCenterSwitchLeft[5]
									|| ((gyroAngleFB > (356.0)) || (gyroAngleFB < 2.0))) 
										{
											leftMotorSpdRefAut = 0; 						// Stop motion
											rightMotorSpdRefAut = 0; 						// Stop motion
											startCenterSwitchLeftSeq = 6; 					// Move to step 6
										}
								} // End of Step 5 Rotate CW to 0 degrees toward Switch
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 4: Use gyro correction to move to correct
							 * angle closer to the Switch
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 4) 								// Step 4: Use gyro correction to move to correct angle
								{
									System.out.println("Step 4 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[4]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.55; 										// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftFirstAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 			// Right side motor speed reference
				
									startCenterSwitchLeftTime[5] = autoSequenceTime; 		// Start time of step 5 when this step exits
									if ((autoSequenceTime - startCenterSwitchLeftTime[4]) > startCenterSwitchLeft[4]
									|| ((rightMotorPosition > startCenterSwitchLeftSecondPosition))) 
										{
											startCenterSwitchLeftSeq = 5; 					// Move to step 5
										}
								}	// End of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 3: Start Decel to min speed, Start rotate CCW
							 * to 305 Deg during decel
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 3) 								// Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[3]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.24; 										// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchLeftFirstAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.28); 			// Right side motor speed reference
				
									startCenterSwitchLeftTime[4] = autoSequenceTime; 		// Start time of step 4 when this step exits
									if (((autoSequenceTime - startCenterSwitchLeftTime[3]) > startCenterSwitchLeft[3])
									|| ((gyroAngleFB < (gyroAngleSP + 5)) && (gyroAngleFB > 200))) 
										{
											startCenterSwitchLeftSeq = 4; 					// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 315 Deg during decel
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 2: Drop intake arms, start moving forward to
							 * LEFT SCALE position, Raise elevator
							 ****************************************************/
							if (startCenterSwitchLeftSeq == 2) 								// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + " , " + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									intakeArmPivotDownCommand = true; 						// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 		// Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false); 							// Enable high speed gear on Left/Left Drive motors
									leftMotorShiftLow.set(true); 							// Disable high speed gear on Left/Left Drive motors
				
									yAxisRef = 0.35; 										// Assign open loop speed reference
									gyroAngleSP = 0.0; 										// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 				// Right side motor speed reference
				
									if (rightMotorPosition > 2800)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); // Start magic motion for absolute position to move to SWITCH
				
									startCenterSwitchLeftTime[3] = autoSequenceTime; 		// Start time of step 3 when this step exits
									if ((autoSequenceTime - startCenterSwitchLeftTime[2]) > startCenterSwitchLeft[2]
									|| (rightMotorPosition > startCenterSwitchLeftFirstPosition)) 
										{
											startCenterSwitchLeftSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START CENTER - SWITCH LEFT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startCenterSwitchLeftSeq == 1)) 							// Step 1: Close intake arms solenoid
							{
								System.out.println("Step 1 : " + autoSequenceTime);
								intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
								intakeArmClose.set(true);									// Intake arms CLOSE solenoid
			
								// Snapshot gyro start angle longer than 1 scan, init scan isn't working
								if (gyroYaw < 0) 											// Left half of compass
									firstGyroScan = 180 + (180 + gyroYaw); 					// gyro raw -179 to 0 = gyro compass angle +181 to 359
								else 														// Right half of compass
									firstGyroScan = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
			
								startCenterSwitchLeftTime[2] = autoSequenceTime;
								if ((autoSequenceTime - startCenterSwitchLeftTime[1]) > startCenterSwitchLeft[1]) 
									{
										startCenterSwitchLeftSeq = 2; // Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
									}
							} // End of Step 1: Close intake arms solenoid
			
						} // End of START CENTER, go for SWITCH LEFT
		
					/**************************************************************
					 * Robot START=CENTER Side, Go for RIGHT SWITCH 
					 * system clock is in milliseconds
					 **************************************************************/
					else if (autonomousSelected.equals("startCenterSwitchRight")) // Go for Left switch starting from Left side if (testStartLeftSwitchLeft)
						{
							startCenterSwitchRight[0] = 0; 					// Step 0: Start of sequence
							startCenterSwitchRight[1] = 50; 				// Step 1: Close intake arms solenoid
							startCenterSwitchRight[2] = 950; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startCenterSwitchRight[3] = 1000; 				// Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
							startCenterSwitchRight[4] = 1400; 				// Step 4: Use gyro correction to move to correct angle
							startCenterSwitchRight[5] = 3000; 				// Step 5 Rotate CW to 0 degrees toward Switch
							startCenterSwitchRight[6] = 2800; 				// Step 6: Start moving forward toward switch 1050
							startCenterSwitchRight[7] = 600; 				// Step 7: Stop forward movement and Open Arms 400
							startCenterSwitchRight[8] = 2600; 				// Step 8: Back up robot away from switch 1200
							startCenterSwitchRight[9] = 500; 				// Step 9: Move elevator down
							startCenterSwitchRight[10] = 3200; 				// Step 10: Start moving forward toward cube in pile
							startCenterSwitchRight[11] = 100; 				// Step 11: Close intake arms
							startCenterSwitchRight[12] = 2500; 				// Step 12: Back up with cube to switch drop off point
							startCenterSwitchRight[13] = 0; 				// Step 13: Rotate CW to 0 degrees to face switch
							startCenterSwitchRight[14] = 2000; 				// Step 14: Start moving forward toward switch
							startCenterSwitchRight[15] = 600; 				// Step 15: Stop forward movement and Open Arms
							startCenterSwitchRight[16] = 1100; 				// Step 16: Back up robot away from switch
							startCenterSwitchRight[17] = 14900; 			// Autonomous movement is done
							startCenterSwitchRight[18] = 14900; 			// Autonomous movement is done
							startCenterSwitchRightFirstPosition = 3500; 	// Encoder counts - first move forward
							startCenterSwitchRightFirstAngle = 50; 			// Gyro angle for first turn toward switch
							startCenterSwitchRightSecondPosition = 23000; 	// Encoder counts - second move forward
							startCenterSwitchRightIntakeSpdRef = 0.4; 		// Intake motor speed ref to spit cube out
							startCenterSwitchRightThirdPosition = 35000; 	// Encoder counts - third move forward
							startCenterSwitchRightSecondAngle = 0; 			// Gyro angle for second turn toward switch
							startCenterSwitchRightForthPosition = -12900; 	// Encoder counts - forth move backwards
							startCenterSwitchRightForthAngle = 50; 			// Gyro angle for forth turn away switch
							startCenterSwitchRightIntakeGrabRef = -0.8; 	// Intake motor speed ref to grab cube in
							double startCenterSwitchRightIntakeHoldRef = -0.35; // To make sure we hold the cube.
							startCenterSwitchRightFifthPosition = -3000; 	// Encoder counts - fifth move forward to get cube
							startCenterSwitchRightFifthAngle = 315; 		// Gyro angle for fifth turn to move along back of switch
							startCenterSwitchRightSixthPosition = -16000; 	// Encoder counts - sixth move backwards
							startCenterSwitchRightSixthAngle = 310; 		// Gyro angle for sixth turn to move along back of switch
							startCenterSwitchRightSeventhAngle = 0; 		// Gyro angle for seventh turn to move toward switch
							startCenterSwitchRightSeventhPosition = -2000; 	// Encoder counts - reverse from getting the cube
							startCenterSwitchRightEighthAngle = 0; 			// Gyro angle for eighth turn to move toward switch
							startCenterSwitchRightEighthPosition = -8000; 	// Encoder counts - eighth move forward toward switch
							/****************************************************
							 * Step 0 of start on CENTER, SWITCH on RIGHT INIT
							 ****************************************************/
							if (startCenterSwitchRightSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startCenterSwitchRightSeq = 1; 								// Begin step 1
									startCenterSwitchRightTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 17: Move elevator down, close intake arms
							 ****************************************************/
							if (startCenterSwitchRightSeq == 17) 								// Step 17: Move elevator down, close intake arms
								{
									System.out.println("Step 17 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[17]));
									
									leftMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
									intakeArmClose.set(true); 									// Intake arms CLOSE solenoid
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef); 	// Start magic motion for absolute position  to move to SWITCH
				
									startCenterSwitchRightTime[18] = autoSequenceTime; 			// Start time of step 10 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[17]) > startCenterSwitchRight[17])) 
										{
											startCenterSwitchRightSeq = 18; 					// Move to step 18
										}
				
								} // End of Step 17: Move elevator down, close intake arms
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 16: Back up robot away from switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 16) // Step 8: Back up robot away from switch
								{
									System.out.println("Step 16 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[16]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition);
									yAxisRef = -0.40; 											// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightSixthAngle; 			// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.18)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									leftIntake.set(0.0); 										// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 										// Assign speed ref to Right PWM Talon
				
									startCenterSwitchRightTime[17] = autoSequenceTime; 			// Start time of step 17 when this step exits
									if ( ((autoSequenceTime - startCenterSwitchRightTime[16]) > startCenterSwitchRight[16]) // end on time
									|| ((rightMotorPosition < startCenterSwitchRightEighthPosition))	// end on position
									&& (gyroAngleFB < (gyroAngleSP + 5)) && (gyroAngleFB > 200) ) //end on angle
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftIntake.set(0.0);
											rightIntake.set(0.0);
											startCenterSwitchRightSeq = 17; 						// Move to step 17
										}
								} // End of Step 16: Back up robot away from switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 15: Stop forward movement and Open Arms
							 ****************************************************/
							if (startCenterSwitchRightSeq == 15) // Step 15: Stop forward movement and Open Arms
								{
									System.out.println("Step 15 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[15]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startCenterSwitchRightIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchRightIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startCenterSwitchRightTime[16] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[15]) > startCenterSwitchRight[15])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startCenterSwitchRightSeq = 16; 						// Move to step 16
										}
				
								} // End of Step 15: Stop forward movement and Open Arms
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 14: Start moving forward toward switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 14) // Step 14: Start moving forward toward switch
								{
									System.out.println("Step 14 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[14]) + " , Gyro: " + gyroAngleFB + "Pos: " + rightMotorPosition);
									yAxisRef = 0.38; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightSeventhAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.15 + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									leftIntake.set(0.0);
									rightIntake.set(0.0);
									
									startCenterSwitchRightTime[15] = autoSequenceTime; 				// Start time of step 15 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[14]) > startCenterSwitchRight[14])
									|| (rightMotorPosition > startCenterSwitchRightSeventhPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterSwitchRightSeq = 15; 						// Move to step 15
										}
								} // End of Step 14: Start moving forward toward switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 13: Rotate CW to 0 degrees to face switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 13) 									// Step 13: Rotate CW to 0 degrees to face switch
								{
									System.out.println("Step 13 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[13]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition);
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightSeventhAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef - 0.22); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.22); 					// Right side motor speed reference
				
									startCenterSwitchRightTime[14] = autoSequenceTime; 				// Start time of step 14 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[13]) > startCenterSwitchRight[13]) // end on time
									|| (gyroAngleFB < (gyroAngleSP + 5)) || (gyroAngleFB > 357)) 	// end on angle
										{
											startCenterSwitchRightSeq = 14; 						// Move to step 14
										}
								} // End of Step 13: Rotate CW to 0 degrees to face switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 12: Back up with cube to switch drop off point
							 ****************************************************/
							if (startCenterSwitchRightSeq == 12) 									// Step 12: Back up with cube to switch drop off point
								{
									System.out.println("Step 12 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[12]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition); 
									yAxisRef = -0.6; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightFifthAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									leftIntake.set(startCenterSwitchRightIntakeHoldRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchRightIntakeHoldRef); 			// Assign speed ref to Right PWM Talon
				
									if (rightMotorPosition < -2000)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);	// Start magic motion for absolute position to move to SWITCH
				
									startCenterSwitchRightTime[13] = autoSequenceTime; 				// Start time of step 13 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[12]) > startCenterSwitchRight[12]) // end on time
									|| (rightMotorPosition < startCenterSwitchRightSixthPosition)) 	// end on position
										{
											startCenterSwitchRightSeq = 13; 						// Move to step 13
										}
								} // End of Step 12: Back up with cube to switch drop off point
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 11: Close intake arms
							 ****************************************************/
							if (startCenterSwitchRightSeq == 11) 									// Step 11: Close intake arms
								{
									System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[11]));
									yAxisRef = 0.0;
									gyroAngleSP = startCenterSwitchRightSixthAngle;
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									//intakeArmOpenCommand = false; 								// Intake arms Closed command
									//intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									//intakeArmClose.set(!intakeArmOpenCommand); 					// Intake arms CLOSE solenoid
									
									leftIntake.set(startCenterSwitchRightIntakeHoldRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchRightIntakeHoldRef); 			// Assign speed ref to Left PWM Talon
									
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 		// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
				
									startCenterSwitchRightTime[12] = autoSequenceTime; 				// Start time of step 12 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[11]) > startCenterSwitchRight[11])) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterSwitchRightSeq = 12; 						// Move to step 12
										}
				
								} // End of Step 11: Close intake arms
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 10: Start moving forward toward cube in pile
							 ****************************************************/
							if (startCenterSwitchRightSeq == 10) 									// Step 10: Start moving forward toward cube in pile
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[10]) + " , Gyro: " + gyroAngleFB + "Pos: " + rightMotorPosition);
				
									leftIntake.set(startCenterSwitchRightIntakeGrabRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchRightIntakeGrabRef); 			// Assign speed ref to Left PWM Talon
				
									yAxisRef = 0.30; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightFifthAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterSwitchRightTime[11] = autoSequenceTime; 				// Start time of step 11 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[10]) > startCenterSwitchRight[10])
									|| (rightMotorPosition > startCenterSwitchRightFifthPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftIntake.set(startCenterSwitchRightIntakeHoldRef); 	// Assign speed ref to Left PWM Talon
											rightIntake.set(startCenterSwitchRightIntakeHoldRef); 	// Assign speed ref to Left PWM Talon
											startCenterSwitchRightSeq = 11; 						// Move to step 11
										}
								} // End of Step 10: Start moving forward toward cube in pile
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 9: Move elevator down
							 ****************************************************/
							if (startCenterSwitchRightSeq == 9) 									// Step 9: Move elevator down
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[9]));
									leftMotorSpdRefAut = 0.0; 										// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0; 										// Assign open loop speed reference
									
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef); 		// Start magic motion for absolute position to move to SWITCH
				
									startCenterSwitchRightTime[10] = autoSequenceTime; 				// Start time of step 10 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[9]) > startCenterSwitchRight[9])) 
										{
											startCenterSwitchRightSeq = 10; 						// Move to step 10
										}
				
								} // End of Step 9: Move elevator down
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 8: Back up robot away from switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 8) 									// Step 8: Back up robot away from switch
								{
									System.out.println("Step 8CL : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[8]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.65; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightForthAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 						// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0); 					// Right side motor speed reference
				
									intakeArmOpenCommand = false; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
									intakeArmClose.set(!intakeArmOpenCommand);
									
									leftIntake.set(0.0); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 											// Assign speed ref to Right PWM Talon
				
									startCenterSwitchRightTime[9] = autoSequenceTime; 				// Start time of step 9 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[8]) > startCenterSwitchRight[8]) // end on time
									|| (rightMotorPosition < startCenterSwitchRightForthPosition)) 	// end on position
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterSwitchRightSeq = 9; 							// Move to step 9
										}
								} // End of Step 8: Back up robot away from switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 7: Stop forward movement and Open Arms
							 ****************************************************/
							if (startCenterSwitchRightSeq == 7) 									// Step 7: Stop forward movement and Open Arms
								{
									System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[7]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startCenterSwitchRightIntakeSpdRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startCenterSwitchRightIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
				
									intakeArmOpen.set(true); 										// Intake arms OPEN solenoid
									intakeArmClose.set(false); 										// Intake arms CLOSE solenoid
				
									startCenterSwitchRightTime[8] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[7]) > startCenterSwitchRight[7])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startCenterSwitchRightSeq = 8; 							// Move to step 8
										}
				
								} // End of Step 7: Stop forward movement and Open Arms
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 6: Start moving forward toward switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 6) 									// Step 5: Start moving forward toward switch
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.40; 												// Assign open loop speed reference
									gyroAngleSP = 0.0; 												// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterSwitchRightTime[7] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[6]) > startCenterSwitchRight[6])
									|| (rightMotorPosition > startCenterSwitchRightThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterSwitchRightSeq = 7; 							// Move to step 7
										}
								} // End of Step 6: Start moving forward toward switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 5: Rotate CW to 0 degrees toward Switch
							 ****************************************************/
							if ((startCenterSwitchRightSeq == 5)) 									// Step 5: Rotate CW to 0 degrees toward Switch
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[5]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = 0.0; 												// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef - 0.22); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.32); 					// Right side motor speed reference
				
									startCenterSwitchRightTime[6] = autoSequenceTime;
									if ((autoSequenceTime - startCenterSwitchRightTime[5]) > startCenterSwitchRight[5]
									|| (gyroAngleFB < (gyroAngleSP + 5)) || (gyroAngleFB > 358)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterSwitchRightSeq = 6; 							// Move to step 6
										}
								} // End of Step 5 Rotate CW to 0 degrees toward Switch
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 4: Use gyro correction to move to correct
							 * angle closer to the Switch
							 ****************************************************/
							if (startCenterSwitchRightSeq == 4) 									// Step 4: Use gyro correction to move to correct angle
							{
								System.out.println("Step 4 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[4]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.55; 													// Assign open loop speed reference
								gyroAngleSP = startCenterSwitchRightFirstAngle; 					// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 				// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 						// Right side motor speed reference
			
								startCenterSwitchRightTime[5] = autoSequenceTime; // Start time of step 5 when this step exits
								if ((autoSequenceTime - startCenterSwitchRightTime[4]) > startCenterSwitchRight[4]
								|| ((rightMotorPosition > startCenterSwitchRightSecondPosition))) 
									{
										startCenterSwitchRightSeq = 5; 								// Move to step 5
									}
							}	// End of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 3: Start Decel to min speed, Start rotate CW
							 * to 45 Deg during decel
							 ****************************************************/
							if (startCenterSwitchRightSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[3]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.24; 												// Assign open loop speed reference
									gyroAngleSP = startCenterSwitchRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.28); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterSwitchRightTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startCenterSwitchRightTime[3]) > startCenterSwitchRight[3])
									|| ((gyroAngleFB > (gyroAngleSP - 5)) && (gyroAngleFB < 100))) 
										{
											startCenterSwitchRightSeq = 4; 							// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 315 Deg during decel
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 2: Drop intake arms, start moving forward to
							 * LEFT SCALE position, Raise elevator
							 ****************************************************/
							if (startCenterSwitchRightSeq == 2) 									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + " , " + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									intakeArmPivotDownCommand = true; 								// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 				// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 				// Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false); 									// Enable high speed gear on Left/Left Drive motors
									leftMotorShiftLow.set(true); 									// Disable high speed gear on Left/Left Drive motors
				
									yAxisRef = 0.35; 												// Assign open loop speed reference
									gyroAngleSP = 0.0; 												// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									if (rightMotorPosition > 2800)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); 	// Start magic motion for absolute
																									// position to move to SWITCH
				
									startCenterSwitchRightTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startCenterSwitchRightTime[2]) > startCenterSwitchRight[2]
									|| (rightMotorPosition > startCenterSwitchRightFirstPosition)) 
										{
											startCenterSwitchRightSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START CENTER - SWITCH RIGHT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startCenterSwitchRightSeq == 1)) 									// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
									intakeArmOpen.set(false); 										// Intake arms OPEN solenoid
									intakeArmClose.set(true); 										// Intake arms CLOSE solenoid
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 												// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 						// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 															// Right half of compass
										firstGyroScan = gyroWithDrift; 								// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startCenterSwitchRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startCenterSwitchRightTime[1]) > startCenterSwitchRight[1]) 
										{
											startCenterSwitchRightSeq = 2; // Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
			
						} // End of START CENTER, go for SWITCH RIGHT
		
					else 
						{
							autonomousSelected = "default"; // Run default Autonomous mode of just driving forward
						}
		
				} // End of robot starts in center
	
			/***************************************************************************************************************
			 * 
			 * RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT
			 * 
			 ***************************************************************************************************************/
			if (matchStartPosition == 2) // Robot start position is right (right bumper is aligned with right point of straight part of back wall
				{
					/**************************************************************
					 * Robot Start=Right Side, Go for Right Switch system clock is in milliseconds
					 **************************************************************/
					if (autonomousSelected.equals("startRightSwitchRight")) // Go for Right switch starting from Right side

						{
							startRightSwitchRight[0] = 0; 				// Step 0: Start of sequence
							startRightSwitchRight[1] = 50; 				// Step 1: Close intake arms solenoid
							startRightSwitchRight[2] = 2850; 			// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startRightSwitchRight[3] = 2550; 			// Step 3: Start Decel to min speed, Start rotate CCW to 270 Deg during decel
							startRightSwitchRight[4] = 400; 			// Step 4: Use gyro correction to move to correct angle
							startRightSwitchRight[5] = 100; 			// Step 5: Reset Drive encoders to zero
							startRightSwitchRight[6] = 2800; 			// Step 6: Start moving forward toward switch 1050
							startRightSwitchRight[7] = 800; 			// Step 7: Stop forward movement and Open Arms 400
							startRightSwitchRight[8] = 1200; 			// Step 8: Back up robot away from switch 1200
							startRightSwitchRight[9] = 500; 			// Step 9: Move elevator down, close intake arms 500
							startRightSwitchRight[10] = 0; 				// Autonomous movement
							startRightSwitchRight[11] = 0;	 			// Autonomous movement
							startRightSwitchRight[12] = 0; 				// Autonomous movement
							startRightSwitchRight[13] = 0; 				// Autonomous movement
							startRightSwitchRight[14] = 14900; 			// Autonomous movement is done
							startRightSwitchRightFirstPosition = 35500; // Encoder counts - first move forward
							startRightSwitchRightFirstAngle = 275; 		// Gyro angle for first turn toward switch
							startRightSwitchRightSecondPosition = 8000; // Encoder counts - second move forward
							startRightSwitchRightIntakeSpdRef = -0.5; 	// Intake motor speed ref to spit cube out
							startRightSwitchRightThirdPosition = -4000; // Encoder counts - third move backward
							startRightSwitchRightSecondAngle = 272; 	// Gyro angle for second turn toward switch
							/****************************************************
							 * Step 0 of start on Right, Switch on Right INIT
							 ****************************************************/
							if (startRightSwitchRightSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startRightSwitchRightSeq = 1; 								// Begin step 1
									startRightSwitchRightTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 9: Move elevator down, close intake arms
							 ****************************************************/
							if (startRightSwitchRightSeq == 9) // Step 9: Move elevator down, close intake arms
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[9]));
									leftMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
									rightMotorSpdRefAut = 0.0;	 								// Assign open loop speed reference
									intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
									intakeArmClose.set(true); 									// Intake arms CLOSE solenoid
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef); 	// Start magic motion for absolute position to move to SWITCH
				
									startRightSwitchRightTime[10] = autoSequenceTime; 			// Start time of step 8 when this step exits
									if (((autoSequenceTime - startRightSwitchRightTime[9]) > startRightSwitchRight[9])) 
										{
											startRightSwitchRightSeq = 10; 						// Move to step 10
										}
				
								} // End of Step 9: Move elevator down, close intake arms
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 8: Back up robot away from switch
							 ****************************************************/
							if (startRightSwitchRightSeq == 8) 									// Step 8: Back up robot away from switch
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[8]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = -0.25; 											// Assign open loop speed reference
									gyroAngleSP = startRightSwitchRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									leftIntake.set(0.0); 										// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 										// Assign speed ref to Right PWM Talon
				
									startRightSwitchRightTime[9] = autoSequenceTime; 			// Start time of step 9 when this step exits
									if (((autoSequenceTime - startRightSwitchRightTime[8]) > startRightSwitchRight[8])
									|| (rightMotorPosition < startRightSwitchRightThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightSwitchRightSeq = 9; 						// Move to step 9
										}
								} // End of Step 8: Back up robot away from switch
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 7: Stop forward movement and Open Arms
							 ****************************************************/
							if (startRightSwitchRightSeq == 7) 									// Step 7: Stop forward movement and Open Arms
							{
								System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[7]) + " , Gyro: " + gyroAngleFB);
								leftMotorSpdRefAut = 0; 										// Stop motion
								rightMotorSpdRefAut = 0; 										// Stop motion
			
								leftIntake.set(startRightSwitchRightIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
								rightIntake.set(startRightSwitchRightIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
			
								intakeArmOpen.set(true); 										// Intake arms OPEN solenoid
								intakeArmClose.set(false); 										// Intake arms CLOSE solenoid
			
								startRightSwitchRightTime[8] = autoSequenceTime; 				// Start time of step 6 when this step exits
								if (((autoSequenceTime - startRightSwitchRightTime[7]) > startRightSwitchRight[7])) 
									{
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startRightSwitchRightSeq = 8; 							// Move to step 8
									}
							} // End of Step 7: Stop forward movement and Open Arms
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 6: Start moving forward toward switch
							 ****************************************************/
							if (startRightSwitchRightSeq == 6) 									// Step 6: Start moving forward toward switch
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.35; 											// Assign open loop speed reference
									gyroAngleSP = startRightSwitchRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startRightSwitchRightTime[7] = autoSequenceTime; 			// Start time of step 7 when this step exits
									if (((autoSequenceTime - startRightSwitchRightTime[6]) > startRightSwitchRight[6])
									|| (rightMotorPosition > startRightSwitchRightSecondPosition)) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightSwitchRightSeq = 7; 						// Move to step 7
										}
								} // End of Step 5: Start moving forward toward switch
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 5: Reset Drive encoders to zero
							 ****************************************************/
							if ((startRightSwitchRightSeq == 5)) 								// Step 5: Reset Drive encoders to zero
								{
									System.out.println("Step 5 : " + (autoSequenceTime - startRightSwitchRightTime[5]));
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
				
									startRightSwitchRightTime[6] = autoSequenceTime;
									if ((autoSequenceTime - startRightSwitchRightTime[5]) > startRightSwitchRight[5]) 
										{
											startRightSwitchRightSeq = 6; 						// Move to step 6
										}
								} // End of Step 5 Reset Drive encoders to zero
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 4: Use gyro correction to move to correct angle
							 ****************************************************/
							if (startRightSwitchRightSeq == 4) 									// Step 4: Use gyro correction to move to correct angle
								{
									System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startRightSwitchRightTime[4]));
				
									yAxisRef = 0.0; 											// Assign open loop speed reference
									gyroAngleSP = startRightSwitchRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startRightSwitchRightTime[5] = autoSequenceTime; 			// Start time of step 3 when this step exits
									if ((autoSequenceTime - startRightSwitchRightTime[4]) > startRightSwitchRight[4]
									|| ((gyroAngleFB < startRightSwitchRightFirstAngle) && (gyroAngleFB > 200))) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightSwitchRightSeq = 5; 						// Move to step 5
										}
								} // End of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 3: Start Decel to min speed, Start rotate CCW
							 * to 270 Deg during decel
							 ****************************************************/
							if (startRightSwitchRightSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
								{
									System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightSwitchRightTime[3]));
									yAxisRef = 0.00; 											// Assign open loop speed reference
									gyroAngleSP = startRightSwitchRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - .18)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.18); 				// Right side motor speed reference
				
									startRightSwitchRightTime[4] = autoSequenceTime; 			// Start time of step 4 when this step exits
									if (((autoSequenceTime - startRightSwitchRightTime[3]) > startRightSwitchRight[3])
									|| ((gyroAngleFB < (startRightSwitchRightFirstAngle + 2)) && (gyroAngleFB > 200))) 
										{
											startRightSwitchRightSeq = 4; 						// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 2: Drop intake arms, start moving forward to
							 * position left of scale,start raising elevator
							 ****************************************************/
							if (startRightSwitchRightSeq == 2) 									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + "," + (autoSequenceTime - startRightSwitchRightTime[2]));
				
									intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false); 								// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 								// Disable high speed gear on Left/Right Drive motors
				
									yAxisRef = 0.58; 											// Assign open loop speed reference
									gyroAngleSP = 3.0; 											// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									if (rightMotorPosition > 12000)
										winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); // Start magic motion for absolute position to move to SWITCH
				
									startRightSwitchRightTime[3] = autoSequenceTime; 			// Start time of step 3 when this step exits
									if ((autoSequenceTime - startRightSwitchRightTime[2]) > startRightSwitchRight[2]
									|| (rightMotorPosition > startRightSwitchRightFirstPosition)) 
										{
											startRightSwitchRightSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START RIGHT - SWITCH RIGHT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startRightSwitchRightSeq == 1)) 								// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + (autoSequenceTime - startRightSwitchRightTime[1]));
									intakeArmOpen.set(false); 									// Intake arms OPEN solenoid
									intakeArmClose.set(true); 									// Intake arms CLOSE solenoid
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 											// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 					// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else // Right half of compass
										firstGyroScan = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startRightSwitchRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startRightSwitchRightTime[1]) > startRightSwitchRight[1]) 
										{
											startRightSwitchRightSeq = 2; 						// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
				
							} // End of Go for Right switch starting from Right side
		
					/**************************************************************
					 * Robot START=RIGHT Side, Go for RIGHT SCALE 
					 * system clock is in milliseconds
					 **************************************************************/
					else if (autonomousSelected.equals("startRightScaleRight")) // Go for Right scale starting from Right side
						{
							
							startRightScaleRight[0] = 0; 					// Step 0: Start of sequence
							startRightScaleRight[1] = 50; 					// Step 1: Close intake arms solenoid
							startRightScaleRight[2] = 3500; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startRightScaleRight[3] = 1550; 				// Step 3: Start Decel to min speed, Start rotate CCW to 330 Deg during decel
							startRightScaleRight[4] = 800; 					// Step 4: Drive close to switch
							startRightScaleRight[5] = 12800; 				// Step 5: Wait for Elevator to get up to height
							startRightScaleRight[6] = 2800; 				// Step 6: Start moving forward toward scale 1050
							startRightScaleRight[7] = 600; 					// Step 7: not used
							startRightScaleRight[8] = 300; 					// Step 8: Stop forward movement and Open Arms
							startRightScaleRight[9] = 800; 					// Step 9: Back up robot away from scale
							startRightScaleRight[10] = 3200; 				// Step 10: Move elevator down, close intake arms
							startRightScaleRight[11] = 2600; 				// Step 11: Move to 2nd cube
							startRightScaleRight[12] = 800; 				// Step 12: Back up with second cube
							startRightScaleRight[13] = 2000; 				// Step 13: Raise elevator up, rotate toward scale
							startRightScaleRight[14] = 1700; 				// Step 14: Start moving forward toward scale again
							startRightScaleRight[15] = 300; 				// Step 15: Stop forward movement and Open Arms
							startRightScaleRight[16] = 1800; 				// Step 16: Back up robot away from scale
							startRightScaleRight[17] = 1000; 				// Step 17: Drop Elevator to bottom
							startRightScaleRight[18] = 100; 				// Autonomous movement is done
							
							startRightScaleRightPosition[1] 	= 0;		//Position for 	Step 1: Close intake arms solenoid
							startRightScaleRightAngle[1] 		= 0;		//Angle for 	Step 1: Close intake arms solenoid
							startRightScaleRightPosition[2] 	= 52500;	//Position for 	Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startRightScaleRightAngle[2] 		= 0.0;		//Angle for 	Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startRightScaleRightPosition[3] 	= 0;		//Position for 	Step 3: Start Decel to min speed, Start rotate CCW to 330 Deg during decel
							startRightScaleRightAngle[3] 		= 345;		//Angle for 	Step 3: Start Decel to min speed, Start rotate CCW to 330 Deg during decel
							startRightScaleRightPosition[4] 	= 69000;	//Position for 	Step 4: Drive close to switch
							startRightScaleRightAngle[4] 		= 340;		//Angle for 	Step 4: Drive close to switch
							startRightScaleRightPosition[5] 	= 0;		//Position for 	Step 5: Wait for Elevator to get up to height
							startRightScaleRightAngle[5] 		= 0;		//Angle for 	Step 5: Wait for Elevator to get up to height
							startRightScaleRightPosition[6] 	= 4800;		//Position for 	Step 6: Start moving forward toward scale
							startRightScaleRightAngle[6] 		= 330;		//Angle for 	Step 6: Start moving forward toward scale
							startRightScaleRightPosition[7] 	= 0;		//Position for 	Step 7: Stop forward movement and Open Arms 400
							startRightScaleRightAngle[7] 		= 0.0;		//Angle for 	Step 7: Stop forward movement and Open Arms 400
							startRightScaleRightPosition[8] 	= 0;		//Position for 	Step 8: Stop forward movement and Open Arms
							startRightScaleRightAngle[8] 		= 0;		//Angle for 	Step 8: Stop forward movement and Open Arms
							startRightScaleRightPosition[9] 	= -3600;	//Position for 	Step 9: Back up robot away from scale
							startRightScaleRightAngle[9] 		= 230;		//Angle for 	Step 9: Back up robot away from scale
							startRightScaleRightPosition[10] 	= 0;		//Position for 	Step 10: Move elevator down, close intake arms
							startRightScaleRightAngle[10] 		= 213;		//Angle for 	Step 10: Move elevator down, close intake arms
							startRightScaleRightPosition[11] 	= 9800;		//Position for 	Step 11: Move to 2nd cube
							startRightScaleRightAngle[11] 		= 213;		//Angle for 	Step 11: Move to 2nd cube
							startRightScaleRightPosition[12] 	= 10200;	//Position for 	Step 12: Back up with second cube
							startRightScaleRightAngle[12] 		= 213;		//Angle for 	Step 12: Back up with second cube
							startRightScaleRightPosition[13] 	= 0;		//Position for 	Step 13: Raise elevator up, rotate toward scale
							startRightScaleRightAngle[13] 		= 345;		//Angle for 	Step 13: Raise elevator up, rotate toward scale
							startRightScaleRightPosition[14] 	= 9000;		//Position for 	Step 14: Start moving forward toward scale again
							startRightScaleRightAngle[14] 		= 348;		//Angle for 	Step 14: Start moving forward toward scale again
							startRightScaleRightPosition[15] 	= 0;		//Position for 	Step 15: Stop forward movement and Open Arms
							startRightScaleRightAngle[15] 		= 0;		//Angle for 	Step 15: Stop forward movement and Open Arms
							startRightScaleRightPosition[16] 	= -3300;	//Position for 	Step 16: Back up robot away from scale
							startRightScaleRightAngle[16] 		= 315;		//Angle for 	Step 16: Back up robot away from scale
							startRightScaleRightPosition[17] 	= 0;		//Position for 	Step 17: Drop Elevator to bottom
							startRightScaleRightAngle[17] 		= 0;		//Angle for 	Step 17: Drop Elevator to bottom
							
							startRightScaleRightIntakeSpdRef = 0.30; 		// Intake motor speed ref to spit cube out
							startRightScaleRightIntakeGrabRef = -0.9; 		// Intake motor speed ref to grab cube in	
							startLeftSwitchLeftIntakeHoldRef = -0.55; 		// To make sure we hold the cube.

							/****************************************************
							 * Step 0 of start on RIGHT, SCALE on RIGHT init
							 ****************************************************/
							if (startRightScaleRightSeq == 0) 	// time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startRightScaleRightSeq = 1; 								// Begin step 1
									startRightScaleRightTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 17: Drop Elevator to bottom
							 ****************************************************/
							if (startRightScaleRightSeq == 17) 										// Step 17: Drop Elevator to bottom
								{
									System.out.println("Step 17 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[17]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[17]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.25)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25); 					// Right side motor speed reference
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);		// Start magic motion for absolute position to move to floor
									
									startRightScaleRightTime[18] = autoSequenceTime; 				// Start time of step 18 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[17]) > startRightScaleRight[17])
									|| ((gyroAngleFB < (gyroAngleSP + 2)) && (gyroAngleFB > 150)) )
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startRightScaleRightSeq = 18; 							// Move to step 18
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 17: Drop Elevator to bottom
							
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 16: Back up robot away from scale
							 ****************************************************/
							if (startRightScaleRightSeq == 16) 										// Step 16: Back up robot away from scale
								{
									System.out.println("Step 16 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[16]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									yAxisRef = -0.35; 												// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[16]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 						// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.25); 					// Right side motor speed reference
				
									leftIntake.set(0.0); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 											// Assign speed ref to Right PWM Talon
				
									startRightScaleRightTime[17] = autoSequenceTime; 				// Start time of step 17 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[16]) > startRightScaleRight[16])
									|| (rightMotorPosition < startRightScaleRightPosition[16])) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startRightScaleRightSeq = 17; 							// Move to step 17
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 16: Back up robot away from scale
							
							
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 15: Stop forward movement and Open Arms
							 ****************************************************/
							if (startRightScaleRightSeq == 15) 										// Step 15: Stop forward movement and Open Arms
								{
									System.out.println("Step 15 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[15]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(0.2); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.2); 											// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startRightScaleRightTime[16] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[15]) > startRightScaleRight[15])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startRightScaleRightSeq = 16; 							// Move to step 16
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 15: Stop forward movement and Open Arms
							
							
							
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 14: Start moving forward toward scale again
							 ****************************************************/
							if (startRightScaleRightSeq == 14) 									// Step 14: Start moving forward toward scale again
								{
									System.out.println("Step 14 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[14]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									yAxisRef = 0.37; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[14]; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									startRightScaleRightTime[15] = autoSequenceTime; 			// Start time of step 7 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[14]) > startRightScaleRight[14])
									|| (rightMotorPosition > startRightScaleRightPosition[14])) 
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightScaleRightSeq = 15; 						// Move to step 15
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 14: Start moving forward toward scale again
							
							
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 13: Raise elevator up, rotate toward scale
							 ****************************************************/
							if (startRightScaleRightSeq == 13) 									// Step 13: Raise elevator up, rotate toward scale
								{
									System.out.println("Step 13 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[13]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
				
									winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);		// Start magic motion for absolute position to move to scale
	
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[13]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.24)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.24); 					// Right side motor speed reference
				
									startRightScaleRightTime[14] = autoSequenceTime; 				// Start time of step 10 when this step exits
									if ( ((autoSequenceTime - startRightScaleRightTime[13]) > startRightScaleRight[13])
									|| ( (winchMotorPosition > (winchScaleTopRef - 5000)) 
									&& ( (gyroAngleFB > (gyroAngleSP - 5)) || (gyroAngleFB < 3) ) ) )
										{
											leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startRightScaleRightSeq = 14; 							// Move to step 14
											System.out.println("total time= " + autoSequenceTime);
										}
				
								} // End of Step 13: Raise elevator up, rotate toward scale
							
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 12: Back up with second cube
							 ****************************************************/
							if (startRightScaleRightSeq == 12) // Step 12: Back up with second cube
								{
									System.out.println("Step 12 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[12]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									yAxisRef = -0.35; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[12]; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									leftIntake.set(startLeftSwitchLeftIntakeHoldRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startLeftSwitchLeftIntakeHoldRef); 			// Assign speed ref to Right PWM Talon
				
									startRightScaleRightTime[13] = autoSequenceTime; 				// Start time of step 9 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[12]) > startRightScaleRight[12])
									|| (rightMotorPosition < startRightScaleRightPosition[12]))  
										{
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightScaleRightSeq = 13; 						// Move to step 10
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 12: Back up with second cube
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT
							 * Step 11: Move to 2nd cube
							 ****************************************************/
							if (startRightScaleRightSeq == 11) // Step 11: Move to 2nd cube
								{
									System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[11]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									yAxisRef = 0.55; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[11]; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									leftIntake.set(startRightScaleRightIntakeGrabRef); 			// Assign speed ref to Left PWM Talon
									rightIntake.set(startRightScaleRightIntakeGrabRef); 		// Assign speed ref to Right PWM Talon
				
									startRightScaleRightTime[12] = autoSequenceTime; 			// Start time of step 12 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[11]) > startRightScaleRight[11])
									|| (rightMotorPosition > startRightScaleRightPosition[11])) 
										{
											leftMotorSpdRefAut = 0.0; 							// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 							// Assign open loop speed reference
											startRightScaleRightSeq = 12; 						// Move to step 12
											System.out.println("total time= " + autoSequenceTime);
										}
				
								} // End of Step 11: Move to 2nd cube
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 10: Move elevator down, close intake arms
							 ****************************************************/
							if (startRightScaleRightSeq == 10) 										// Step 10: Move elevator down, close intake arms
								{
									System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[10]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
				
									winchMotor.set(ControlMode.MotionMagic, winchBottomRef);		// Start magic motion for absolute position to move to scale
	
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[10]; 					// Assign gyro angle setpoint in 0-359 degrees
									
				
									intakeArmOpenCommand = false; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.17)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.17); 					// Right side motor speed reference
				
									startRightScaleRightTime[11] = autoSequenceTime; 				// Start time of step 10 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[10]) > startRightScaleRight[10])
									|| ((gyroAngleFB < (gyroAngleSP + 2)) && (gyroAngleFB > 150)) )
										{
											leftMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											rightMotorSpdRefAut = 0.0; 								// Assign open loop speed reference
											
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											
											startRightScaleRightSeq = 11; 							// Move to step 11
											System.out.println("total time= " + autoSequenceTime);
										}
				
								} // End of Step 10: Move elevator down, close intake arms
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 9: Back up robot away from scale
							 ****************************************************/
							if (startRightScaleRightSeq == 9) 										// Step 9: Back up robot away from scale
								{
									System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[9]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									yAxisRef = -0.35; 												// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[9]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.25)); 				// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									leftIntake.set(0.0); 											// Assign speed ref to Left PWM Talon
									rightIntake.set(0.0); 											// Assign speed ref to Right PWM Talon
				
									startRightScaleRightTime[10] = autoSequenceTime; 				// Start time of step 9 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[9]) > startRightScaleRight[9])
									|| (rightMotorPosition < startRightScaleRightPosition[9])) 
										{
											leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 	// Open loop ramp rate
											rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); // Open loop ramp rate
											
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startRightScaleRightSeq = 10; 							// Move to step 10
											System.out.println("total time= " + autoSequenceTime);
					
										}
								} // End of Step 9: Back up robot away from scale
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 8: Stop forward movement and Open Arms
							 ****************************************************/
							if (startRightScaleRightSeq == 8) 										// Step 8: Stop forward movement and Open Arms
								{
									System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[8]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									leftIntake.set(startRightScaleRightIntakeSpdRef); 				// Assign speed ref to Left PWM Talon
									rightIntake.set(startRightScaleRightIntakeSpdRef); 				// Assign speed ref to Right PWM Talon
				
									intakeArmOpenCommand = true; 									// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 						// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 						// Intake arms CLOSE solenoid
				
									startRightScaleRightTime[9] = autoSequenceTime; 					// Start time of step 8 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[8]) > startRightScaleRight[8])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											
											leftMotorSpdRefAut = -0.01; 								// Stop motion
											rightMotorSpdRefAut = 0.01; 								// Stop motion
											
											startRightScaleRightSeq = 9; 								// Move to step 9
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 8: Stop forward movement and Open Arms
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 7: Start rotate CCW to 0 Deg during decel
							 * Skip this step
							 ****************************************************/
							if (startRightScaleRightSeq == 7) 									// Step 7: Start Decel to min speed, Start rotate CW to 30 Deg during decel
								{
									System.out.println("Step 7 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightScaleRightTime[3]));
									yAxisRef = 0.15; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[7]; 											// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.25)); 			// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startRightScaleRightTime[8] = autoSequenceTime; 			// Start time of step 4 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[7]) > startRightScaleRight[7])
									|| ((gyroAngleFB < (gyroAngleSP + 2)) && (gyroAngleFB < 100))) 
										{
											// startRightScaleRightSeq = 8; 					//Move to step 8
											startRightScaleRightSeq = 18; 						// Move to step 18
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 7: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 6: Start moving forward toward scale
							 ****************************************************/
							if (startRightScaleRightSeq == 6) 									// Step 6: Start moving forward toward scale
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[6]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									
									
									yAxisRef = 0.35; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[6]; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									startRightScaleRightTime[8] = autoSequenceTime; 			// Start time of step 7 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[6]) > startRightScaleRight[6])
									|| (rightMotorPosition > startRightScaleRightPosition[6])) 
										{
											leftMotor.configOpenloopRamp(0.8, leftKTimeoutMs); 	// Open loop ramp rate
											rightMotor.configOpenloopRamp(0.8, leftKTimeoutMs); // Open loop ramp rate
										
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											startRightScaleRightSeq = 8; 						// Move to step 8
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 6: Start moving forward toward scale
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 5: Wait for Elevator to get up to height
							 ****************************************************/
							if ((startRightScaleRightSeq == 5)) 								// Step 5: Wait for Elevator to get up to height
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[5]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
				
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
									rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									
									leftMotorSpdRefAut = 0; 									// Stop motion
									rightMotorSpdRefAut = 0; 									// Stop motion
									
									startRightScaleRightTime[6] = autoSequenceTime;
									if (((autoSequenceTime - startRightScaleRightTime[5]) > startRightScaleRight[5])
									|| (winchMotorPosition > (winchScaleTopRef - 100))) 
										{
											leftMotor.configOpenloopRamp(0.9, leftKTimeoutMs); 	// Open loop ramp rate
											rightMotor.configOpenloopRamp(0.9, leftKTimeoutMs); // Open loop ramp rate
											startRightScaleRightSeq = 6; // Move to step 6
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 5: Wait for Elevator to get up to height
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 4: Drive close to switch
							 ****************************************************/
							if (startRightScaleRightSeq == 4) 									// Step 4: Drive close to switch
								{
									System.out.println("Step 4 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[4]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									yAxisRef = 0.55; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[4]; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
				
									startRightScaleRightTime[5] = autoSequenceTime; 			// Start time of step 5 when this step exits
									if ((autoSequenceTime - startRightScaleRightTime[4]) > startRightScaleRight[4]
									|| (rightMotorPosition > startRightScaleRightPosition[4])) 
										{
											
											
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											
											leftMotorSpdRefAut = 0; 							// Stop motion
											rightMotorSpdRefAut = 0; 							// Stop motion
											
											startRightScaleRightSeq = 5; 						// Move to step 5
											System.out.println("total time= " + autoSequenceTime);
										}
								} // END of Step 4: Drive close to switch
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 3: Start Decel to min speed, 
							 * Start rotate CCW to 330 Deg during decel
							 ****************************************************/
							if (startRightScaleRightSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CCW to 330 Deg during decel
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[3]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
									
									leftMotorShiftHigh.set(false); 								// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 								// Enable high speed gear on Left/Right Drive motor
									
									yAxisRef = 0.5; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[3]; 				// Assign gyro angle setpoint in 0-359 degrees
									
									if (winchMotorPosition > (winchScaleRef - 8000) )			//Wait for slow elevator to get up to position
										{
											gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); // Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0)); 	// Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.30); 		// Right side motor speed reference
										}
									else
										{
											yAxisRef = 0.30;	
											gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); // Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0)); 	// Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.20); 		// Right side motor speed reference
											System.out.println("Step 3 Waiting for Elevator");
										}
											
									startRightScaleRightTime[4] = autoSequenceTime; 			// Start time of step 4 when this step exits
									if (((autoSequenceTime - startRightScaleRightTime[3]) > startRightScaleRight[3])
									|| ((gyroAngleFB < (gyroAngleSP + 2)) && (gyroAngleFB > 200)) ) 
										{
											startRightScaleRightSeq = 4; 						// Move to step 4
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CCW to 330 Deg during decel
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 2: Drop intake arms, start moving forward to
							 * position left of SCALE
							 ****************************************************/
							if (startRightScaleRightSeq == 2) 									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[2]) + " , Gyro: " + gyroAngleFB + ", Winch Pos: " + winchMotorPosition);
				
									intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
									intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
									intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
				
									yAxisRef = 0.78; 											// Assign open loop speed reference
									gyroAngleSP = startRightScaleRightAngle[2]; 					// Assign gyro angle setpoint in 0-359 degrees
				
									if (rightMotorPosition > (startRightScaleRightPosition[2] - 26000)) 
										{
											leftMotorShiftHigh.set(false); 						// Enable high speed gear on Left/Right Drive motors
											leftMotorShiftLow.set(true); 						// Enable high speed gear on Left/Right Drive motor
										}
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
				
									if (rightMotorPosition > 1000)
										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef); // Start magic motion for absolute position to move to SCALE
				
									startRightScaleRightTime[3] = autoSequenceTime; 			// Start time of step 3 when this step exits
									if ((autoSequenceTime - startRightScaleRightTime[2]) > startRightScaleRight[2]
									|| (rightMotorPosition > startRightScaleRightPosition[2])) 
										{
											leftMotor.configOpenloopRamp(0.8, leftKTimeoutMs); 	// Open loop ramp rate
											rightMotor.configOpenloopRamp(0.8, leftKTimeoutMs); // Open loop ramp rate
											
											startRightScaleRightSeq = 3;
											System.out.println("total time= " + autoSequenceTime);
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START RIGHT - SCALE RIGHT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startRightScaleRightSeq == 1)) 					// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
				
									intakeArmOpenCommand = false; 					// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 		// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand); 		// Intake arms CLOSE solenoid
				
									leftMotorShiftHigh.set(true); 					// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(false); 					// Enable high speed gear on Left/Right Drive motor
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 								// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 		// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 											// Right half of compass
										firstGyroScan = gyroWithDrift; 				// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startRightScaleRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startRightScaleRightTime[1]) > startRightScaleRight[1]) 
										{
											startRightScaleRightSeq = 2; 			// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
			
						} // End of START RIGHT, go for SCALE RIGHT
		
					
					/**************************************************************
					 * Robot Start=RIGHT Side, Go for LEFT Switch 
					 * system clock is in milliseconds
					 **************************************************************/
					//else if (ourGameData.equals("LL") && goOpposite) // Go for Right switch starting from Left side
					else if (autonomousSelected.equals("startRightSwitchLeft")) // Go for Right switch starting from Left side
					{
						startRightSwitchLeft[0] = 0; 				// Step 0: Start of sequence
						startRightSwitchLeft[1] = 50; 				// Step 1: Close intake arms solenoid
						startRightSwitchLeft[2] = 3500; 			// Step 2: Move past Switch frame
						startRightSwitchLeft[3] = 1500; 			// Step 3: Rotate CCW 270 degrees to get behind switch
						startRightSwitchLeft[4] = 100; 				// Step 4: Reset Drive encoders to zero
						startRightSwitchLeft[5] = 3500; 			// Step 5: Drive forward, drop arms, raise elevator
						startRightSwitchLeft[6] = 1800; 			// Step 6: Rotate CCW to 180 degrees to face switch
						startRightSwitchLeft[7] = 1700; 			// Step 7: Start moving forward toward switch
						startRightSwitchLeft[8] = 600; 				// Step 8: Stop forward movement and Open Arms
						startRightSwitchLeft[9] = 2000; 			// Step 9: Back up robot away from switch 1200
						startRightSwitchLeft[10] = 500; 			// Step 10: Move elevator down, close intake arms
						startRightSwitchLeft[11] = 0; 				// Autonomous movement
						startRightSwitchLeft[12] = 0; 				// Autonomous movement
						startRightSwitchLeft[13] = 0; 				// Autonomous movement
						startRightSwitchLeft[14] = 0; 				// Autonomous movement is done
						startRightSwitchLeftFirstPosition = 44000; 	// Encoder counts - first move forward
						startRightSwitchLeftFirstAngle = 274; 		// Gyro angle for first turn toward switch
						startRightSwitchLeftSecondPosition = 37000; // Encoder counts - second move forward
						startRightSwitchLeftSecondAngle = 195; 		// Gyro angle for second turn toward switch
						startRightSwitchLeftThirdPosition = 54200; 	// Encoder counts - third move forward
						startRightSwitchLeftIntakeSpdRef = 0.5; 	// Intake motor speed ref to spit cube out
						startRightSwitchLeftForthPosition = -6500; 	// Encoder counts - forth move backward
		
						/****************************************************
						 * Step 0 of start on RIGHT, Switch on LEFT INIT
						 ****************************************************/
						if (startRightSwitchLeftSeq == 0) // time = 0
							{
								System.out.println("Step 0 : " + autoSequenceTime);
								leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 			// Open loop ramp rate
								leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
								rightMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
								rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
								startRightSwitchLeftSeq = 1; 								// Begin step 1
								startRightSwitchLeftTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
							} // End of Step 0
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT Step 10: Move elevator down, close intake arms
						 ****************************************************/
						if (startRightSwitchLeftSeq == 10) 									// Step 10: Move elevator down, close intake arms
							{
								System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchLeftTime[10]));
								leftMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
								rightMotorSpdRefAut = 0.0; 									// Assign open loop speed reference
								
								leftIntake.set(0.0); 										// Assign speed ref to Left PWM Talon
								rightIntake.set(0.0); 										// Assign speed ref to Left PWM Talon
			
								intakeArmOpenCommand = false; 								// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);
			
								winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	// Start magic motion for absolute position to move to SWITCH
								startRightSwitchLeftTime[11] = autoSequenceTime; 			// Start time of step 10 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[10]) > startRightSwitchLeft[10])) 
									{
										startRightSwitchLeftSeq = 11; 						// Move to step 11
									}
			
							} // End of Step 10: Move elevator down, close intake arms
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 9: Back up robot away from switch
						 ****************************************************/
						if (startRightSwitchLeftSeq == 9) 									// Step 8: Back up robot away from switch
							{
								System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchLeftTime[9]) + " , Gyro: " + gyroAngleFB);
								yAxisRef = -0.30; 											// Assign open loop speed reference
								gyroAngleSP = startRightSwitchLeftSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.15)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								startRightSwitchLeftTime[10] = autoSequenceTime; 			// Start time of step 10 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[9]) > startRightSwitchLeft[9])
								|| (rightMotorPosition < startRightSwitchLeftForthPosition)) 
									{
										leftMotorSpdRefAut = 0; 							// Stop motion
										rightMotorSpdRefAut = 0; 							// Stop motion
										startRightSwitchLeftSeq = 10; 						// Move to step 10
									}
							} // End of Step 9: Back up robot away from switch
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 8: Stop forward movement and Open Arms
						 ****************************************************/
						if (startRightSwitchLeftSeq == 8) 									// Step 8: Stop forward movement and Open Arms
							{
								System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchLeftTime[8]) + " , Gyro: " + gyroAngleFB);
								leftMotorSpdRefAut = 0; 									// Stop motion
								rightMotorSpdRefAut = 0; 									// Stop motion
			
								leftIntake.set(startRightSwitchLeftIntakeSpdRef); 			// Assign speed ref to Left PWM Talon
								rightIntake.set(startRightSwitchLeftIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
			
								intakeArmOpenCommand = true; 								// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);
			
								startRightSwitchLeftTime[9] = autoSequenceTime; 			// Start time of step 8 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[8]) > startRightSwitchLeft[8])) 
									{
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startRightSwitchLeftSeq = 9; 						// Move to step 8
									}
			
							} // End of Step 8: Stop forward movement and Open Arms
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 7: Start moving forward toward switch
						 ****************************************************/
						if (startRightSwitchLeftSeq == 7) 									// Step 7: Start moving forward toward switch
							{
								System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchLeftTime[7]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.26; 											// Assign open loop speed reference
								gyroAngleSP = startRightSwitchLeftSecondAngle;				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								startRightSwitchLeftTime[9] = autoSequenceTime; 			// Start time of step 7 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[7]) > startRightSwitchLeft[7])
								|| (rightMotorPosition > startRightSwitchLeftThirdPosition)) 
									{
										leftMotorSpdRefAut = 0; 							// Stop motion
										rightMotorSpdRefAut = 0; 							// Stop motion
										startRightSwitchLeftSeq = 8; 						// Move to step 8
									}
							} // End of Step 7: Start moving forward toward switch
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 6: Rotate CCW to 180 degrees to face switch
						 ****************************************************/
						if (startRightSwitchLeftSeq == 6) 									// Step 6: Rotate CCW to 180 degrees to face switch
							{
								System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchLeftTime[6]) + " , Gyro: " + gyroAngleFB);
								yAxisRef = 0.0; 											// Assign open loop speed reference
								gyroAngleSP = startRightSwitchLeftSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.25)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25); 				// Right side motor speed reference
			
								startRightSwitchLeftTime[7] = autoSequenceTime; 			// Start time of step 7 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[6]) > startRightSwitchLeft[6])
								|| ((gyroAngleFB < (startRightSwitchLeftSecondAngle + 3)) && (gyroAngleFB > 170))) 
									{
										startRightSwitchLeftSeq = 7; 						// Move to step 7
									}
							} // End of Step 6: Rotate CCW to 180 degrees to face switch
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 5: Drive forward, drop arms, raise elevator
						 ****************************************************/
						if (startRightSwitchLeftSeq == 5) 									// Step 5: Drive forward, drop arms, raise elevator
							{
								System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[5]) + " , Gyro: " + gyroAngleFB);
			
								intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
								intakeArmPivotDOWN.set(intakeArmPivotDownCommand);			// Enable Intake Arm to pivot DOWN
								intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
			
								yAxisRef = 0.55; 											// Assign open loop speed reference
								gyroAngleSP = startRightSwitchLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								if (rightMotorPosition > 10000)
									winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); // Start magic motion for absolute position to move to SWITCH
			
								startRightSwitchLeftTime[6] = autoSequenceTime; 			// Start time of step 6 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[5]) > startRightSwitchLeft[5])
								|| (rightMotorPosition > startRightSwitchLeftSecondPosition)) 
									{
										startRightSwitchLeftSeq = 6; // Move to step 6
									}
							} // END of Step 5: Drive forward, drop arms, raise elevator
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 4: Reset Drive encoders to zero
						 ****************************************************/
						if (startRightSwitchLeftSeq == 4) 									// Step 4: Reset Drive encoders to zero
							{
								System.out.println("Step 4 : " + (autoSequenceTime - startRightSwitchLeftTime[4]));
								leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
								rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
			
								startRightSwitchLeftTime[5] = autoSequenceTime;
								if ((autoSequenceTime - startRightSwitchLeftTime[4]) > startRightSwitchLeft[4]) 
									{
										startRightSwitchLeftSeq = 5; 						// Move to step 5
									}
							} // End of Step 4 Reset Drive encoders to zero
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT Step 3: Rotate CCW 270 degrees to get behind switch
						 ****************************************************/
						if (startRightSwitchLeftSeq == 3) 									// Step 3: Rotate CCW to 270 degrees to get behind switch
							{
								System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightSwitchLeftTime[3]));
								yAxisRef = 0.30; 											// Assign open loop speed reference
								gyroAngleSP = startRightSwitchLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef)); 					// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.27); 				// Right side motor speed reference
			
								startRightSwitchLeftTime[4] = autoSequenceTime; 			// Start time of step 4 when this step exits
								if (((autoSequenceTime - startRightSwitchLeftTime[3]) > startRightSwitchLeft[3])
								|| ((gyroAngleFB < (startRightSwitchLeftFirstAngle + 3)) && (gyroAngleFB > 200))) 
									{
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startRightSwitchLeftSeq = 4; 						// Move to step 4
									}
							} // End of Step 3: Rotate CCW to 270 degrees to get behind switch
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 2: Move past Switch frame
						 ****************************************************/
						if (startRightSwitchLeftSeq == 2) 									// Step 2: Move past Switch frame
							{
								System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
			
								leftMotorShiftHigh.set(false); 								// Enable high speed gear on Left/Left Drive motors
								leftMotorShiftLow.set(true); 								// Disable high speed gear on Left/Left Drive motors
			
								yAxisRef = 0.85; 											// Assign open loop speed reference
								gyroAngleSP = 0.0; 											// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								// if (rightMotorPosition > 12000)
								// winchMotor.set(ControlMode.MotionMagic, winchSwitchRef); //Start magic motion
								// for absolute position to move to SWITCH
			
								startRightSwitchLeftTime[3] = autoSequenceTime; 			// Start time of step 3 when this step exits
								if ((autoSequenceTime - startRightSwitchLeftTime[2]) > startRightSwitchLeft[2]
								|| (rightMotorPosition > startRightSwitchLeftFirstPosition)) 
									{
										startRightSwitchLeftSeq = 3;
									}
							} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
		
						/****************************************************
						 * START RIGHT - SWITCH LEFT 
						 * Step 1: Close intake arms solenoid
						 ****************************************************/
						if ((startRightSwitchLeftSeq == 1)) // Step 1: Close intake arms solenoid
							{
								System.out.println("Step 1 : " + autoSequenceTime);
			
								intakeArmOpenCommand = false;							 	// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);	
			
								// Snapshot gyro start angle longer than 1 scan, init scan isn't working
								if (gyroYaw < 0) 											// Left half of compass
									firstGyroScan = 180 + (180 + gyroYaw); 					// gyro raw -179 to 0 = gyro compass angle +181 to 359
								else // Right half of compass
									firstGyroScan = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
			
								startRightSwitchLeftTime[2] = autoSequenceTime;
								if ((autoSequenceTime - startRightSwitchLeftTime[1]) > startRightSwitchLeft[1]) {
									startRightSwitchLeftSeq = 2; 							// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
								}
							} // End of Step 1: Close intake arms solenoid
		
					} // End of Go for LEFT SWITCH starting from RIGHT SIDE
		
					/**************************************************************
					 * Robot START=RIGHT Side, Go for LEFT SCALE
					 * system clock is in milliseconds
					 **************************************************************/
					else if (autonomousSelected.equals("startRightScaleLeft")) // Go for Right Scale starting from Left side
					{
						startRightScaleLeft[0] = 0; 				// Step 0: Start of sequence
						startRightScaleLeft[1] = 50; 				// Step 1: Close intake arms solenoid, shift to high speed
						startRightScaleLeft[2] = 3500; 				// Step 2: Move past Switch frame
						startRightScaleLeft[3] = 1500; 				// Step 3: Rotate CCW 90 degrees to get behind switch
						startRightScaleLeft[4] = 100; 				// Step 4: Reset Drive encoders to zero
						startRightScaleLeft[5] = 3500; 				// Step 5: Drive forward, drop arms, raise elevator
						startRightScaleLeft[6] = 1800; 				// Step 6: Rotate CW to 0 degrees to face Scale
						startRightScaleLeft[7] = 1600; 				// Step 7: Start moving forward toward switch
						startRightScaleLeft[8] = 800; 				// Step 8: Stop forward movement and Open Arms
						startRightScaleLeft[9] = 2300; 				// Step 9: Back up robot away from switch 1200
						startRightScaleLeft[10] = 2100; 			// Step 10: Move elevator down, close intake arms, rotate to face switch
						startRightScaleLeft[11] = 50; 				// Step 11: Stop motion
						startRightScaleLeft[12] = 0; 				// Autonomous movement
						startRightScaleLeft[13] = 0; 				// Autonomous movement
						startRightScaleLeft[14] = 0; 				// Autonomous movement is done
						
						startRightScaleLeftPosition[1] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[1] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[2] = 42000;		//Autonomous mode position points
						startRightScaleLeftAngle[2] = 0.0;			//Autonomous mode angle points
						startRightScaleLeftPosition[3] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[3] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[4] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[4] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[5] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[5] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[6] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[6] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[7] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[7] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[8] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[8] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[9] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[9] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[10] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[10] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[11] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[11] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[12] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[12] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[13] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[13] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[14] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[14] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[15] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[15] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[16] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[16] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[17] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[17] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[18] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[18] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[19] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[19] = 0;			//Autonomous mode angle points
						startRightScaleLeftPosition[20] = 0;			//Autonomous mode position points
						startRightScaleLeftAngle[20] = 0;			//Autonomous mode angle points
						
						
						startRightScaleLeftFirstPosition = 42600; 	// Encoder counts - first move forward
						startRightScaleLeftFirstAngle = 272; 		// Gyro angle for first turn toward scale
						startRightScaleLeftSecondPosition = 36700; // Encoder counts - second move forward
						startRightScaleLeftSecondAngle = 20; 		// Gyro angle for second turn toward scale
						startRightScaleLeftThirdPosition = 52000; 	// Encoder counts - third move forward toward scale
						startRightScaleLeftThirdAngle = 300; 		// Gyro angle for third turn away from scale
						startRightScaleLeftIntakeSpdRef = -0.7; 	// Intake motor speed ref to spit cube out
						startRightScaleLeftForthPosition = -4000;	// Encoder counts - forth move backward
						startRightScaleLeftFourthAngle = 160; 		// Gyro angle for fourth turn away from scale
		
						/****************************************************
						 * Step 0 of start on RIGHT, SCALE on LEFT INIT
						 ****************************************************/
						if (startRightScaleLeftSeq == 0) 									// time = 0
							{
								System.out.println("Step 0 : " + autoSequenceTime);
								leftMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 			// Open loop ramp rate
								leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
								rightMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 		// Open loop ramp rate
								rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
								startRightScaleLeftSeq = 1; 								// Begin step 1
								startRightScaleLeftTime[1] = autoSequenceTime; 				// Start time of step 1 when this step exits
							} // End of Step 0
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 11: Stop motion
						 ****************************************************/
						if (startRightScaleLeftSeq == 11) 									// Step 11: Stop motion
							{
								System.out.println("Step 11 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[11]));
			
								leftMotorSpdRefAut = 0; 									// Stop motion
								rightMotorSpdRefAut = 0; 									// Stop motion
			
								startRightScaleLeftTime[12] = autoSequenceTime; 			// Start time of step 12 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[11]) > startRightScaleLeft[11])) 
									{
										startRightScaleLeftSeq = 12; 						// Move to step 12
									}
			
							} // End of Step 11: Stop motion
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 10: Move elevator down, close intake arms,
						 * rotate to face switch
						 ****************************************************/
						if (startRightScaleLeftSeq == 10) 									// Step 10: Move elevator down, close intake arms, rotate to face switch
							{
								System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[10]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.0; // Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftFourthAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.25)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25); 				// Right side motor speed reference
			
								leftIntake.set(0.0);										// Assign speed ref to Left PWM Talon
								rightIntake.set(0.0); 										// Assign speed ref to Right PWM Talon
			
								intakeArmOpenCommand = false; 								// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);
			
								winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	// Start magic motion for absolute position to move to SWITCH
			
								startRightScaleLeftTime[11] = autoSequenceTime; 			// Start time of step 10 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[10]) > startRightScaleLeft[10])) 
									{
										startRightScaleLeftSeq = 11; 						// Move to step 11
									}
			
							} // End of Step 10: Move elevator down, close intake arms, rotate to face switch
			
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 9: Back up robot away from switch
						 ****************************************************/
						if (startRightScaleLeftSeq == 9) 									// Step 8: Back up robot away from switch
							{
								System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[9]) + " , Gyro: " + gyroAngleFB);
								yAxisRef = -0.22; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								startRightScaleLeftTime[10] = autoSequenceTime; 			// Start time of step 10 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[9]) > startRightScaleLeft[9])
								|| (rightMotorPosition < startRightScaleLeftForthPosition)) 
									{
										leftMotorSpdRefAut = 0; 							// Stop motion
										rightMotorSpdRefAut = 0; 							// Stop motion
										startRightScaleLeftSeq = 10; 						// Move to step 10
									}
							} // End of Step 9: Back up robot away from switch
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 8: Stop forward movement and Open Arms
						 ****************************************************/
						if (startRightScaleLeftSeq == 8) 									// Step 8: Stop forward movement and Open Arms
							{
								System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[8]) + " , Gyro: " + gyroAngleFB);
								leftMotorSpdRefAut = 0; 									// Stop motion
								rightMotorSpdRefAut = 0; 									// Stop motion
			
								leftIntake.set(startRightScaleLeftIntakeSpdRef); 			// Assign speed ref to Left PWM Talon
								rightIntake.set(startRightScaleLeftIntakeSpdRef); 			// Assign speed ref to Right PWM Talon
			
								intakeArmOpenCommand = true; 								// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);
			
								startRightScaleLeftTime[9] = autoSequenceTime; 				// Start time of step 8 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[8]) > startRightScaleLeft[8])) 
									{
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startRightScaleLeftSeq = 9; 						// Move to step 8
									}
			
							} // End of Step 8: Stop forward movement and Open Arms
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 7: Start moving forward toward scale
						 ****************************************************/
						if (startRightScaleLeftSeq == 7) 									// Step 7: Start moving forward toward switch
							{
								System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[7]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.30; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef); 					// Right side motor speed reference
			
								startRightScaleLeftTime[9] = autoSequenceTime; 				// Start time of step 7 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[7]) > startRightScaleLeft[7])
								|| (rightMotorPosition > startRightScaleLeftThirdPosition)) 
									{
										startRightScaleLeftSeq = 8; 						// Move to step 8
									}
							} // End of Step 7: Start moving forward toward switch
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 6: Rotate CW to 0 degrees to face Scale
						 ****************************************************/
						if (startRightScaleLeftSeq == 6) 									// Step 6: Rotate CW to 0 degrees to face Scale
							{
								System.out.println("Step 6 : " + rightMotorPosition + " , " + (autoSequenceTime - startRightScaleLeftTime[6]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.15; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.35); 				// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0); 				// Right side motor speed reference
			
								startRightScaleLeftTime[7] = autoSequenceTime; 				// Start time of step 7 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[6]) > startRightScaleLeft[6])
								|| ((winchMotorPosition > winchScaleRef)
										&& ((gyroAngleFB < (gyroAngleSP + 5)) || (gyroAngleFB > 355)))) 
									{
										startRightScaleLeftSeq = 7; // Move to step 7
									}
							} // End of Step 6: Rotate CW to 0 degrees to face Scale
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 5: Drive forward, drop arms, raise elevator
						 ****************************************************/
						if (startRightScaleLeftSeq == 5) 									// Step 5: Drive forward, drop arms, raise elevator
							{
								System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleLeftTime[5]) + " , Gyro: " + gyroAngleFB);
			
								intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
								intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
								intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
			
								if (rightMotorPosition > (startRightScaleLeftSecondPosition - 15000)) 
									{
										leftMotorShiftHigh.set(false); 						// Enable high speed gear on Left/Right Drive motors
										leftMotorShiftLow.set(true); 						// Enable high speed gear on Left/Right Drive motor
									}
			
								yAxisRef = 0.75; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								if (rightMotorPosition > 1000)
									winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef); // Start magic motion for absolute position to move to SWITCH
			
								startRightScaleLeftTime[6] = autoSequenceTime; 				// Start time of step 6 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[5]) > startRightScaleLeft[5])
								|| (rightMotorPosition > startRightScaleLeftSecondPosition)) 
									{
										leftMotorSpdRefAut = 0; 							// Stop motion
										rightMotorSpdRefAut = 0; 							// Stop motion
										startRightScaleLeftSeq = 6; 						// Move to step 6
									}
							} // END of Step 5: Drive forward, drop arms, raise elevator
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 4: Reset Drive encoders to zero
						 ****************************************************/
						if (startRightScaleLeftSeq == 4) 									// Step 4: Reset Drive encoders to zero
							{
								System.out.println("Step 4 : " + (autoSequenceTime - startRightScaleLeftTime[4]));
								leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
								rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
			
								startRightScaleLeftTime[5] = autoSequenceTime;
								if ((autoSequenceTime - startRightScaleLeftTime[4]) > startRightScaleLeft[4]) 
									{
										startRightScaleLeftSeq = 5; 						// Move to step 5
									}
							} // End of Step 4 Reset Drive encoders to zero
		
						/****************************************************
						 * START RIGHT - SCALE LEFT Step 3: Rotate CCW 90 degrees to get behind switch
						 ****************************************************/
						if (startRightScaleLeftSeq == 3) 									// Step 3: Rotate CCW 90 degrees to get behind switch
							{
								System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightScaleLeftTime[3]));
			
								intakeArmPivotDownCommand = true; 							// Intake Arm to pivot Down command
								intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot DOWN
								intakeArmPivotUP.set(!intakeArmPivotDownCommand); 			// Enable Intake Arm to pivot UP
			
								leftMotorShiftHigh.set(false); 								// Disable high speed gear on Left/Right Drive motors
								leftMotorShiftLow.set(true); 								// Disable high speed gear on Left/Right Drive motor
			
								yAxisRef = 0.30; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * (yAxisRef); 					// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.27); 				// Right side motor speed reference
			
								startRightScaleLeftTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
								if (((autoSequenceTime - startRightScaleLeftTime[3]) > startRightScaleLeft[3])
								|| ((gyroAngleFB < (startRightScaleLeftFirstAngle + 3)) && (gyroAngleFB > 200))) 
									{
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
										startRightScaleLeftSeq = 4; 						// Move to step 4
									}
							} // End of Step 3: Rotate CCW 90 degrees to get behind switch
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 2: Move past Switch frame
						 ****************************************************/
						if (startRightScaleLeftSeq == 2) 									// Step 2: Move past Switch frame
							{
								System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
			
								yAxisRef = 0.85; 											// Assign open loop speed reference
								gyroAngleSP = startRightScaleLeftAngle[2]; 					// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 		// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 	// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 				// Right side motor speed reference
			
								if (rightMotorPosition > (startRightScaleLeftFirstPosition - 12000)) 
									{
										leftMotorShiftHigh.set(false); 						// Enable high speed gear on Left/Right Drive motors
										leftMotorShiftLow.set(true); 						// Enable high speed gear on Left/Right Drive motor
									}
			
								startRightScaleLeftTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
								if ((autoSequenceTime - startRightScaleLeftTime[2]) > startRightScaleLeft[2]
								|| (rightMotorPosition > startRightScaleLeftPosition[2])) 
									{
										startRightScaleLeftSeq = 3;
									}
							} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
		
						/****************************************************
						 * START RIGHT - SCALE LEFT 
						 * Step 1: Close intake arms solenoid, shift to high speed
						 ****************************************************/
						if ((startRightScaleLeftSeq == 1)) 									// Step 1: Close intake arms solenoid, shift to high speed
							{
								System.out.println("Step 1 : " + autoSequenceTime);
								intakeArmOpenCommand = false; 								// Intake arms Closed command
								intakeArmOpen.set(intakeArmOpenCommand); 					// Intake arms OPEN solenoid
								intakeArmClose.set(!intakeArmOpenCommand);					// Intake arms CLOSE solenoid
			
								leftMotorShiftHigh.set(true); 								// Enable high speed gear on Left/Right Drive motors
								leftMotorShiftLow.set(false); 								// Enable high speed gear on Left/Right Drive motor
			
								// Snapshot gyro start angle longer than 1 scan, init scan isn't working
								if (gyroYaw < 0) 											// Left half of compass
									firstGyroScan = 180 + (180 + gyroYaw); // gyro raw -179 to 0 = gyro compass angle +181 to 359
								else 														// Right half of compass
									firstGyroScan = gyroWithDrift; 							// gyro raw 0 to 179 = gyro compass angle 0 to 179
			
								startRightScaleLeftTime[2] = autoSequenceTime;
								if ((autoSequenceTime - startRightScaleLeftTime[1]) > startRightScaleLeft[1]) 
									{
										startRightScaleLeftSeq = 2; 						// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
									}
							} // End of Step 1: Close intake arms solenoid, shift to hi	
		
					} // End of Go for LEFT SCALE starting from RIGHT SIDE
	
				else 
					{
						autonomousSelected = "default";										// Run default Autonomous mode of just driving forward
						System.out.println("Run Default Code");
					}
	
			} // End of robot starts on right
	
			/***************************************************************************************************************
			 * 
			 * DEFAULT STRAIGHT DEFAULT STRAIGHT DEFAULT STRAIGHT DEFAULT STRAIGHT DEFAULT STRAIGHT
			 * 
			 ***************************************************************************************************************/
			if (autonomousSelected.equals("default")) // Robot just goes forward
				{
					/**************************************************************
					 * Robot Start=ANY Side, Just go straight system clock is in milliseconds
					 **************************************************************/
					if ( (matchStartPosition == 0) || (matchStartPosition == 2) || (matchStartPosition == 3) ) // Go straight
						{
							startAnyForward[0] = 0; 				// Step 0: Start of sequence
							startAnyForward[1] = 50; 				// Step 1: Close intake arms solenoid
							startAnyForward[2] = 8000; 				// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
							startAnyForward[3] = 500; 				// Step 3: Start Decel to zero speed
							startAnyForward[4] = 0; 				// Autonomous movement
							startAnyForward[5] = 0; 				// Autonomous movement
							startAnyForward[6] = 0; 				// Autonomous movement
							startAnyForward[7] = 0; 				// Autonomous movement
							startAnyForward[8] = 0; 				// Autonomous movement
							startAnyForward[9] = 0; 				// Autonomous movement
							startAnyForward[10] = 0; 				// Autonomous movement
							startAnyForward[11] = 0; 				// Autonomous movement
							startAnyForward[12] = 0; 				// Autonomous movement
							startAnyForward[13] = 0; 				// Autonomous movement
							startAnyForward[14] = 14900; 			// Autonomous movement is done
							startAnyForwardFirstPosition = 25000; 	// Encoder counts - first move forward
							startAnyForwardFirstAngle = 0; 			// Gyro angle for first turn toward switch
							startAnyForwardSecondPosition = 0; 		// Encoder counts - second move forward
							startAnyForwardIntakeSpdRef = -0.5; 	// Intake motor speed ref to spit cube out
							startAnyForwardThirdPosition = -4000; 	// Encoder counts - third move backward
							startAnyForwardSecondAngle = 0; 		// Gyro angle for second turn toward switch
							/****************************************************
							 * Step 0 of start on ANY side, Just go Straight INIT
							 ****************************************************/
							if (startAnyForwardSeq == 0) 									// time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 		// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs); 	// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startAnyForwardSeq = 1; // Begin step 1
									startAnyForwardTime[1] = autoSequenceTime; 				// Start time of step 1 when this step exits
								} // End of Step 0
			
							/****************************************************
							 * START ANY - Just go Straight 
							 * Step 3: Start Decel to zero speed,
							 ****************************************************/
							if (startAnyForwardSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
								{
									System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startAnyForwardTime[3]));
									leftMotorSpdRefAut = 0; 								// Stop motion
									rightMotorSpdRefAut = 0; 								// Stop motion
				
									startAnyForwardTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startAnyForwardTime[3]) > startAnyForward[3])
									|| ((gyroAngleFB < (startAnyForwardFirstAngle + 2)) && (gyroAngleFB > 200))) 
										{
											startAnyForwardSeq = 14; 						// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
			
							/****************************************************
							 * START ANY - Just go Straight 
							 * Step 2: Drop intake arms, start moving forward
							 * to position left of scale,start raising elevator
							 ****************************************************/
							if (startAnyForwardSeq == 2) 									// Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + "," + (autoSequenceTime - startAnyForwardTime[2]));
									// intakeArmPivotDOWN.set(true); 						//Enable Intake Arm to pivot DOWN
									// intakeArmPivotUP.set(false); 						//Enable Intake Arm to pivot UP
				
									leftMotorShiftHigh.set(false);							// Enable high speed gear on Left/Right Drive motors
									leftMotorShiftLow.set(true); 							// Disable high speed gear on Left/Right Drive motors
				
									yAxisRef = 0.30; 										// Assign open loop speed reference
									gyroAngleSP = 3.0; 										// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 	// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); // Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 			// Right side motor speed reference
				
				
									startAnyForwardTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startAnyForwardTime[2]) > startAnyForward[2]
									|| (rightMotorPosition > startAnyForwardFirstPosition)) 
										{
											startAnyForwardSeq = 3;
										}
								} // End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
			
							/****************************************************
							 * START ANY - Just go Straight Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startAnyForwardSeq == 1)) 									// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + (autoSequenceTime - startAnyForwardTime[1]));
									intakeArmOpenCommand = false; 							// Intake arms Closed command
									intakeArmOpen.set(intakeArmOpenCommand); 				// Intake arms OPEN solenoid
									intakeArmClose.set(!intakeArmOpenCommand);				// Intake arms CLOSE solenoid
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 										// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 				// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 													// Right half of compass
										firstGyroScan = gyroWithDrift; 						// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startAnyForwardTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startAnyForwardTime[1]) > startAnyForward[1]) 
										{
											startAnyForwardSeq = 2; 						// Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
			
						} // End of Just Go Forward starting from Any side but center
					
					/**************************************************************
					 * Robot START CENTER, DEFAULT RIGHT - Angle around center cube stack
					 * system clock is in milliseconds
					 **************************************************************/
					if ( (matchStartPosition == 1) ) // Angle around center cube stack
						{
							startCenterDefaultRight[0] = 0; 				// Step 0: Start of sequence
							startCenterDefaultRight[1] = 50; 				// Step 1: Close intake arms solenoid
							startCenterDefaultRight[2] = 950; 				// Step 2: Start moving forward to RGHT DEFAULT position
							startCenterDefaultRight[3] = 1500; 				// Step 3: Start Decel to min speed, Start rotate CW to 50 Deg during decel
							startCenterDefaultRight[4] = 1400; 				// Step 4: Use gyro correction to move to correct angle
							startCenterDefaultRight[5] = 3000; 				// Step 5 Rotate CCW to 0 degrees toward Switch
							startCenterDefaultRight[6] = 2800; 				// Step 6: Start moving forward toward switch 1050
							startCenterDefaultRight[7] = 600; 				// Step 7: Stop forward movement
							startCenterDefaultRight[8] = 0; 				// Step 8: Autonomous movement
							startCenterDefaultRight[9] = 0; 				// Step 9: Autonomous movement
							startCenterDefaultRight[10] = 0; 				// Step 10: Autonomous movement
							startCenterDefaultRight[11] = 0; 				// Step 11: Autonomous movement
							startCenterDefaultRight[12] = 0; 				// Step 12: Autonomous movement
							startCenterDefaultRight[13] = 0; 				// Step 13: Autonomous movement
							startCenterDefaultRight[14] = 0; 				// Step 14: Autonomous movement
							startCenterDefaultRight[15] = 0; 				// Step 15: Autonomous movement
							startCenterDefaultRight[16] = 0; 				// Step 16: Autonomous movement
							startCenterDefaultRight[17] = 0; 				// Autonomous movement is done
							startCenterDefaultRight[18] = 0; 				// Autonomous movement is done
							startCenterDefaultRightFirstPosition = 3500; 	// Encoder counts - first move forward
							startCenterDefaultRightFirstAngle = 50; 		// Gyro angle for first turn toward switch
							startCenterDefaultRightSecondPosition = 23000; 	// Encoder counts - second move forward
							startCenterDefaultRightSecondAngle = 0.0; 		// Gyro angle for second turn toward switch
							startCenterDefaultRightThirdPosition = 35000; 	// Encoder counts - third move forward

							/****************************************************
							 * Step 0 of start on CENTER, DEFAULT on RIGHT INIT
							 ****************************************************/
							if (startCenterDefaultRightSeq == 0) // time = 0
								{
									System.out.println("Step 0 : " + autoSequenceTime);
									leftMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 			// Open loop ramp rate
									leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); 	// Zero the position encoder
									rightMotor.configOpenloopRamp(0.1, leftKTimeoutMs); 		// Open loop ramp rate
									rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
									startCenterDefaultRightSeq = 1; 							// Begin step 1
									startCenterDefaultRightTime[1] = autoSequenceTime; 			// Start time of step 1 when this step exits
								} // End of Step 0
						
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 7: Stop forward movement
							 ****************************************************/
							if (startCenterDefaultRightSeq == 7) 									// Step 7: Stop forward movement
								{
									System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterDefaultRightTime[7]) + " , Gyro: " + gyroAngleFB);
									leftMotorSpdRefAut = 0; 										// Stop motion
									rightMotorSpdRefAut = 0; 										// Stop motion
				
									startCenterDefaultRightTime[8] = autoSequenceTime; 				// Start time of step 8 when this step exits
									if (((autoSequenceTime - startCenterDefaultRightTime[7]) > startCenterDefaultRight[7])) 
										{
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs); // Zero the position encoder
											startCenterDefaultRightSeq = 8; 							// Move to step 8
										}
				
								} // End of Step 7: Stop forward movement
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 6: Start moving forward toward switch
							 ****************************************************/
							if (startCenterDefaultRightSeq == 6) 									// Step 5: Start moving forward toward switch
								{
									System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterDefaultRightTime[6]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.30; 												// Assign open loop speed reference
									gyroAngleSP = startCenterDefaultRightSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterDefaultRightTime[7] = autoSequenceTime; 				// Start time of step 7 when this step exits
									if (((autoSequenceTime - startCenterDefaultRightTime[6]) > startCenterDefaultRight[6])
									|| (rightMotorPosition > startCenterDefaultRightThirdPosition)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterDefaultRightSeq = 7; 						// Move to step 7
										}
								} // End of Step 6: Start moving forward toward switch
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 5: Rotate CW to 0 degrees toward Switch
							 ****************************************************/
							if ((startCenterDefaultRightSeq == 5)) 									// Step 5: Rotate CW to 0 degrees toward Switch
								{
									System.out.println("Step 5 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterDefaultRightTime[5]) + " , Gyro: " + gyroAngleFB);
				
									yAxisRef = 0.0; 												// Assign open loop speed reference
									gyroAngleSP = startCenterDefaultRightSecondAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef - 0.17); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.327); 				// Right side motor speed reference
				
									startCenterDefaultRightTime[6] = autoSequenceTime;
									if ((autoSequenceTime - startCenterDefaultRightTime[5]) > startCenterDefaultRight[5]
									|| (gyroAngleFB < (gyroAngleSP + 5)) || (gyroAngleFB > 358)) 
										{
											leftMotorSpdRefAut = 0; 								// Stop motion
											rightMotorSpdRefAut = 0; 								// Stop motion
											startCenterDefaultRightSeq = 6; 						// Move to step 6
										}
								} // End of Step 5 Rotate CW to 0 degrees toward Switch
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 4: Use gyro correction to move to correct
							 * angle closer to the Switch
							 ****************************************************/
							if (startCenterDefaultRightSeq == 4) 									// Step 4: Use gyro correction to move to correct angle
							{
								System.out.println("Step 4 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterDefaultRightTime[4]) + " , Gyro: " + gyroAngleFB);
			
								yAxisRef = 0.24; 													// Assign open loop speed reference
								gyroAngleSP = startCenterDefaultRightFirstAngle; 					// Assign gyro angle setpoint in 0-359 degrees
			
								gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 				// Sync robot to gyro angle setpoint
								leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 			// Left side motor speed reference
								rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0); 						// Right side motor speed reference
			
								startCenterDefaultRightTime[5] = autoSequenceTime; // Start time of step 5 when this step exits
								if ((autoSequenceTime - startCenterDefaultRightTime[4]) > startCenterDefaultRight[4]
								|| ((rightMotorPosition > startCenterDefaultRightSecondPosition))) 
									{
										startCenterDefaultRightSeq = 5; 								// Move to step 5
									}
							}	// End of Step 4: Use gyro correction to move to correct angle
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 3: Start Decel to min speed, Start rotate CW
							 * to 45 Deg during decel
							 ****************************************************/
							if (startCenterDefaultRightSeq == 3) 									// Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
								{
									System.out.println("Step 3 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterDefaultRightTime[3]) + " , Gyro: " + gyroAngleFB);
									yAxisRef = 0.24; 												// Assign open loop speed reference
									gyroAngleSP = startCenterDefaultRightFirstAngle; 				// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.20); 					// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterDefaultRightTime[4] = autoSequenceTime; 				// Start time of step 4 when this step exits
									if (((autoSequenceTime - startCenterDefaultRightTime[3]) > startCenterDefaultRight[3])
									|| ((gyroAngleFB > (gyroAngleSP - 5)) && (gyroAngleFB < 100))) 
										{
											startCenterDefaultRightSeq = 4; 						// Move to step 4
										}
								} // End of Step 3: Start Decel to min speed, Start rotate CW to 315 Deg during decel
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 2: Start moving forward to RGHT DEFAULT position
							 ****************************************************/
							if (startCenterDefaultRightSeq == 2) 									// Step 2: Start moving forward to RGHT DEFAULT position
								{
									System.out.println("Step 2 pos: " + rightMotorPosition + " , " + autoSequenceTime + " gyro: " + gyroAngleFB);
				
									yAxisRef = 0.35; 												// Assign open loop speed reference
									gyroAngleSP = 0.0; 												// Assign gyro angle setpoint in 0-359 degrees
				
									gyroAngleSync = syncAngle(gyroAngleSP, gyroAngleFB); 			// Sync robot to gyro angle setpoint
									leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync)); 		// Left side motor speed reference
									rightMotorSpdRefAut = 1.0 * (yAxisRef); 						// Right side motor speed reference
				
									startCenterDefaultRightTime[3] = autoSequenceTime; 				// Start time of step 3 when this step exits
									if ((autoSequenceTime - startCenterDefaultRightTime[2]) > startCenterDefaultRight[2]
									|| (rightMotorPosition > startCenterDefaultRightFirstPosition)) 
										{
											startCenterDefaultRightSeq = 3;
										}
								} // End of Step 2: Start moving forward to RGHT DEFAULT position
			
							/****************************************************
							 * START CENTER - DEFAULT RIGHT 
							 * Step 1: Close intake arms solenoid
							 ****************************************************/
							if ((startCenterDefaultRightSeq == 1)) 									// Step 1: Close intake arms solenoid
								{
									System.out.println("Step 1 : " + autoSequenceTime);
									intakeArmOpen.set(false); 										// Intake arms OPEN solenoid
									intakeArmClose.set(true); 										// Intake arms CLOSE solenoid
				
									// Snapshot gyro start angle longer than 1 scan, init scan isn't working
									if (gyroYaw < 0) 												// Left half of compass
										firstGyroScan = 180 + (180 + gyroYaw); 						// gyro raw -179 to 0 = gyro compass angle +181 to 359
									else 															// Right half of compass
										firstGyroScan = gyroWithDrift; 								// gyro raw 0 to 179 = gyro compass angle 0 to 179
				
									startCenterDefaultRightTime[2] = autoSequenceTime;
									if ((autoSequenceTime - startCenterDefaultRightTime[1]) > startCenterDefaultRight[1]) 
										{
											startCenterDefaultRightSeq = 2; // Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
										}
								} // End of Step 1: Close intake arms solenoid
						
						
						}	// End of START CENTER, DEFAULT RIGHT 
		
				}	//Default settings
	
			// System.out.println("Gyro = " + gyroAngleFB);
			leftMotor.set(ControlMode.PercentOutput, leftMotorSpdRefAut); 					// Assign speed ref to Left CAN bus Talon
			leftSlaveMotor.follow(leftMotor); 												// Assign Left Slave motor to follow Left Talon motor
			rightMotor.set(ControlMode.PercentOutput, rightMotorSpdRefAut); 				// Assign speed ref to Right CAN bus Talon
			rightSlaveMotor.follow(rightMotor); 											// Assign Right Slave motor to follow Left Talon motor
			updateNetworkTables();
	
		} // End of Autonomous Periodic

	public void teleopInit() // Action routine: Teleop Init code
		{
			/**********************************************************************************************************************
			 * Action Code for Teleop Initialization
			 **********************************************************************************************************************/
	
		}

	public void teleopPeriodic() // Action routine: code called periodically during teleop
		{

		
			/**********************************************************************************************************************
			 * Variable Declarations for Teleop Periodic
			 **********************************************************************************************************************/
	
			/*****************************************
			 * XBox Controller assignments 1=A 2=B 3=X 4=Y 5=Left bumper 6= Right Bumper
			 * Axes2= Left Trigger Axes3= Right trigger
			 * 
			 * POV = -1 when not moving 0 = Top 90 = Right 180 = Bottom 270 = Left
			 *****************************************/
			xBoxAnalog[0] = xBox.getRawAxis(0); 				// Left joystick left-right motion
			xBoxAnalog[1] = xBox.getRawAxis(1); 				// Left joystick up-down motion
			xBoxAnalog[2] = xBox.getRawAxis(2); 				// Press Left Trigger to adjust speed of climber motor
			xBoxAnalog[3] = xBox.getRawAxis(3); 				// Press Right Trigger to turn on shooter if trigger passes threashold
			xBoxAnalog[4] = xBox.getRawAxis(4); 				// Right joystick left-right motion
			xBoxAnalog[5] = xBox.getRawAxis(5); 				// Right joystick up-down motion
	
			xBoxButtons[1] = xBox.getRawButton(1); 				// Press button 1 "A" on
			xBoxButtons[2] = xBox.getRawButton(2); 				// Press button 2 "B" on
			xBoxButtons[3] = xBox.getRawButton(3); 				// Press button 3 "X" on
			xBoxButtons[4] = xBox.getRawButton(4); 				// Press button 4 "Y" on
			xBoxButtons[5] = xBox.getRawButton(5); 				// Press button 5 "Left Bumper" on
			xBoxButtons[6] = xBox.getRawButton(6); 				// Press button 6 "Right Bumper" on
			xBoxPOV = xBox.getPOV(); 							// Read the flat joystick POV on the X-Box controller
	
			/*****************************************
			 * Joystick variable assignments
			 *****************************************/
			double stickScale; 									// 0-1.0 Linearly scale all axis of motion
			double xAxis = stick.getRawAxis(0); 				// L:-1.0 - R:1.0 Left-to-Right motion
			double yAxis = stick.getRawAxis(1); 				// F:-1.0 - R:1.0 Forward-to-Reverse motion
			double zAxis = stick.getRawAxis(2); 				// L:-1.0 - R:1.0 Twist CCW-to-CW
			double slider = stick.getRawAxis(3); 				// F:-1.0 - R:1.0 Slider forward-to-reverse
			stickButtons[1] = stick.getRawButton(1); 			// Press joystick trigger button
			stickButtons[2] = stick.getRawButton(2); 			// Press joystick button 2
			stickButtons[3] = stick.getRawButton(3); 			// Press joystick button 3
			stickButtons[4] = stick.getRawButton(4); 			// Press joystick button 4
			stickButtons[5] = stick.getRawButton(5); 			// Press joystick button 5
			stickButtons[6] = stick.getRawButton(6); 			// Press joystick button 6
			stickButtons[7] = stick.getRawButton(7); 			// Press joystick button 7
			stickButtons[8] = stick.getRawButton(8); 			// Press joystick button 8
			stickButtons[9] = stick.getRawButton(9); 			// Press joystick button 9
			stickButtons[10] = stick.getRawButton(10); 			// Press joystick button 10
			stickButtons[11] = stick.getRawButton(11); 			// Press joystick button 11
			stickButtons[12] = stick.getRawButton(12); 			// Press joystick button 12
	
			leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 	// Open loop ramp rate
			rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs); // Open loop ramp rate
	
			/**********************************************************************************************************************
			 * Action Code for Teleop Periodic
			 **********************************************************************************************************************/
	
			/****************************************************************************************************
			 ** Read raw gyro angle from the input mod. Compensate for gyro drift by adding
			 * an offset value to Compensate for gyro drift by adding an offset value to
			 * summation gyroDriftTotal Scale +/- gyro values and compensate for rollover so
			 * scalled to 0-360 degrees and
			 ****************************************************************************************************/
			gyroPitch = navxGyro.getPitch(); 						// Get gyro pitch because roboRIO is mounted on side
			gyroYaw = navxGyro.getYaw(); 							// Read gyro yaw value from navx board
			gyroDriftTotal = gyroDriftTotal + gyroDriftIncrement; 	// Keep track of total gyro angle drift
			gyroWithDrift = gyroYaw + gyroDriftTotal; 				// Real angle is read gyro angle plus drift compensation
	
			if (gyroWithDrift < 0)
				gyroAngleMod = 180 + (180 + gyroWithDrift); 		// gyro raw -179 to 0 = gyro compass angle +181 to 359
			else
				gyroAngleMod = gyroWithDrift; 						// gyro raw 0 to 179 = gyro compass angle 0 to 179
			gyroAngleOffset = gyroAngleMod - firstGyroScan; 		// Normalize gyro first scan to be zero
	
			if (gyroAngleOffset < 0)
				gyroAngleFB = gyroAngleOffset + 360;
			else
				gyroAngleFB = gyroAngleOffset;
	
			// System.out.println(" Gyro Yaw= " + gyroYaw + "FB: " + gyroAngleFB);
	
			/*****************************************
			 * Turn on the air compressor
			 *****************************************/
			airCompressor.setClosedLoopControl(true); 				// Enable Air Compressor to be controlled by pressure switch input
	
			/**************************************************
			 * Power Distribution Panel Read the current from each channel
			 *
			 * Get Talon motors position and speed feedback Use for calculating motor speed
			 * references
			 ***************************************************/
			// currentFdbkPDP[0] = ourPDP.getCurrent(0); //Get PDP motor channel 0 current
			// currentFdbkPDP[1] = ourPDP.getCurrent(1); //Get PDP motor channel 1 current
			// currentFdbkPDP[2] = ourPDP.getCurrent(2); //Get PDP motor channel 2 current
			// currentFdbkPDP[3] = ourPDP.getCurrent(3); //Get PDP motor channel 3 current
			// currentFdbkPDP[4] = ourPDP.getCurrent(4); //Get PDP motor channel 4 current
			// currentFdbkPDP[5] = ourPDP.getCurrent(5); //Get PDP motor channel 5 current
			// currentFdbkPDP[6] = ourPDP.getCurrent(6); //Get PDP motor channel 6 current
			// currentFdbkPDP[7] = ourPDP.getCurrent(7); //Get PDP motor channel 7 current
			// currentFdbkPDP[8] = ourPDP.getCurrent(8); //Get PDP motor channel 8 current
			// currentFdbkPDP[9] = ourPDP.getCurrent(9); //Get PDP motor channel 9 current
			// currentFdbkPDP[10] = ourPDP.getCurrent(10); //Get PDP motor channel 10 current
			// currentFdbkPDP[11] = ourPDP.getCurrent(11); //Get PDP motor channel 11 current
			// currentFdbkPDP[12] = ourPDP.getCurrent(12); //Get PDP motor channel 12 current
			// currentFdbkPDP[13] = ourPDP.getCurrent(13); //Get PDP motor channel 13 current
			// currentFdbkPDP[14] = ourPDP.getCurrent(14); //Get PDP motor channel 14 current
			// currentFdbkPDP[15] = ourPDP.getCurrent(15); //Get PDP motor channel 15 current
			leftMotorVelocity = leftMotor.getSelectedSensorVelocity(0); 		// Get Left drive motor Speed from Talon SRX
			leftMotorPosition = leftMotor.getSelectedSensorPosition(0); 		// Get Left drive motor Position from Talon SRX
			rightMotorVelocity = rightMotor.getSelectedSensorVelocity(0); 		// Get Right drive motor Speed from Talon SRX
			rightMotorPosition = rightMotor.getSelectedSensorPosition(0); 		// Get Right drive motor Position from Talon SRX
			winchMotorVelocity = winchMotor.getSelectedSensorVelocity(0); 		// Get Winch drive motor Speed from Talon SRX
			winchMotorPosition = winchMotor.getSelectedSensorPosition(0); 		// Get Winch drive motor Position from Talon SRX
			winchPositionErr = winchMotor.getClosedLoopError(0);				// Get Winch Closed Loop Position Error from Talon SRX
			winchMotorCurrent = currentFdbkPDP[2]; 								// Assign current feedback
	
			/***************************************************
			 * Get navX Gyro data Use for calculating motor speed references
			 ***************************************************/
			gyroCompass = navxGyro.getCompassHeading();
			gyroPitch = navxGyro.getPitch();
			gyroRoll = navxGyro.getRoll();
			gyroYaw = navxGyro.getYaw();
	
			/********************************************************
			 * X-BOX DIGITAL TRIGGER BUTTONS 5 & 6 ON TOP Press X-BOX controller Trigger
			 * button on top for solenoids Left to Close Pickup Arms 
			 * Right to Open Pickup Arms
			 ********************************************************/
			if (xBoxButtons[5]) 
				{
					intakeArmOpenCommand = false; 					// CLOSE the takeup arms
				}
			if (xBoxButtons[6]) 
				{
					intakeArmOpenCommand = true; 					// OPEN the takeup arms
				}
			intakeArmOpen.set(intakeArmOpenCommand); 				// Intake arms Close solenoid
			intakeArmClose.set(!(intakeArmOpenCommand)); 			// Intake arms Open solenoid
	
			/**********************************************************
			 * X-BOX LEFT & RIGHT ANALOG TRIGGER BUTTONS ON BOTTOM - PWM 0 & 1 Speed
			 * reference for cube pick-up motors Pressing left trigger more increases speed
			 * forward to pick-up Pressing right trigger more increases speed reverse to
			 * spit out
			 **********************************************************/
			if (xBoxAnalog[2] > 0.05) 
				{
					cubePickUpMotorSpdRef = 1.0 * xBoxAnalog[2]; 	// Forward speed reference for cube pick-up
				}
			if (xBoxAnalog[3] > 0.05) 
				{
					cubePickUpMotorSpdRef = -1.0 * xBoxAnalog[3]; 	// Reverse speed reference for cube pick-up
				}
			if ((xBoxAnalog[2] <= 0.05) & (xBoxAnalog[3] <= 0.05)) 	// Create a deadband +/- 0/05 around zero
				{
					cubePickUpMotorSpdRef = 0; 						// Set speed reference to zero if in deadband
				}
	
			/****************************************************************************************************
			 * Talon/Victor motors ID = 1 & 2 and 3 & 4 JOYSTICK POSITION processed with
			 * deadband filters to provide reference to Arcade style Drive Talon 1&2 (left)
			 * & 3&4 (right) X axis = turn left-right movement, Y axis = forward movement, Z
			 * axis = rotation
			 ****************************************************************************************************/
			stickScale = (slider + 1) / 2; 							// L:-1.0 - R:1.0 = 0-1.0 Linearly scale all axis of motion
	
			/****************************************************************************************************
			 * Add a dead band of movement on x-axis left-to-right
			 ****************************************************************************************************/
			if (xAxis < 0) 										// x-Axis side-to-side from joystick = -1.0 to 0
				{
					if (xAxis <= -deadBandPercentX)
						xAxisMod = xAxis + deadBandPercentX; 	// x-Axis side-to-side outside of deadband, subtract deadband to remove DB gap
					else
						xAxisMod = 0; 							// x-Axis side-to-side in deadband, set to zero
				} 
			else 												// x-Axis side-to-side from joystick = 0 to 1.0
				{
					if (xAxis >= deadBandPercentX)
						xAxisMod = xAxis - deadBandPercentX; 	// x-Axis side-to-side outside of deadband, subtract deadband to remove DB gap
					else
						xAxisMod = 0; // x-Axis side-to-side in deadband, set to zero
				}
	
			if (xAxisMod <= 0) 									// Left turn - Speed up right side to turn to left
				{
					left_x_axisTurnMod = 0;
					right_x_axisTurnMod = 0.9 * xAxisMod; 		// Speed up right side to turn to left
				} 
			else 												// Right turn - Speed up left side to turn to right
				{
					left_x_axisTurnMod = -0.9 * xAxisMod; 		// Speed up left side to turn to right
					right_x_axisTurnMod = 0;
				}
	
			/****************************************************************************************************
			 * Forward and Backward movement on y-axis
			 ****************************************************************************************************/
			yAxisMod = yAxis; // Allow normal front-to-back movement
	
			/****************************************************************************************************
			 * Add a dead band of movement on z-axis twist (Z-axis value from joystick =
			 * -1.0 - 1.0)
			 ****************************************************************************************************/
			if (zAxis < 0) 									// z-Axis twist from joystick = -1.0 to 0
				{
					if (zAxis <= -deadBandPercentZ)
						zAxisMod = zAxis + deadBandPercentZ; // z-Axis outside of deadband, subtract deadband to remove DB gap
					else
						zAxisMod = 0; 						// z-Axis in deadband, set to zero
				} 
			else 											// z-Axis twist from joystick = 0 to 1.0
				{
					if (zAxis >= deadBandPercentZ)
						zAxisMod = zAxis - deadBandPercentZ; // z-Axis outside of deadband, subtract deadband to remove DB gap
					else
						zAxisMod = 0; 						// z-Axis in deadband, set to zero
				}
			/*****************************************
			 * Rotate the robot Counter Clock Wise
			 *****************************************/
			if (zAxisMod <= 0) 
				{
					left_z_axisRotMod = -0.9 * zAxisMod; 	// Left side motor reverse
					right_z_axisRotMod = 0.9 * zAxisMod; 	// Right side motor forward
				}
			/*****************************************
			 * Rotate the robot Clock Wise
			 *****************************************/
			else 
				{
					left_z_axisRotMod = -0.9 * zAxisMod; 	// Left side motor reverse
					right_z_axisRotMod = 0.9 * zAxisMod; 	// Right side motor forward
				} 											// End of Left/Right drive motors speed reference from JOYSTICK
			/********** END of JOYSTICK position calculating **********/
	
			/********************************************************
			 * X-Box LEFT JOYSTICK Winch motor manual control with x-box left joystick
			 * up-down motion
			 ********************************************************/
			if ((xBoxAnalog[1] > deadBandWinch) | (xBoxAnalog[1] < -1.0 * deadBandWinch)) // joystick value is outside zero deadband
				{
					winchMotorSpdRef = -1.0 * xBoxAnalog[1]; // Winch motor manual speed refernce
				} 
			else 											// joystick value is inside zero deadband so ignore value
				{
					winchMotorSpdRef = 0; 					// Winch speed ref is zero
				}
	
			/********************************************************
			 * X-BOX POV - INTAKE ARM PIVOT UP/DOWN Raise Arm Pivot if (POV>270) OR (POV<90)
			 * AND (POV>-1) Lower Arm Pivot if (POV>90) AND (POV<270) AND (POV>-1)
			 ********************************************************/
			if ((xBoxPOV > -1) && ((xBoxPOV > 270) || (xBoxPOV < 90))) 
				{
					intakeArmPivotDownCommand = false; 			// Latch intake arm pivot to UP
				}
			if ((xBoxPOV > -1) && (xBoxPOV > 90) && (xBoxPOV < 270)) 
				{
					intakeArmPivotDownCommand = true; 			// Latch intake arm pivot to UP
				}
			intakeArmPivotDOWN.set(intakeArmPivotDownCommand); 	// Enable Intake Arm to pivot DOWN
			intakeArmPivotUP.set(!(intakeArmPivotDownCommand)); // Enable Intake Arm to pivot UP
	
			/********************************************************
			 * JOYSTICK DIGITAL TRIGGER BUTTON ON TOP Press joystick controller Trigger
			 * button to fire both left and right motor high speed solenoid to extend
			 ********************************************************/
			leftMotorShiftHigh.set(stickButtons[1]); 			// Enable high speed gear on Left/Right Drive motors
			leftMotorShiftLow.set(!(stickButtons[1])); 			// Disable high speed gear on Left/Right Drive motors
	
			/********************************************************
			 * JOYSTICK LEFT SIDE BUTTONS 7 & 8 Press Button 7 to RUN (Disengage Lock) Press
			 * Button 8 to LOCK (Cannot run elevator
			 ********************************************************/
			if (stickButtons[7]) 
				{
					//winchMotor.setSelectedSensorPosition(0, 0, winchKTimeoutMs); 		// Zero the position encoder
				}
			if (stickButtons[8]) 
				{
					winchMotor.setSelectedSensorPosition(0, 0, winchKTimeoutMs); 		// Zero the position encoder
				}
	
			/***************************************************
			 * Calculate left & right motor speed references
			 ***************************************************/
			leftMotorSpdRef = 1.0 * (yAxis + left_x_axisTurnMod + left_z_axisRotMod); 		// Left side motor speed reference
			rightMotorSpdRef = -1.0 * (yAxis + right_x_axisTurnMod + right_z_axisRotMod); 	// Right side motor speed reference
	
			/***************************************************
			 * Assign speed references to PWM motors
			 ***************************************************/
			leftIntake.set(-1.0 * cubePickUpMotorSpdRef); 									// Assign speed ref to Left PWM Talon
			rightIntake.set(-1.0 * cubePickUpMotorSpdRef); 									// Assign speed ref to Right PWM Talon
	
			/***************************************************
			 * Assign speed references to CAN bus motors
			 ***************************************************/
			if (winchMotorPosition > winchSwitchRef) 
				{
					leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 						// Rate = [1023-0] / [500 msec] * 10 msec = 20
					rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs); 					// Rate = [1023-0] / [500 msec] * 10 msec = 20
				} 
			else 
				{
					leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs); 						// Rate = [1023-0] / [500 msec] * 10 msec = 20
					rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs); 					// Rate = [1023-0] / [500 msec] * 10 msec = 20
				}
			leftMotor.set(ControlMode.PercentOutput, (stickScale * leftMotorSpdRef)); 		// Assign speed ref to Left CAN bus Talon
			leftSlaveMotor.follow(leftMotor); 												// Assign Left Slave motor to follow Left Talon motor
			rightMotor.set(ControlMode.PercentOutput, (stickScale * rightMotorSpdRef)); 	// Assign speed ref to Right CAN bus Talon
			rightSlaveMotor.follow(rightMotor); 											// Assign Right Slave motor to follow Left Talon motor
	
			/*****************************************
			 * WINCH MOTOR POSITION REFERENCES
			 * 
			 * Press X-Box controller button[1] "A" to start motion magic profile to run
			 * winch motor to Bottom position
			 * 
			 * Press X-Box controller button[2] "B" to start motion magic profile to run
			 * winch motor to Scale position
			 * 
			 * Press X-Box controller button[3] "X" to start motion magic profile to run
			 * winch motor to Switch position
			 * 
			 * JOYSTICK DIGITAL BUTTON 11 Press joystick controller button 11 to start winch
			 * motor climbing
			 *****************************************/
			if (xBoxButtons[1]) 								// "A" = Drive the elevator to Bottom Position
				{
					winchManualOffset = 0;
					winchPresetPosition = winchBottomRef; 		// Assign absolute position to move to BOTTOM
					enableClimbMode = false; 					// Disable Climbing mode if accidently set
				} 
			else if (xBoxButtons[2]) 							// "B" = Drive the elevator to Scale Position
				{
					winchManualOffset = 0;
					winchFeedFwdFix = 0;
					winchPresetPosition = winchScaleRef; 		// Assign absolute position to move to SCALE
					enableClimbMode = false; 					// Disable Climbing mode if accidently set
				} 
			else if (xBoxButtons[3]) 							// "X" = Drive the elevator to Scale Position
				{
					winchManualOffset = 0;
					winchFeedFwdFix = 0;
					winchPresetPosition = winchSwitchRef; 		// Assign absolute position to move to SWITCH
					enableClimbMode = false; 					// Disable Climbing mode if accidently set
				} 
			else 
				{
					if ( ((winchManualOffset + winchPresetPosition) < winchMaxHeight) || (winchMotorSpdRef < 0) )
					{
						winchManualOffset = winchManualOffset + (winchManualModeInc * winchMotorSpdRef);	
					}
					// winchMotor.set(ControlMode.PercentOutput, (stickScale * winchMotorSpdRef)); //Assign speed ref to Winch CAN bus Talon
				}
	
			winchTargetAbsolutePosition = winchPresetPosition + winchFeedFwdFix + winchManualOffset;
	
			if (xBoxButtons[4]) 								// Press the X-Box "Y" button for MANUAL MODE OPERATION
				winchMotor.set(ControlMode.PercentOutput, (0.65 * winchMotorSpdRef)); // Assign speed ref to Winch CAN bus Talon
	
			else
				winchMotor.set(ControlMode.MotionMagic, winchTargetAbsolutePosition); // Start magic motion for absolute position move
	
			// winchMotor.set(ControlMode.Position, winchTargetAbsolutePosition);
			winchSlaveMotor.follow(winchMotor); // Assign Winch Slave motor to follow Left Talon motor
	
			/***************************************************
			 * Output to console: buttons and pads from xbox and joystick
			 ***************************************************/
			// System.out.println( "X-Box Left stick left-right = " + xBoxAnalog[0]);
			// System.out.println( "X-Box Left stick up-down = " + xBoxAnalog[1]);
			// System.out.println( "Left Trigger= " + xBoxAnalog[2]);
			// System.out.println( "Right Trigger= " + xBoxAnalog[3]);
			// System.out.println( "X-Box Right stick left-right = " + xBoxAnalog[4]);
			// System.out.println( "X-Box Right stick up-down = " + xBoxAnalog[5]);
			// System.out.println("Button A " + xBoxButtons[1]);
			// System.out.println("Button B " + xBoxButtons[2]);
			// System.out.println("Joystick Trigger " + stickButtons[1]);
	
			/***************************************************
			 * Output to console: Talon motors position and speed feedback
			 ***************************************************/
			// System.out.println((currentSystemClock - firstClockScan) + ", " + slider);
			// System.out.println("System Clock = " + currentSystemClock);
			// System.out.println("Climb Start Time = " + climbTimeStartSnapshot);
			// System.out.println("Climb Time = " + climbTimeElapsed);
			// System.out.println("leftMotorSpdRef = " + leftMotorSpdRef);
			// System.out.println("rightMotorSpdRef = " + rightMotorSpdRef);
			// System.out.println("Left Motor Position = " + leftMotorPosition);
			// System.out.println("Left Motor Velocity = " + leftMotorVelocity);
			// System.out.println("Right Motor Position = " + rightMotorPosition);
			// System.out.println("Right Motor Velocity = " + rightMotorVelocity);
			 System.out.println("Winch Motor Position = " + winchMotorPosition);
			// System.out.println("Winch Position Setpoint = " +
			// winchTargetAbsolutePosition);
			// System.out.println("Winch Position Error = " + winchPositionErr);
			// System.out.println("Winch Motor Velocity = " + winchMotorVelocity);
			// System.out.println("Winch Manual Ref = " + winchManualOffset);
			// System.out.println("Winch Motor Current = " + currentFdbkPDP[2]);
	
			// System.out.println("Gyro Compass= " + gyroCompass);
			// System.out.println("Gyro Pitch= " + gyroPitch);
			// System.out.println("Gyro Roll= " + gyroRoll);
			// System.out.println("Gyro Yaw= " + gyroYaw);
			// System.out.println("Gyro Angle= " + gyroAngleFB);
			// System.out.println("Cube pickup ref = " + cubePickUpMotorSpdRef);
			// System.out.println("xBox POV = " + xBoxPOV);
			// System.out.println("Arm Pivot = " + intakeArmPivotDownCommand);
			// System.out.println("Arm Close = " + intakeArmOpenCommand);
			// System.out.println("match start position = " + matchStartPosition);
	
			// System.out.println("In 0 = " + digitalIn0.get());
			// System.out.println("In 1 = " + digitalIn1.get());
			// System.out.println("In 2 = " + digitalIn2.get());
	
			updateNetworkTables();
		} // End of Teleop Periodic
	
	public void testPeriodic() 
		{
			updateNetworkTables();
	
		} // End of Test Periodic

	/***************************************************
	 * Populate the network tables with data. We can pull this data from the network
	 * tables and place it on the Shuffleboard. This is placed inside Robot, but is
	 * its own method that can be called from the other iterative (periotic and
	 * init) methods Simply call `updateNetworkTables()` in each method to place
	 * this data in the network tables. Add values by using
	 * `SmartDashboard.put[type]. For example: SmartDashboard.putBoolean()
	 * SmartDashboard.putNumber() // Doubles SmartDashboard.putString()
	 * 
	 * After these are placed, the Shuffleboard widgets can pull data from the
	 * network table entries.
	 ***************************************************/
	// Suppress warnings about deprecated methods.
	@SuppressWarnings("deprecation")
	public void updateNetworkTables() {
		// Put battery voltage (WARNING: It's deprecated)
		SmartDashboard.putNumber("Battery Voltage", DriverStation.getInstance().getBatteryVoltage());
		// Put the game data
		SmartDashboard.putString("Game data", DriverStation.getInstance().getGameSpecificMessage());
		// Get gyro readings
		gyroCompass = navxGyro.getCompassHeading();
		gyroPitch = navxGyro.getPitch();
		gyroRoll = navxGyro.getRoll();
		gyroYaw = navxGyro.getYaw();
		// Put gyro readings
		SmartDashboard.putNumber("Gyro/Pitch", gyroPitch);
		SmartDashboard.putNumber("Gyro/Roll", gyroRoll);
		SmartDashboard.putNumber("Gyro/Yaw", gyroYaw);
		SmartDashboard.putNumber("Gyro/Compass", gyroCompass);
		// Get PDP readings - WARNING: THIS MAY NOT WORK. Last time I tried it, it
		// didn't work.
		// IF THIS FAILS: Comment out the next 4 lines to revert back to a working state
		// double PDPTemperature = PDPJNI.getPDPTemperature(module);
		// double PDPChannelCurrent1 = PDPJNI.getPDPChannelCurrent(channel[1], module);
		// Put PDP readings
		// SmartDashboard.putNumber("PDP/Temperature", PDPTemperature);
		// SmartDashboard.putNumber("PDP/Current/Channel 1", PDPChannelCurrent1);

		// ByteBuffer status = ByteBuffer.allocateDirect(4);
		// status.order(ByteOrder.LITTLE_ENDIAN);
		// double voltage=PDPJNI.getPDPVoltage(status.asIntBuffer());
		// double voltage = PDPJNI.getPDPVoltage(59);
		// System.out.println(voltage);
	} // end of updateNetworkTables

	// *****************************EXPERIMENTAL*****************************\\
	/**
	 * An experimental method that will allow us to place values in the network
	 * tables depending on what mode we are in. Simply call it like so:
	 * _updateNetworkTables(_driveMode.auton); // If we are in auton
	 * _updateNetworkTables(_driveMode.teleop); // If we are in teleop // Etc...
	 * 
	 * @param driveMode
	 *            The drive mode to set network table values according to.
	 */
	public void _updateNetworkTables(_driveMode driveMode) {
		switch (driveMode) {
		case auton:
			break;
		case test:
			break;
		case teleop:
			break;
		default:
			break;
		}
	}

	/**
	 * The possible drive modes.
	 */
	public enum _driveMode {
		auton, test, teleop, // Call in the corresponding method.
		init // Call in all init classes.
	}
	// *************************END EXPERIMENTAL*****************************\\

	
	/****************************************************************************************************
	 * syncAngle class Subroutine
	 ** 
	 ** Input: Angle setpoint in degrees
	 *         Angle Feedback in degrees
	 *         
	 *  Output: motor correction to be ADDED to left motor (scaled to speed ref (-1.0 to 1.0 = full min-to-full max
	 ****************************************************************************************************/
	public static double syncAngle(double angleSP, double angleFB) 
		{
			double syncAngleOut = 0;				// calc'ed output for method
			double angleErr = 0; 					// calc'ed angle err (SP-FB)
			double angleErrAdj = 0; 				// error adjusted to be +/- 180
			double angleErrMod = 0; 				// Angle Error modified by scale factor
			double absAngleErrMod = 0; 				// Absolute Value of Angle Error modified by scale factor
			double absSyncAngleOut = 0; 			// Absolute value of reference to move robot
			double correctDirection = 1.0; 			// Direction to rotate for shortest distance to setpoint
			final double zAxisCorrectScale = .022; 	// Scale gyro rotation drift to send ref to z-axis
			final double gyroMaxZaxisCorrect = 0.24; // Maximum amount of z-axis correction due to rotation drift
			final double minRotationThreash = 0.01; // Min Z-Axis rotation threashold to try to correct for
			final double minSpeedRef = 0.035; 		// Min speed reference to move robot
	
			angleErr = (angleSP - angleFB); 		// calc angle error modified value to get back on course
	
			angleErrAdj = angleErr;
			if ((angleSP > angleFB) && (angleFB < 180) && (angleSP > 180))
				angleErrAdj = angleErr - 360;
			if ((angleSP <= angleFB) && (angleFB >= 180) && (angleSP < 180))
				angleErrAdj = angleErr + 360;
	
			angleErrMod = angleErrAdj * zAxisCorrectScale; // calc angle error modified value to get back on course
			absAngleErrMod = Math.abs(angleErrMod); // Find absolute value of angle error modified by scale factor
	
			// System.out.println("angleSP=" + angleSP);
			// System.out.println("angleFB=" + angleFB);
			// System.out.println("angleErr" + angleErr);
			// System.out.println("angleErrAdj=" + angleErrAdj);
			// System.out.println("angleErrMod=" + angleErrMod);
			// System.out.println("");
	
			/****************************************************************************************************
			 ** Max positive correction is limited to max positive correction reference value
			 ** Max negative correction is limited to max negative correction reference value
			 ****************************************************************************************************/
			if (absAngleErrMod >= gyroMaxZaxisCorrect) 										// Limit max output
				{
					if (angleErrMod > 0) 													// Max positive reference
						syncAngleOut = gyroMaxZaxisCorrect;	 								// Limit angular reference to send to z-axis
					else 																	// Max negative reference
						syncAngleOut = (-1.0 * gyroMaxZaxisCorrect);						// Limit angular reference to send to z-axis
					// System.out.println("max correct=" + syncAngleOut);
				}
			/****************************************************************************************************
			 ** In between Max correction and Min correction - reference is scaled with angle
			 * error value
			 ****************************************************************************************************/
			else if (absAngleErrMod < gyroMaxZaxisCorrect) // Error is less than max output limit
				syncAngleOut = angleErrMod; // Scale angular reference to send to z-axis
			/****************************************************************************************************
			 ** Correction is less than threshold to care about - reference is set to zero
			 ****************************************************************************************************/
			else if (absAngleErrMod < minRotationThreash) // Error is less than max output limit
				syncAngleOut = 0; // Scale angular reference to send to z-axis
			/****************************************************************************************************
			 ** If the speed reference is too low, the robot won't move so set set a min speed ref
			 ****************************************************************************************************/
			absSyncAngleOut = Math.abs(syncAngleOut); // Absolute value of speed reference
			if (absSyncAngleOut < minSpeedRef) // Speed reference is less than value to cause movement
				{
					if (syncAngleOut < 0.0) // Scale negative angular reference to send to z-axis
						syncAngleOut = (-1.0 * minSpeedRef);
					else
						syncAngleOut = minSpeedRef; // Scale positive angular reference to send to z-axis
				}
	
			return syncAngleOut; // Assign method returned value - must be last line
	
		}// end of syncAngle method

} // End of Robot class Iterative
