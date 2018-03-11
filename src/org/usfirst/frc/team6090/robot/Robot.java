package org.usfirst.frc.team6090.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;							//import Joystick method of WPIlib archive
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Victor;							//import Victor (non-CAN bus) method of WPIlib archive

import com.ctre.phoenix.motorcontrol.ControlMode;				//import CAN bus motor control mode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;			//import CAN bus Talon SRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;			//import CAN bus Victor SRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice;			//import CAN but Talon encoder FDBK
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.Solenoid;							//import control for solenoids
import edu.wpi.first.wpilibj.Compressor;						//import control for air compressor
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;						//import Camera method of WPIlib archive
import edu.wpi.first.wpilibj.PowerDistributionPanel;			//import interface for Power Distribution Panel
import com.kauailabs.navx.frc.AHRS;

/*****************************************
 * Suppress warnings:
 * - Unused variables and imports
 * - Unchecked types
 * - Raw types
 *****************************************/
@SuppressWarnings({"unused", "unchecked", "rawtypes"})

/**********************************************************************************************************************
 * Start of main class Robot
 **********************************************************************************************************************/
public class Robot extends IterativeRobot 
	{

		Joystick stick;										//Reserve memory for variable	
		Joystick xBox;										//Reserve memory for variable
		Victor leftIntake;									//Reserve memory for variable for Left Victor PWM controller
		Victor rightIntake;									//Reserve memory for variable for Right Victor PWM controller


		/*****************************************
		 * CAN bus motors
		 *****************************************/
		WPI_TalonSRX leftMotor;								//Reserve memory for variable for Left Talon CAN bus controller
		WPI_TalonSRX rightMotor;							//Reserve memory for variable for Right Talon CAN bus controller
		WPI_TalonSRX winchMotor;							//Reserve memory for variable for Winch Talon CAN bus controller
		WPI_VictorSPX leftSlaveMotor;						//Reserve memory for variable for Left Victor CAN bus controller
		WPI_VictorSPX rightSlaveMotor;						//Reserve memory for variable for Right Victor CAN bus controller
		WPI_VictorSPX winchSlaveMotor;						//Reserve memory for variable for Winch Victor CAN bus controller

		PowerDistributionPanel ourPDP;						//Reserve memory for variable for Power Distribution Panel
		double[] currentFdbkPDP = new double[16];			//Current feedback values from PDP
		
		double leftMotorVelocity = 0;						//Left drive motor speed
		double leftMotorPosition = 0;						//Left drive motor relative position
		double leftMotorPositionAut = 0;					//Left drive motor relative position
		int leftKTimeoutMs = 10;							//Reserve memory - 0=don't wait for confirmation, <>0 is wait 
		double rightMotorVelocity = 0;						//Right drive motor speed
		double rightMotorPosition = 0;						//Right drive motor relative position
		int rightKTimeoutMs = 10;							//Reserve memory - 0=don't wait for confirmation, <>0 is wait 
		double latchMotorVelocity = 0;						//Latch drive motor speed
		double latchMotorPosition = 0;						//Latch drive motor relative position
		int latchKTimeoutMs = 10;							//Reserve memory - 0=don't wait for confirmation, <>0 is wait 
		double yAxisRef=0;									//F:-1.0 - R:1.0 Forward-to-Reverse motion
		
		int winchKTimeoutMs = 10;							//Reserve memory - 0=don't wait for confirmation, <>0 is wait 
		double winchFeedFwdFix = 0;							//Feed forward gain messes with the final target position so adjust to get it back
		double winchManualOffset = 0;						//Change fixed positions with manual adjustments from the left x-box joystick
		double winchMotorVelocity = 0;						//Winch drive motor speed
		double winchMotorPosition = 0;						//Winch drive motor relative position
		double winchMotorCurrent = 0;						//Winch drive motor current
		double winchPositionErr = 0;						//Winch position error
		double winchPresetPosition = 0;						//Winch predefined position setpoints
		double winchTargetAbsolutePosition = 0;				//Reserve memory - Absolute target position in encoder cnts (0=bottom)
		
		
		/*****************************************
		 * Gyro Data
		 *****************************************/
		AHRS navxGyro;										//Navx Gryo module
		float gyroCompass;									//Gyro Compass heading as read from navx gyro
		float gyroPitch;									//Gyro Pitch read from navx gyro
		float gyroRoll;										//Gyro Roll read from navx gyro
		float gyroYaw;										//Gyro Yaw read from navx gyro
		double gyroDriftTotal;								//Total gyro drift while running
		final double gyroDriftIncrement = 0.0001688555;		//Amout of drift correction per scan of gyro
		double gyroWithDrift = 0;							//Gyro feedback value with drift added
		double firstGyroScan = 0;							//Gyro angle on first scan of autonomous
		double gyroAngleOffset = 0;							//Gyro angle with first scan angle added to it
		double gyroAngleFB;									//Gyro Feedback scaled to 0-359 degrees
		double gyroAngleSP;									//Gyro SetPoint scaled to 0-359 degrees
		double gyroAngleMod = 0.0;							//gyro angle modified for drift
		double gyroAngleSync = 0.0;							//New gyro angle sync reference
		int numRotations = 1;								//number of whole rotations of feedback

		
		
		
		/*****************************************
		 * Solenoid valves
		 *****************************************/
		Compressor airCompressor; 							//Reserve memory for Compressor on CAN bus ID=60
		Solenoid leftMotorShiftHigh;						//Reserve memory for variable for Left motor shift to High
		Solenoid leftMotorShiftLow;							//Reserve memory for variable for Left motor shift to Low
		Solenoid winchMotorLock;						//Reserve memory for variable for Winch motor shift to High
		Solenoid winchMotorRun;						//Reserve memory for variable for Winch motor shift to Low
		Solenoid intakeArmClose;							//Reserve memory for variable for Intake Arm Open
		Solenoid intakeArmOpen;								//Reserve memory for variable for Intake Arm Close
		Solenoid intakeArmPivotDOWN;						//Reserve memory for variable for Intake Arm Pivot DOWN
		Solenoid intakeArmPivotUP;							//Reserve memory for variable for Intake Arm Pivot UP
		
		
		/*****************************************
		 * Robot wheel speed control by joystick
		 *****************************************/
		double xAxisMod=0;									//Modified x-Axis position command to include deadband
		double yAxisMod=0;									//Modified y-Axis position command to include deadband
		double zAxisMod=0;									//Modified z-Axis position command modified to include deadband
		double leftMotorSpdRef=0;							//X-Axis speed command 
		double rightMotorSpdRef=0;							//Y-Axis speed command
		double leftMotorSpdRefAut=0;						//X-Axis speed command in Autonomous 
		double rightMotorSpdRefAut=0;						//Y-Axis speed command in Autonomous
		double left_x_axisTurnMod=0;						//Turning Left xAxis mod
		double right_x_axisTurnMod=0;						//Turning Right xAxis mod
		double left_z_axisRotMod=0;							//Rotate Left zAxis mod
		double right_z_axisRotMod=0;						//Rotate Right zAxis mod
		final double deadBandPercentX = 0.1;				//Dead band percent of X-Axis side-to-side from joystick
		final double deadBandPercentZ = 0.2;				//Dead band percent of Z-Axis twist from joystick
		final double deadBandWinch = 0.2;					//Dead band from x-box joystick for Winch manual speed control
		final double deadBandLatch = 0.2;					//Dead band from x-box joystick for Latch manual speed control
		
		double winchMotorSpdRef = 0;						//Winch motor manual mode speed reference
		double latchMotorSpdRef = 0;						//Latch motor manual mode speed reference
		double cubePickUpMotorSpdRef = 0;					//PWM motor speed reference for cube pick-up
		boolean intakeArmOpenCommand;						//Latched command to move the intake arms Closed
		boolean intakeArmPivotDownCommand;					//Latched command to move intake arm Pivot DOWN
		boolean winchRunCommand;							//Winch motor gearbox engaged to run mode
		boolean enableClimbMode;							//Climb mode started
		
		
		/*****************************************
		 * Digital Inputs
		 *****************************************/ 
		DigitalInput digitalIn0;							//Reserve memory for Digital Input 0
		DigitalInput digitalIn1;							//Reserve memory for Digital Input 1
		DigitalInput digitalIn2;							//Reserve memory for Digital Input 2
		DigitalInput digitalIn3;							//Reserve memory for Digital Input 3
		DigitalInput digitalIn4;							//Reserve memory for Digital Input 4
		DigitalInput digitalIn5;							//Reserve memory for Digital Input 5
		
		/*****************************************
		 * Position SETPOINTS
		 *****************************************/
		final double winchBottomRef = 400;					//Winch Bottom position ref in encoder quad counts (old 300)
		final double winchScaleRef = 21000;					//Winch Scale position ref in encoder quad counts (old 7500)
		final double winchScaleTopRef = 24000;				//Winch Scale Top position ref in encoder quad counts (old 8400)
		final double winchSwitchRef = 7500;					//Winch Switch position ref in encoder quad counts (old 3200)
		
		boolean testStartLeftSwitchLeft = true;
		boolean testStartRightSwitchRight=false; 
		

		
		/*****************************************
		 * XBox controller
		 *****************************************/
		boolean[] xBoxButtons = new boolean[10];			//Reserve memory for variable for XBox controller buttons
		double[] xBoxAnalog = new double[10];				//Reserve memory for variable for XBox controller analog pads 
		boolean[] stickButtons = new boolean[20];			//Reserve memory for variable for Joystick controller buttons
		int xBoxPOV = 0; 									//Reserve memory for variable for XBox controller POV
		
		
		/*****************************************
		 * Time related variables
		 *****************************************/
		boolean autoFirstScan=false;						//First scan OFF, On for rest of running
		boolean climbStartTime = false;						//Snapshot time when climb mode started
		long currentSystemClock = 0;						//System clock time read
		long autoSequenceTime = 0;							//Autonomous mode sequence timer number
		long autoTimeStart=0;								//System clock time snapshot at start of autonomous
		long climbTimeStartSnapshot=0;						//System clock time snapshot at start of climb mode
		long climbTimeElapsed=0;							//Elapsed time to do climb
		boolean firstScan=false; 
		long firstClockScan=0;
		
		
		/*****************************************
		 * Camera setup variables
		 *****************************************/
		public UsbCamera cameraGear;						//Reserve memory for gear camera video stream
//		public UsbCamera cameraMain;						//Reserve memory for gear camera video stream
		final int kVideoWidth = 160;						// set dashboard default camera image width	
		final int kVideoHeight = 120;						// set dashboard default camera image height
		final int kVideoFramesPerSecond = 30;				// set dashboard default camera frames per second(fps)
		private int videoWidth;								//Store value of actual width of video stream
		private int videoHeight;							//Store value of actual height of video stream
		private int videoFPS;								//Store value of actual fps of video stream
		

		/*****************************************
		 * Read Game information
		 *****************************************/
		String gameData;									//3 letters from FMS; Our Switch side nearest us, Our Scale side, Enemy Switch side farthest away
		String ourGameData;									//Strip off the first 2 characters from the FMS string
		boolean robotStartLeft = false;						//Robot starts at left position
		boolean robotStartCenter = false;					//Robot starts at center position
		boolean robotStartRight = false;					//Robot starts at right position
		double matchStartPosition = -1;						//Robot Start position: 0=Left, 1=Center, 2=Right
		boolean goForScale = false;							//Team override: True = Force going for scale instead of switch
		
		
		/*****************************************
		 * Autonomous mode 
		 *****************************************/
		int startLeftSwitchLeftSeq = 0;							//Start on Left, Switch on Left Autonomous Sequence number
		long[] startLeftSwitchLeftTime = new long[20];			//Create array of time points when each sequence starts in autonomous
		int startLeftScaleLeftSeq = 0;							//Start on Left, Scale on Left Autonomous Sequence number
		long[] startLeftScaleLeftTime = new long[20];			//Create array of time points when each sequence starts in autonomous
		int startRightSwitchRightSeq = 0;						//Start on Right, Switch on Right Autonomous Sequence number
		long[] startRightSwitchRightTime = new long[20];		//Create array of time points when each sequence starts in autonomous
		int startRightScaleRightSeq = 0;						//Start on Right, Scale on Right Autonomous Sequence number
		long[] startRightScaleRightTime = new long[20];			//Create array of time points when each sequence starts in autonomous
		int startCenterSwitchLeftSeq = 0;						//Start on Center, Switch on Left Autonomous Sequence number
		long[] startCenterSwitchLeftTime = new long[20];		//Create array of time points when each sequence starts in autonomous
		int startCenterSwitchRightSeq = 0;						//Start on Center, Switch on Right Autonomous Sequence number
		long[] startCenterSwitchRightTime = new long[20];		//Create array of time points when each sequence starts in autonomous
		double autonMovementStartPosition = 0;					//Snapshot current position to get starting point for calibrated movement

		
		
		int count=0;
		
		public void robotInit() 
			{
			
				/**
				 * The List of options for auton start position
				 */
				SendableChooser autoChooser;
				count=0;

	    		stick = new Joystick(0);					//Create a new instance of Joystick called stick
	    		xBox =  new Joystick(1);					//Create a new instance of Joystick called xBox
		    	leftIntake  = new Victor(0); 				//Create a new instance for Left Victor PWM motor controller	
		    	rightIntake = new Victor(1);				//Create a new instance for Right Victor PWM motor controller

		    	
		    	leftMotor = 		new WPI_TalonSRX(1);			//Create a new instance for Left Talon SRX CAN bus motor controller
		    	leftSlaveMotor =	new WPI_VictorSPX(2);			//Create a new instance for Left Victor SPX CAN bus motor controller
		    	rightMotor =		new WPI_TalonSRX(3);			//Create a new instance for Right Talon SRX CAN bus motor controller
		    	rightSlaveMotor =	new WPI_VictorSPX(4);			//Create a new instance for Right Victor SPX CAN bus motor controller
		    	winchMotor = 		new WPI_TalonSRX(5);			//Create a new instance for Winch Talon SRX CAN bus motor controller
		    	winchSlaveMotor = 	new WPI_VictorSPX(6);			//Create a new instance for Left Victor SPX CAN bus motor controller
		    	ourPDP = 			new PowerDistributionPanel(59);	//Create a new instance for reading PDP values from CAN ID=59
		    	airCompressor = 	new Compressor(60); 			//Create a new instance for Compressor on CAN bus ID=60
		    	leftMotorShiftHigh=	new Solenoid(60, 0);			//Create a new instance for Left motor High Speed Sol shifter on Can ID=60
		    	leftMotorShiftLow =	new Solenoid(60, 1);			//Create a new instance for Left motor Low Speed Sol shifter on Can ID=60
		    	winchMotorLock=new Solenoid(60, 2);			//Create a new instance for Winch motor High Speed Sol shifter on CAN ID=60
		    	winchMotorRun=	new Solenoid(60, 3);			//Create a new instance for Winch motor Low Speed Sol shifter on CAN ID=60
		    	intakeArmClose = 	new Solenoid(60, 4);			//Create a new instance for Intake Arms Open Sol on CAN ID=60
		    	intakeArmOpen = 	new Solenoid(60, 5);			//Create a new instance for Intake Arms Close Sol on CAN ID=60
		    	intakeArmPivotDOWN=	new Solenoid(60, 6);			//Create a new instance for Intake Arms Pivot DOWN Sol on CAN ID=60
		    	intakeArmPivotUP=	new Solenoid(60, 7);			//Create a new instance for Intake Arms Pivot UP Sol on CAN ID=60
		    	
		    	
		    	leftMotor.setSensorPhase(true);												//Make sure talons blink green when position increasing
		    	leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);	//Left motor has a Quadrature encoder hooked up to talon
		    	leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);					//Zero the position encoder

		    	
		    	rightMotor.setSensorPhase(true);											//Make sure talons blink green when position increasing
		    	rightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);	//Right motor has a Quadrature encoder hooked up to talon   	
		    	rightMotor.setSelectedSensorPosition(0, 0, rightKTimeoutMs);				//Zero the position encoder

		    	/**********************************************************************************************
				 * Setup the closed loop parameters for the winch Talon.
				 * For the SelectProfileSlot: First parameter is setpoint slot (0-4)
				 * 							  Second parameter is PID loop number - usually 0 for primary loop
				 * 
				 * kFeed forward=(Percent motor load to achieve max speed) * 1023/max spd in encoder cnts/100ms
				 *   To set-run full speed in manual joystick mode and read talon speed feedback and output
				 *   percent from the Self-Test web interface roborio-6090-frc.local
				 **********************************************************************************************/
		    	/***************************************************************************
				 * Slot 0 is wench coarse movement UP and DOWN during Teleop
				 ***************************************************************************/
		    	winchMotor.selectProfileSlot(0, 0);									//Slot 0, PID loop number=0

		    	winchMotor.config_kF(0, 3.0, winchKTimeoutMs);						//Feed Forward gain - PID loop num, value, timout
		    	winchMotor.config_kP(0, 1.0, winchKTimeoutMs);						//Proportional gain - PID loop num, value, timout
		    	winchMotor.config_kI(0, .0, winchKTimeoutMs);						//Integral gain - PID loop num, value, timout
		    	winchMotor.config_kD(0, 0, winchKTimeoutMs);						//Differential gain - PID loop num, value, timout
		    	winchMotor.configAllowableClosedloopError(0, 50, winchKTimeoutMs);	//This doesn't appear to do anything useful
		    	winchMotor.config_IntegralZone(0, 5, winchKTimeoutMs);				//Enable integral component when error gets below this threshold
		    	winchMotor.configPeakCurrentDuration(50, winchKTimeoutMs);			//Number of msec to allow peak current
		    	winchMotor.configContinuousCurrentLimit(80, winchKTimeoutMs);		//Continuous allow current in Amps
		    	winchMotor.configPeakCurrentLimit(68, winchKTimeoutMs);				//Peak current in Amps
		    	winchMotor.configNeutralDeadband(0.03, winchKTimeoutMs);			//If error is below threshold, consider to be "in position" (0.25=25%)
		    	

		    		/***************************************************************************
		    		 * Set accel and cruise velocity to use to make trapezoid profile
		    		 * Cruise speed = 95% of max speed = 1400*0.95 = 1330
		    		 * Acc rate = Cruise spd in 0.5 sec = 1330/0.5 = 2660
		    		 ***************************************************************************/
		    	winchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);	//Winch motor has a magnetic pickup encoder hooked to talon
		    	winchMotor.setSensorPhase(false);				//Make sure talons blink green when position increasing
		    	winchMotor.setInverted(true);					//Invert the speed reference
		    	winchSlaveMotor.setInverted(true);				//Invert the speed reference (Slave must have same boolean value as master)
		    	
		    	/* Set relevant frame periods to be at least as fast as periodic rate */
		    	winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, winchKTimeoutMs);
		    	winchMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, winchKTimeoutMs);
		    	winchMotor.configMotionCruiseVelocity(1330, winchKTimeoutMs);	//Cruize velociy in encoder/100msec
		    	winchMotor.configMotionAcceleration(700, winchKTimeoutMs);		//Accel
		    	winchMotor.setSelectedSensorPosition(0, 0, winchKTimeoutMs);	//Zero the position encoder

		    		/*************************************************************************
		    		 * Set Open and Close Loop ramp rate -- Higher values give slower ramp--
		    		 * 
		    		 * Rate is change in output / 10msec chunks
		    		 * Max Talon speed reference = 1023 counts internally so speed change = end speed - start speed
		    		 *************************************************************************/
		    	winchMotor.configOpenloopRamp(0.5, winchKTimeoutMs);		//Rate = [1023-0] / [1000 msec] * 10 msec = 10
		    	winchMotor.configClosedloopRamp(0.5, winchKTimeoutMs);		//Rate = [1023-0] / [500 msec] * 10 msec = 20
		    	winchManualOffset = 0;										//Reset manual ref to zero on startup

				/*****************************************
				 * Shift winch to RUN elevator mode
				 ****************************************/ 		
				winchRunCommand = true;												//Shifter solenoid ON forward to RUN
				winchMotorRun.set(winchRunCommand);									//Intake arms Close solenoid
				winchMotorLock.set(!(winchRunCommand));								//Intake arms Open solenoid
		    	
		    	/*****************************************
				 * NavX MXP am-3060b plugged in roborio MXP
				 * port declaration. Plug in with micro-USB
				 * to verify operation and update firmware:
				 * hold cal button then plug in USB to update.
				 *****************************************/
				navxGyro = new AHRS(SPI.Port.kMXP);


				
	    		/***************************************************************************
	    		 * autoChooser is a created instance so Shuffleboard has a container to 
	    		 *   store values in
	    		 ****************************************************************************/
				SmartDashboard.putNumber("Auton/Match Start Position", -1);
				
				autoChooser = new SendableChooser();
				autoChooser.addObject("Left Position", false);					//Our robot start position is Left
				autoChooser.addObject("Center Position", false);				//Our robot start position is Center
				autoChooser.addObject("Right Position", false);					//Our robot start position is Right
				SmartDashboard.putData("Auton Start Position", autoChooser);
				
				SmartDashboard.putBoolean("Auton/Go for Scale", false);
				
				digitalIn0 = new DigitalInput(0);											//Assign digital input to variable
				digitalIn1 = new DigitalInput(1);											//Assign digital input to variable
				digitalIn2 = new DigitalInput(2);											//Assign digital input to variable
				digitalIn3 = new DigitalInput(3);											//Assign digital input to variable
				digitalIn4 = new DigitalInput(4);											//Assign digital input to variable
				digitalIn5 = new DigitalInput(5);											//Assign digital input to variable
				
				cameraGear = CameraServer.getInstance().startAutomaticCapture("lift Camera(0)",0);
				cameraGear.setFPS(kVideoFramesPerSecond);						// set Frames per second 
				cameraGear.setResolution(kVideoWidth, kVideoHeight);			// Set details camera resolution
				videoHeight = cameraGear.getVideoMode().height;					// Get values of video height. Needed for vision calculations
				videoWidth = cameraGear.getVideoMode().width;					// Get values of video width. Needed for vision calculations
				videoFPS = cameraGear.getVideoMode().fps;	
//				cameraMain = CameraServer.getInstance().startAutomaticCapture("Main Camera(0)",0);
//				cameraMain.setFPS(kVideoFramesPerSecond);						// set Frames per second 
//				cameraMain.setResolution(kVideoWidth, kVideoHeight);			// Set details camera resolution
//				videoHeight = cameraMain.getVideoMode().height;					// Get values of video height. Needed for vision calculations
//				videoWidth = cameraMain.getVideoMode().width;					// Get values of video width. Needed for vision calculations
//				videoFPS = cameraMain.getVideoMode().fps;						// Get values of video fps. Just for test purposes
				
				updateNetworkTables();
		    	
			} // End of RobotInit


		public void autonomousInit() 
			{
				updateNetworkTables();
				/**********************************************************************************************************************
				 * Variable Declarations for Autonomous Initialization
				 **********************************************************************************************************************/			
				
				winchMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
				autoFirstScan = false;										//Reset first scan latch bit
				
				/*****************************************
				 * The FMS gives a 3 letter setup message to
				 * indicate the position of your alliance colored
				 * Our Switch side nearest us, Our Scale side, Enemy Switch side farthest away
				 * 
				 *  LRL  for example: (near switch, scale, far switch)
				 *  (1st L) Our alliance switch nearest us is on the Left
				 *  (2nd R) Our alliance scale is on the right
				 *  (3rd L) Our alliance switch color farthest us is on the Left
				 *****************************************/
				gameData = DriverStation.getInstance().getGameSpecificMessage();	//Read from the driver station panel from the Field Management System
				ourGameData = gameData.substring(0, 2);								//Strip off the first 2 characters from the FMS string
				System.out.println("Our data= " + ourGameData);

				/*****************************************
				 * Turn on the air compressor
				 *****************************************/
				airCompressor.setClosedLoopControl(true);							//Enable Air Compressor to be controlled by pressure switch input
				
				/**********************************************************************************************************************
				 * Action Code for Autonomous Initialization
				 **********************************************************************************************************************/
	    		navxGyro.reset();
	    		leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);			//Zero the position encoder
	    		rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);			//Zero the position encoder
				startLeftSwitchLeftSeq = 0;											//Zero Start on Left, Switch on Left sequence number
				startRightSwitchRightSeq = 0;										//Zero Start on Right, Switch on Right sequence number
				startLeftScaleLeftSeq = 0;											//Zero Start on Left, Scale on Left sequence number
				startRightScaleRightSeq = 0;										//Zero Start on Right, Scale on Right sequence number
				startCenterSwitchLeftSeq = 0;										//Zero Start on Center, Scale on Left sequence number
				startCenterSwitchRightSeq = 0;										//Zero Start on Center, Scale on Right sequence number
				
				/*****************************************
				 * Shift winch to RUN elevator mode
				 ****************************************/ 		
				winchRunCommand = true;												//Shifter solenoid ON forward to RUN
				winchMotorRun.set(winchRunCommand);									//Intake arms Close solenoid
				winchMotorLock.set(!(winchRunCommand));								//Intake arms Open solenoid
	    		
	    		/***************************************************************************
	    		 * 
	    		 ***************************************************************************/
	    		String autonChooser = SmartDashboard.getString("Auton Start Position/selected", "Nothing selected");
	    		
	    		if (autonChooser.equals("Left Position")) 
	    			{
	    				SmartDashboard.putNumber("Auton/Match Start Position", 0);
	    			}
	    		if (autonChooser.equals("Center Position")) 
	    			{
	    				SmartDashboard.putNumber("Auton/Match Start Position", 1);
	    			}
	    		if (autonChooser.equals("Right Position")) 
	    			{
	    				SmartDashboard.putNumber("Auton/Match Start Position", 2);
	    			}
	    		matchStartPosition = SmartDashboard.getNumber("Auton/Match Start Position", -1);
				System.out.println("Match start postition = " + matchStartPosition);
				goForScale = SmartDashboard.getBoolean("Auton/Go for scale", false);

				if (!digitalIn0.get())
					matchStartPosition = 0;			//0 = Robot Start on LEFT
				if (!digitalIn1.get())
					matchStartPosition = 1;			//1 = Robot Start on CENTER	
				if (!digitalIn2.get())
					matchStartPosition = 2;			//2 = Robot Start on RIGHT	

				System.out.println("Match Pos = " + matchStartPosition );

				matchStartPosition = 2;		//TEST MODE ONLY 0=left, 1=center, 2=right
				goForScale = true;
	    		
			}	//End of Autonomous Initialization

		public void autonomousPeriodic()		//Action routine: code called periodically during Autonomous 
			{
			
			/**********************************************************************************************************************
			 * Variable Declarations for Autonomous Periodic
			 **********************************************************************************************************************/
			
			long[] startLeftSwitchLeft   =    new long[20];				//Create array of time points for autonomous mode for:start Left side, go for switch on Left side
			double startLeftSwitchLeftFirstPosition = 0;				//First movement position point	for:start Left side, go for switch on Left side
			double startLeftSwitchLeftFirstAngle = 0;					//First movement angle	for:start Left side, go for switch on Left side
			double startLeftSwitchLeftSecondPosition = 0;				//Second movement position point	for:start Right side, go for switch on Right side
			double startLeftSwitchLeftSecondAngle = 0;					//Second movement angle	for:start Left side, go for switch on Left side
			double startLeftSwitchLeftIntakeSpdRef = 0.0;				//Intake speed ref to spit out cube
			double startLeftSwitchLeftThirdPosition = 0;				//Third movement position point	for:start Left side, go for switch on Left side
			double startLeftSwitchLeftThirdAngle = 0;					//Third movement angle	for:start Left side, go for switch on Left side
			
			long[] startLeftScaleLeft   =    new long[20];				//Create array of time points for autonomous mode for:start Left side, go for Scale on Left side
			double startLeftScaleLeftFirstPosition = 0;					//First movement position point	for:start Left side, go for Scale on Left side
			double startLeftScaleLeftFirstAngle = 0;					//First movement angle	for:start Left side, go for Scale on Left side
			double startLeftScaleLeftSecondPosition = 0;				//Second movement position point	for:start Right side, go for Scale on Right side
			double startLeftScaleLeftSecondAngle = 0;					//Second movement angle	for:start Left side, go for Scale on Left side
			double startLeftScaleLeftIntakeSpdRef = 0.0;				//Intake speed ref to spit out cube
			double startLeftScaleLeftThirdPosition = 0;					//Third movement position point	for:start Left side, go for Scale on Left side
			double startLeftScaleLeftThirdAngle = 0;					//Third movement angle	for:start Left side, go for Scale on Left side
					
			long[] startRightSwitchRight =    new long[20];				//Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startRightSwitchRightFirstPosition = 0;				//First movement position point	for:start Right side, go for switch on Right side
			double startRightSwitchRightFirstAngle = 0;					//First movement angle	for:start Right side, go for switch on Right side
			double startRightSwitchRightSecondPosition = 0;				//Second movement position point	for:start Right side, go for switch on Right side
			double startRightSwitchRightSecondAngle = 0;				//Second movement angle	for:start Right side, go for switch on Right side
			double startRightSwitchRightIntakeSpdRef = 0.0;				//Intake speed ref to spit out cube
			double startRightSwitchRightThirdPosition = 0;				//Third movement position point	for:start Right side, go for switch on Right side
			double startRightSwitchRightThirdAngle = 0;					//Third movement angle	for:start Right side, go for switch on Right side

			long[] startRightScaleRight =    new long[20];				//Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startRightScaleRightFirstPosition = 0;				//First movement position point	for:start Right side, go for scale on Right side
			double startRightScaleRightFirstAngle = 0;					//First movement angle	for:start Right side, go for scale on Right side
			double startRightScaleRightSecondPosition = 0;				//Second movement position point	for:start Right side, go for switch on Right side
			double startRightScaleRightSecondAngle = 0;					//Second movement angle	for:start Right side, go for scale on Right side
			double startRightScaleRightIntakeSpdRef = 0.0;				//Intake speed ref to spit out cube
			double startRightScaleRightThirdPosition = 0;				//Third movement position point	for:start Right side, go for scale on Right side
			double startRightScaleRightThirdAngle = 0;					//Third movement angle	for:start Right side, go for scale on Right side

			long[] startCenterSwitchLeft   =    new long[20];			//Create array of time points for autonomous mode for:start Left side, go for switch on Left side
			double startCenterSwitchLeftFirstPosition = 0;				//First movement position point	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftFirstAngle = 0;					//First movement angle	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSecondPosition = 0;				//Second movement position point	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftSecondAngle = 0;				//Second movement angle	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftIntakeSpdRef = 0.0;				//Intake speed ref to spit out cube
			double startCenterSwitchLeftThirdPosition = 0;				//Third movement position point	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftThirdAngle = 0;					//Third movement angle	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftForthPosition = 0;				//Forth movement position point	for:start Center side, go for switch on Left side
			double startCenterSwitchLeftForthAngle = 0;					//Forth movement angle	for:start Center side, go for switch on Left side
			
			long[] startCenterSwitchRight =    new long[20];			//Create array of time points for autonomous mode for:start Right side, go for switch on Right side
			double startCenterSwitchRightFirstPosition = 0;				//First movement position point	for:start Center side, go for switch on Right side
			double startCenterSwitchRightFirstAngle = 0;				//First movement angle	for:start Center side, go for switch on Right side
			double startCenterSwitchRightSecondPosition = 0;			//Second movement position point	for:start Center side, go for switch on Right side
			double startCenterSwitchRightSecondAngle = 0;				//Second movement angle	for:start Center side, go for switch on Right side
			double startCenterSwitchRightIntakeSpdRef = 0.0;			//Intake speed ref to spit out cube
			double startCenterSwitchRightThirdPosition = 0;				//Third movement position point	for:start Center side, go for switch on Right side
			double startCenterSwitchRightThirdAngle = 0;				//Third movement angle	for:start Center side, go for switch on Right side	
			double startCenterSwitchRightForthPosition = 0;				//Forth movement position point	for:start Center side, go for switch on Right side
			double startCenterSwitchRightForthAngle = 0;				//Forth movement angle	for:start Center side, go for switch on Right side	
		
			
				/**********************************************************************************************************************
				 * Action Code for Autonomous Periodic
				 **********************************************************************************************************************/


				/**************************************************************
				 * Snapshot system clock value on first scan of robot running
				 * Then: Current system clock - Start system clock will
				 *       give autonomous clock starting at zero
				 **************************************************************/
    			if (! autoFirstScan)
					{
						autoTimeStart = System.currentTimeMillis();				//Snapshot system time at start of autonomous
						gyroYaw = navxGyro.getYaw();							//Read gyro yaw value from navx board
						if (gyroYaw < 0)										//Left half of compass
							firstGyroScan = 180 + (180 + gyroYaw);				//gyro raw -179 to 0 = gyro compass angle +181 to 359
						else													//Right half of compass
							firstGyroScan = gyroWithDrift;						//gyro raw 0 to 179 = gyro compass angle 0 to 179
						autoFirstScan = true;									//turn on first scan flag
					}
    			currentSystemClock = System.currentTimeMillis();				//Read System clock
    			autoSequenceTime = currentSystemClock - autoTimeStart;			//Calc system time elapsed
    			//System.out.println("clock=" + autoSequenceTime);
    			//System.out.println("Our data auton= " + ourGameData);
   			
			    /****************************************************************************************************
				 **  Read raw gyro angle from the input mod. Compensate for gyro drift by adding an offset value to
				 **  Compensate for gyro drift by adding an offset value to summation gyroDriftTotal
				 **  Scale +/- gyro values and compensate for rollover so scalled to 0-360 degrees and 
				 ****************************************************************************************************/
				gyroPitch = navxGyro.getPitch();								//Get gyro pitch because roboRIO is mounted on side
				gyroYaw = navxGyro.getYaw();									//Read gyro yaw value from navx board
				gyroDriftTotal = gyroDriftTotal + gyroDriftIncrement;			//Keep track of total gyro angle drift
				gyroWithDrift = gyroYaw + gyroDriftTotal;						//Real angle is read gyro angle plus drift compensation 

				
				if (gyroWithDrift < 0)
					gyroAngleMod = 180 + (180 + gyroWithDrift);					//gyro raw -179 to 0 = gyro compass angle +181 to 359
				else
					gyroAngleMod = gyroWithDrift;								//gyro raw 0 to 179 = gyro compass angle 0 to 179
				gyroAngleOffset = gyroAngleMod - firstGyroScan;					//Normalize gyro first scan to be zero
				
				
				
				if (gyroAngleOffset < 0)
					gyroAngleFB = gyroAngleOffset + 360;
				else
					gyroAngleFB = gyroAngleOffset;
				
				/****************************************************************************************************
				 **  Read Right Drive motor encoder 
				 ****************************************************************************************************/
				rightMotorPosition = rightMotor.getSelectedSensorPosition(0);	//Get Right drive motor Position from Talon SRX
				winchMotorPosition = winchMotor.getSelectedSensorPosition(0);	//Get Winch drive motor Position from Talon SRX
				
				/*****************************************
				 * Shift winch to high speed elevator mode
				 *****************************************/
				winchMotorLock.set(false);										//Shift the winch motor lock OFF
	    		winchMotorRun.set(true);										//Shift the winch motor Run ON

    
				if (matchStartPosition == 0)		//Robot start position is left (left bumper is aligned with left point of straight part of back wall
					{

    					/**************************************************************
    					 * Robot Start=Left Side,  Go for Left Switch
    					 *   system clock is in milliseconds
    					 **************************************************************/
//    					if (((ourGameData == "LL") || (ourGameData == "LR")) && (goForScale != 1))	//Go for Left switch starting from Left side
						if (((ourGameData.equals("LL")) || (ourGameData.equals("LR"))) && (!goForScale))	//Go for Left switch starting from Left side
//					if (testStartLeftSwitchLeft)
    					{
								
								startLeftSwitchLeft[0]  = 0;						//Step 0: Start of sequence
								startLeftSwitchLeft[1]  = 50;						//Step 1: Close intake arms solenoid
								startLeftSwitchLeft[2]  = 2850;						//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								startLeftSwitchLeft[3]  = 2550;						//Step 3: Start Decel to min speed, Start rotate CW to 90 Deg during decel
								startLeftSwitchLeft[4]  = 400;						//Step 4: Use gyro correction to move to correct angle 
								startLeftSwitchLeft[5]  = 100;						//Step 5: Reset Drive encoders to zero
								startLeftSwitchLeft[6]  = 2800;						//Step 6: Start moving forward toward switch  1050
								startLeftSwitchLeft[7]  = 600;						//Step 7: Stop forward movement and Open Arms 400
								startLeftSwitchLeft[8]  = 1200;						//Step 8: Back up robot away from switch 1200
								startLeftSwitchLeft[9]  = 500;						//Step 9: Move elevator down, close intake arms 500
								startLeftSwitchLeft[10] = 0;						//Autonomous movement
								startLeftSwitchLeft[11] = 0;						//Autonomous movement
								startLeftSwitchLeft[12] = 0;						//Autonomous movement
								startLeftSwitchLeft[13] = 0;						//Autonomous movement
								startLeftSwitchLeft[14] = 14900;					//Autonomous movement is done
								startLeftSwitchLeftFirstPosition = 34500;			//Encoder counts - first move forward
								startLeftSwitchLeftFirstAngle = 84;					//Gyro angle for first turn toward switch
								startLeftSwitchLeftSecondPosition = 5950;			//Encoder counts - second move forward
								startLeftSwitchLeftIntakeSpdRef = -0.5;				//Intake motor speed ref to spit cube out
								startLeftSwitchLeftThirdPosition = -3400;			//Encoder counts - third move backward
								startLeftSwitchLeftSecondAngle = 120;				//Gyro angle for second turn toward switch
								/****************************************************
								 * Step 0 of start on Left, Switch on Left INIT
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 0)			//time = 0
									{
										System.out.println("Step 0 : " + autoSequenceTime);
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										startLeftSwitchLeftSeq = 1;									//Begin step 1
										startLeftSwitchLeftTime[1] = autoSequenceTime;				//Start time of step 1 when this step exits
									}	//End of Step 0	
									
									
								
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 9: Move elevator down, close intake arms
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 9)		//Step 9: Move elevator down, close intake arms
									{
										System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[9]));
										leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										intakeArmOpen.set(false);								//Intake arms OPEN solenoid
										intakeArmClose.set(true);								//Intake arms CLOSE solenoid
												
										winchMotor.set(ControlMode.MotionMagic, winchBottomRef);//Start magic motion for absolute position to move to SWITCH

										startLeftSwitchLeftTime[10] = autoSequenceTime;			//Start time of step 10 when this step exits			
										if (((autoSequenceTime - startLeftSwitchLeftTime[9]) > startLeftSwitchLeft[9]) )
											{
												startLeftSwitchLeftSeq = 10;					//Move to step 10
											}
											
									}	//End of Step 9: Move elevator down, close intake arms
										
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * 	Step 8: Back up robot away from switch
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 8)		//Step 8: Back up robot away from switch
									{
										System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[8]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = -0.30;											//Assign open loop speed reference 
										gyroAngleSP = startLeftSwitchLeftFirstAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));	//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Left side motor speed reference
												
										leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
										rightIntake.set(0.0);										//Assign speed ref to Left PWM Talon

										startLeftSwitchLeftTime[9] = autoSequenceTime;				//Start time of step 9 when this step exits			
										if (((autoSequenceTime - startLeftSwitchLeftTime[8]) > startLeftSwitchLeft[8])|| (rightMotorPosition < startLeftSwitchLeftThirdPosition) )
											{
												leftMotorSpdRefAut=0;								//Stop motion
												rightMotorSpdRefAut=0;								//Stop motion
												startLeftSwitchLeftSeq = 9;							//Move to step 9
											}
									}	//End of Step 8: Back up robot away from switch
										

								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 7: Stop forward movement and Open Arms
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 7)		//Step 7: Stop forward movement and Open Arms
									{
										System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[7]) + " , Gyro: " + gyroAngleFB);
										leftMotorSpdRefAut=0;										//Stop motion
										rightMotorSpdRefAut=0;										//Stop motion
											
										leftIntake.set(startLeftSwitchLeftIntakeSpdRef);			//Assign speed ref to Left PWM Talon
										rightIntake.set(-1.0 * startLeftSwitchLeftIntakeSpdRef);	//Assign speed ref to Left PWM Talon
											
										intakeArmOpen.set(true);									//Intake arms OPEN solenoid
										intakeArmClose.set(false);									//Intake arms CLOSE solenoid	

										startLeftSwitchLeftTime[8] = autoSequenceTime;				//Start time of step 8 when this step exits			
										if (((autoSequenceTime - startLeftSwitchLeftTime[7]) > startLeftSwitchLeft[7]) )
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												startLeftSwitchLeftSeq = 8;							//Move to step 8
											}
											
									}	//End of Step 7: Stop forward movement and Open Arms
										
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 6: Start moving forward toward switch
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 6)	//Step 5: Start moving forward toward switch
									{
										System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftSwitchLeftTime[6]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = 0.35;												//Assign open loop speed reference 
										gyroAngleSP = startLeftSwitchLeftFirstAngle;					//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.0));					//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

										startLeftSwitchLeftTime[7] = autoSequenceTime;					//Start time of step 7 when this step exits	
										if (((autoSequenceTime - startLeftSwitchLeftTime[6]) > startLeftSwitchLeft[6]) 
										|| (rightMotorPosition > startLeftSwitchLeftSecondPosition) )
											{
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startLeftSwitchLeftSeq = 7;								//Move to step 7
											}
									}	//End of Step 6: Start moving forward toward switch	  
										
									
									
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 5: Reset Drive encoders to zero
								 ****************************************************/
								if ((startLeftSwitchLeftSeq == 5))			//Step 5: Reset Drive encoders to zero
									{
										System.out.println("Step 5 : " + (autoSequenceTime - startLeftSwitchLeftTime[5]));	
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												
										startLeftSwitchLeftTime[6] = autoSequenceTime;
										if ((autoSequenceTime - startLeftSwitchLeftTime[5]) > startLeftSwitchLeft[5])
												{
													startLeftSwitchLeftSeq = 6;						//Move to step 6
												}
									}	//End of Step5 Reset Drive encoders to zero								
									
						
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 4: Use gyro correction to move to correct angle 
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
									{
										System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startLeftSwitchLeftTime[4]));

										yAxisRef = 0.0;													//Assign open loop speed reference 
										gyroAngleSP = startLeftSwitchLeftFirstAngle;					//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

										startLeftSwitchLeftTime[5] = autoSequenceTime;					//Start time of step 5 when this step exits
										if ((autoSequenceTime - startLeftSwitchLeftTime[4]) > startLeftSwitchLeft[4] || ((gyroAngleFB > startLeftSwitchLeftFirstAngle) && (gyroAngleFB < 100)))
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startLeftSwitchLeftSeq = 5;								//Move to step 5
											}
									}
									
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 3: Start Decel to min speed, 
								 * Start rotate CW to 270 Deg during decel
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
									{
										System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftSwitchLeftTime[3]));
										yAxisRef = 0.00;													//Assign open loop speed reference 
										gyroAngleSP = startLeftSwitchLeftFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.17));					//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.17 );						//Left side motor speed reference	

										startLeftSwitchLeftTime[4] = autoSequenceTime;						//Start time of step 4 when this step exits
										if (((autoSequenceTime - startLeftSwitchLeftTime[3]) > startLeftSwitchLeft[3]) || ((gyroAngleFB > (startLeftSwitchLeftFirstAngle-5)) && (gyroAngleFB < 100)))
											{
													startLeftSwitchLeftSeq = 4;								//Move to step 4
											}
									}	//End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
									
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 2: Drop intake arms, start moving forward to 
								 * position left of scale,start raising elevator
								 ****************************************************/
								if (startLeftSwitchLeftSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									{
										System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
										
										intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
										intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
									
										leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Left Drive motors
										leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Left Drive motors
							
										yAxisRef = 0.58;												//Assign open loop speed reference 
										gyroAngleSP = 3.0;												//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

										if (rightMotorPosition > 12000)
											winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);	//Start magic motion for absolute position to move to SWITCH
											
										startLeftSwitchLeftTime[3] = autoSequenceTime;					//Start time of step 3 when this step exits
										if ((autoSequenceTime - startLeftSwitchLeftTime[2]) > startLeftSwitchLeft[2] || (rightMotorPosition > startLeftSwitchLeftFirstPosition) )
											{
												startLeftSwitchLeftSeq = 3;
											}
									}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									
									
								/****************************************************
								 * START LEFT - SWITCH LEFT
								 * Step 1: Close intake arms solenoid
								 ****************************************************/
								if ((startLeftSwitchLeftSeq == 1))			//Step 1: Close intake arms solenoid
									{
										System.out.println("Step 1 : " + autoSequenceTime);	
										intakeArmOpen.set(false);									//Intake arms OPEN solenoid
										intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
												
										startLeftSwitchLeftTime[2] = autoSequenceTime;
										if ((autoSequenceTime - startLeftSwitchLeftTime[1]) > startLeftSwitchLeft[1])
												{
													startLeftSwitchLeftSeq = 2;						//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
												}
									}	//End of Step 1: Close intake arms solenoid	

    					}	//End of Go for Left switch starting from Left side
    

						
    				/**************************************************************
    				 * Robot START=LEFT Side,  Go for LEFT SCALE
    				 *   system clock is in milliseconds
    				 **************************************************************/
						if ((ourGameData.equals("RL")) || (goForScale))	//Go for Left switch starting from Left side
//						if (testStartLeftSwitchLeft)
    						{
								
								startLeftScaleLeft[0]  = 0;							//Step 0: Start of sequence
								startLeftScaleLeft[1]  = 50;						//Step 1: Close intake arms solenoid
								startLeftScaleLeft[2]  = 3500;						//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								startLeftScaleLeft[3]  = 2550;						//Step 3: Start Decel to min speed, Start rotate CW to 90 Deg during decel
								startLeftScaleLeft[4]  = 400;						//Step 4: Use gyro correction to move to correct angle 
								startLeftScaleLeft[5]  = 1000;						//Step 5: Reset Drive encoders to zero, Raise elevator
								startLeftScaleLeft[6]  = 2800;						//Step 6: Start moving forward toward switch  1050
								startLeftScaleLeft[7]  = 600;						//Step 7: Stop forward movement and Open Arms 400
								startLeftScaleLeft[8]  = 1200;						//Step 8: Back up robot away from switch 1200
								startLeftScaleLeft[9]  = 1500;						//Step 9: Move elevator down, close intake arms 500
								startLeftScaleLeft[10] = 500;							//Autonomous movement
								startLeftScaleLeft[11] = 0;							//Autonomous movement
								startLeftScaleLeft[12] = 0;							//Autonomous movement
								startLeftScaleLeft[13] = 0;							//Autonomous movement
								startLeftScaleLeft[14] = 14900;						//Autonomous movement is done
								startLeftScaleLeftFirstPosition = 48500;			//Encoder counts - first move forward
								startLeftScaleLeftFirstAngle = 50;					//Gyro angle for first turn toward switch
								startLeftScaleLeftSecondPosition = 8000;			//Encoder counts - second move forward
								startLeftScaleLeftIntakeSpdRef = -0.2;				//Intake motor speed ref to spit cube out
								startLeftScaleLeftThirdPosition = -8500;			//Encoder counts - third move backward
								startLeftScaleLeftSecondAngle = 30;					//Gyro angle for second turn toward switch
								/****************************************************
								 * Step 0 of start on Left, SCALE on Left INIT
								 ****************************************************/
								if (startLeftScaleLeftSeq == 0)			//time = 0
									{
										System.out.println("Step 0 : " + autoSequenceTime);
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										startLeftScaleLeftSeq = 1;									//Begin step 1
										startLeftScaleLeftTime[1] = autoSequenceTime;				//Start time of step 1 when this step exits
									}	//End of Step 0	
									
									
								
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 10: Move elevator down, close intake arms
								 ****************************************************/
								if (startLeftScaleLeftSeq == 10)		//Step 10: Move elevator down, close intake arms
									{
										System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[10]));
										leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										intakeArmOpen.set(false);								//Intake arms OPEN solenoid
										intakeArmClose.set(true);								//Intake arms CLOSE solenoid
												
										winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	//Start magic motion for absolute position to move to SWITCH

										startLeftScaleLeftTime[11] = autoSequenceTime;			//Start time of step 10 when this step exits			
										if (((autoSequenceTime - startLeftScaleLeftTime[10]) > startLeftScaleLeft[10]) )
											{
												startLeftScaleLeftSeq = 11;					//Move to step 11
											}
											
									}	//End of Step 10: Move elevator down, close intake arms
										
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 9: Back up robot away from switch
								 ****************************************************/
								if (startLeftScaleLeftSeq == 9)		//Step 9: Back up robot away from switch
									{
										System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[9]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = -0.30;											//Assign open loop speed reference 
										gyroAngleSP = startLeftScaleLeftSecondAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));	//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Left side motor speed reference
												
										leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
										rightIntake.set(0.0);										//Assign speed ref to Left PWM Talon

										startLeftScaleLeftTime[10] = autoSequenceTime;				//Start time of step 9 when this step exits			
										if (((autoSequenceTime - startLeftScaleLeftTime[9]) > startLeftScaleLeft[9])
										|| (rightMotorPosition < startLeftScaleLeftThirdPosition) )
											{
												leftMotorSpdRefAut=0;								//Stop motion
												rightMotorSpdRefAut=0;								//Stop motion
												startLeftScaleLeftSeq = 10;							//Move to step 10
											}
									}	//End of Step 9: Back up robot away from switch
										

								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 8: Stop forward movement and Open Arms
								 ****************************************************/
								if (startLeftScaleLeftSeq == 8)		//Step 8: Stop forward movement and Open Arms
									{
										System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[8]) + " , Gyro: " + gyroAngleFB);
										leftMotorSpdRefAut=0;										//Stop motion
										rightMotorSpdRefAut=0;										//Stop motion
											
										leftIntake.set(startLeftScaleLeftIntakeSpdRef);				//Assign speed ref to Left PWM Talon
										rightIntake.set(-1.0 * startLeftScaleLeftIntakeSpdRef);		//Assign speed ref to Left PWM Talon
											
										intakeArmOpen.set(true);									//Intake arms OPEN solenoid
										intakeArmClose.set(false);									//Intake arms CLOSE solenoid	

										startLeftScaleLeftTime[9] = autoSequenceTime;				//Start time of step 8 when this step exits			
										if (((autoSequenceTime - startLeftScaleLeftTime[8]) > startLeftScaleLeft[8]) )
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												startLeftScaleLeftSeq = 9;							//Move to step 9
											}
									}	//End of Step 8: Stop forward movement and Open Arms
									
								
								
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 7: Start rotate CCW to 0 Deg during decel
								 ****************************************************/
								if (startLeftScaleLeftSeq == 7)	//Step 7: Start Decel to min speed, Start rotate CW to 30 Deg during decel
									{
										System.out.println("Step 7 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftScaleLeftTime[3]));
										yAxisRef = 0.15;													//Assign open loop speed reference 
										gyroAngleSP = 0.0;							//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0));					//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25 );						//Left side motor speed reference	

//										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);			//Start magic motion for absolute position to move to SWITCH
										
										startLeftScaleLeftTime[8] = autoSequenceTime;						//Start time of step 4 when this step exits
										if (((autoSequenceTime - startLeftScaleLeftTime[7]) > startLeftScaleLeft[7])
										|| ((gyroAngleFB < (gyroAngleSP+2)) && (gyroAngleFB < 100)))
											{
												startLeftScaleLeftSeq = 8;									//Move to step 4
											}
									}	//End of Step 7: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
								

								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 6: Start moving forward toward switch
								 ****************************************************/
								if (startLeftScaleLeftSeq == 6)	//Step 6: Start moving forward toward switch
									{
										System.out.println("Step 6LC : " + rightMotorPosition + ", " + (autoSequenceTime - startLeftScaleLeftTime[6]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = 0.20;												//Assign open loop speed reference 
										gyroAngleSP = startLeftScaleLeftFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

										startLeftScaleLeftTime[7] = autoSequenceTime;					//Start time of step 7 when this step exits	
										if (((autoSequenceTime - startLeftScaleLeftTime[6]) > startLeftScaleLeft[6]) 
										|| (rightMotorPosition > startLeftScaleLeftSecondPosition) )
											{
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startLeftScaleLeftSeq = 7;								//Move to step 7
											}
									}	//End of Step 6: Start moving forward toward switch	  
										
									
									
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 5: Reset Drive encoders to zero, start raising elevator
								 ****************************************************/
								if ((startLeftScaleLeftSeq == 5))			//Step 5: Reset Drive encoders to zero
									{
										//System.out.println("Step 5 : " + (autoSequenceTime - startLeftScaleLeftTime[5]));	
										//System.out.println("Winch Pos: " + winchMotorPosition);
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
											
										//winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);	//Start magic motion for absolute position to move to SWITCH
										
										startLeftScaleLeftTime[6] = autoSequenceTime;
										if ((autoSequenceTime - startLeftScaleLeftTime[5]) > startLeftScaleLeft[5])
												{
													startLeftScaleLeftSeq = 6;						//Move to step 6
												}
									}	//End of Step5 Reset Drive encoders to zero								
									
						
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 4: Use gyro correction to move to correct angle 
								 ****************************************************/
								if (startLeftScaleLeftSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
									{
										System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startLeftScaleLeftTime[4]));

										yAxisRef = 0.2;													//Assign open loop speed reference 
										gyroAngleSP = startLeftScaleLeftFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

										startLeftScaleLeftTime[5] = autoSequenceTime;					//Start time of step 5 when this step exits
										if ((autoSequenceTime - startLeftScaleLeftTime[4]) > startLeftScaleLeft[4] 
										|| ((gyroAngleFB > startLeftScaleLeftFirstAngle) && (gyroAngleFB < 100)))
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startLeftScaleLeftSeq = 5;								//Move to step 5
											}
									}
									
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 3: Start Decel to min speed, 
								 * Start rotate CW to 30 Deg during decel
								 ****************************************************/
								if (startLeftScaleLeftSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CW to 30 Deg during decel
									{
										System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startLeftScaleLeftTime[3]));
										yAxisRef = 0.20;													//Assign open loop speed reference 
										gyroAngleSP = startLeftScaleLeftFirstAngle;							//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.15));					//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0 );						//Left side motor speed reference	

										System.out.println("Step 3 : " + (autoSequenceTime - startLeftScaleLeftTime[5]));	
										System.out.println("Winch Pos: " + winchMotorPosition);
										
//										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);			//Start magic motion for absolute position to move to SWITCH
										winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);	//Start magic motion for absolute position to move to SWITCH
										
										startLeftScaleLeftTime[4] = autoSequenceTime;						//Start time of step 4 when this step exits
										if (((autoSequenceTime - startLeftScaleLeftTime[3]) > startLeftScaleLeft[3])
										|| ((gyroAngleFB > (startLeftScaleLeftFirstAngle-2)) && (gyroAngleFB < 100)))
											{
												startLeftScaleLeftSeq = 4;									//Move to step 4
											}
									}	//End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
									
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 2: Drop intake arms, start moving forward to 
								 * position left of SCALE
								 ****************************************************/
								if (startLeftScaleLeftSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									{
										System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
										
										intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
										intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
									
										leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Left Drive motors
										leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Left Drive motors
							
										yAxisRef = 0.85;												//Assign open loop speed reference 
										gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

										startLeftScaleLeftTime[3] = autoSequenceTime;					//Start time of step 3 when this step exits
										if ((autoSequenceTime - startLeftScaleLeftTime[2]) > startLeftScaleLeft[2] 
										|| (rightMotorPosition > startLeftScaleLeftFirstPosition) )
											{
												startLeftScaleLeftSeq = 3;
											}
									}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									
									
								/****************************************************
								 * START LEFT - SCALE LEFT
								 * Step 1: Close intake arms solenoid
								 ****************************************************/
								if ((startLeftScaleLeftSeq == 1))			//Step 1: Close intake arms solenoid
									{
										System.out.println("Step 1 : " + autoSequenceTime);	
										intakeArmOpen.set(false);									//Intake arms OPEN solenoid
										intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
										
										
										if (gyroYaw < 0)										//Left half of compass
											firstGyroScan = 180 + (180 + gyroYaw);				//gyro raw -179 to 0 = gyro compass angle +181 to 359
										else													//Right half of compass
											firstGyroScan = gyroWithDrift;						//gyro raw 0 to 179 = gyro compass angle 0 to 179
										
												
										startLeftScaleLeftTime[2] = autoSequenceTime;
										if ((autoSequenceTime - startLeftScaleLeftTime[1]) > startLeftScaleLeft[1])
												{
													startLeftScaleLeftSeq = 2;						//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
												}
									}	//End of Step 1: Close intake arms solenoid	

    					}	//End of START LEFT, go for SCALE LEFT
    
						
						

    				
					}	//End of robot starts on left
    			
				if (matchStartPosition == 1)		//Robot start position is center (left bumper is aligned with right wall of exchange zone box of back wall
					{
						/**************************************************************
						 * Robot START=CENTER Side,  Go for LEFT SWITCH
						 *   system clock is in milliseconds
						 **************************************************************/
						if ((ourGameData.equals("LL")) || (ourGameData.equals("LR")) || (goForScale))	//Go for Left switch starting from Left side
//						if (testStartLeftSwitchLeft)
							{
								
								startCenterSwitchLeft[0]  = 0;							//Step 0: Start of sequence
								startCenterSwitchLeft[1]  = 50;							//Step 1: Close intake arms solenoid
								startCenterSwitchLeft[2]  = 2200;						//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								startCenterSwitchLeft[3]  = 2200;						//Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
								startCenterSwitchLeft[4]  = 400;						//Step 4: Use gyro correction to move to correct angle 
								startCenterSwitchLeft[5]  = 6000;						//Step 5 Rotate CW to 0 degrees toward Switch
								startCenterSwitchLeft[6]  = 2800;						//Step 6: Start moving forward toward switch  1050
								startCenterSwitchLeft[7]  = 600;						//Step 7: Stop forward movement and Open Arms 400
								startCenterSwitchLeft[8]  = 1200;						//Step 8: Back up robot away from switch 1200
								startCenterSwitchLeft[9]  = 500;						//Step 9: Move elevator down, close intake arms 500
								startCenterSwitchLeft[10] = 0;							//Autonomous movement
								startCenterSwitchLeft[11] = 0;							//Autonomous movement
								startCenterSwitchLeft[12] = 0;							//Autonomous movement
								startCenterSwitchLeft[13] = 0;							//Autonomous movement
								startCenterSwitchLeft[14] = 14900;						//Autonomous movement is done
								startCenterSwitchLeftFirstPosition = 6000;				//Encoder counts - first move forward
								startCenterSwitchLeftFirstAngle = 305;					//Gyro angle for first turn toward switch
								startCenterSwitchLeftSecondPosition = 15500;			//Encoder counts - second move forward
								startCenterSwitchLeftIntakeSpdRef = -0.4;				//Intake motor speed ref to spit cube out
								startCenterSwitchLeftThirdPosition = 31550;				//Encoder counts - third move forward
								startCenterSwitchLeftSecondAngle = 0;					//Gyro angle for second turn toward switch
								startCenterSwitchLeftForthPosition = -4000;			//Encoder counts - forth move backwards
								startCenterSwitchLeftForthAngle = 70;					//Gyro angle for forth turn away switch
								/****************************************************
								 * Step 0 of start on CENTER, SWITCH on Left INIT
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 0)			//time = 0
									{
										System.out.println("Step 0 : " + autoSequenceTime);
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										startCenterSwitchLeftSeq = 1;								//Begin step 1
										startCenterSwitchLeftTime[1] = autoSequenceTime;			//Start time of step 1 when this step exits
									}	//End of Step 0	
									
									
								
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 9: Move elevator down, close intake arms
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 9)		//Step 9: Move elevator down, close intake arms
									{
										System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[9]));
										leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										intakeArmOpen.set(false);								//Intake arms OPEN solenoid
										intakeArmClose.set(true);								//Intake arms CLOSE solenoid
												
										winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	//Start magic motion for absolute position to move to SWITCH

										startCenterSwitchLeftTime[10] = autoSequenceTime;		//Start time of step 10 when this step exits			
										if (((autoSequenceTime - startCenterSwitchLeftTime[9]) > startCenterSwitchLeft[9]) )
											{
												startCenterSwitchLeftSeq = 10;					//Move to step 10
											}
											
									}	//End of Step 9: Move elevator down, close intake arms
										
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 8: Back up robot away from switch
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 8)		//Step 8: Back up robot away from switch
									{
										System.out.println("Step 8CL : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[8]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = -0.30;											//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchLeftFirstAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));	//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Left side motor speed reference
												
										leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
										rightIntake.set(0.0);										//Assign speed ref to Left PWM Talon

										startCenterSwitchLeftTime[9] = autoSequenceTime;				//Start time of step 9 when this step exits	

										if ( ((autoSequenceTime - startCenterSwitchLeftTime[8]) > startCenterSwitchLeft[8])	//end on time 
											|| (rightMotorPosition < startCenterSwitchLeftForthPosition) )				//end on position
											{
												leftMotorSpdRefAut=0;								//Stop motion
												rightMotorSpdRefAut=0;								//Stop motion
												startCenterSwitchLeftSeq = 9;						//Move to step 9
											}
									}	//End of Step 8: Back up robot away from switch
										

								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 7: Stop forward movement and Open Arms
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 7)		//Step 7: Stop forward movement and Open Arms
									{
										System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[7]) + " , Gyro: " + gyroAngleFB);
										leftMotorSpdRefAut=0;										//Stop motion
										rightMotorSpdRefAut=0;										//Stop motion
											
										leftIntake.set(startCenterSwitchLeftIntakeSpdRef);			//Assign speed ref to Left PWM Talon
										rightIntake.set(-1.0 * startCenterSwitchLeftIntakeSpdRef);	//Assign speed ref to Left PWM Talon
											
										intakeArmOpen.set(true);									//Intake arms OPEN solenoid
										intakeArmClose.set(false);									//Intake arms CLOSE solenoid	

										startCenterSwitchLeftTime[8] = autoSequenceTime;			//Start time of step 8 when this step exits			
										if (((autoSequenceTime - startCenterSwitchLeftTime[7]) > startCenterSwitchLeft[7]) )
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												startCenterSwitchLeftSeq = 8;						//Move to step 8
											}
											
									}	//End of Step 7: Stop forward movement and Open Arms
										
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 6: Start moving forward toward switch
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 6)	//Step 5: Start moving forward toward switch
									{
										System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchLeftTime[6]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = 0.25;												//Assign open loop speed reference 
										gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

										startCenterSwitchLeftTime[7] = autoSequenceTime;				//Start time of step 7 when this step exits	
										if (((autoSequenceTime - startCenterSwitchLeftTime[6]) > startCenterSwitchLeft[6]) 
										|| (rightMotorPosition > startCenterSwitchLeftThirdPosition) )
											{
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startCenterSwitchLeftSeq = 7;							//Move to step 7
											}
									}	//End of Step 6: Start moving forward toward switch	  
										
									
									
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 5: Rotate CW to 0 degrees toward Switch
								 ****************************************************/
								if ((startCenterSwitchLeftSeq == 5))			//Step 5: Rotate CW to 0 degrees toward Switch
									{
										System.out.println("Step 5 : " + (autoSequenceTime - startCenterSwitchLeftTime[5]));	
											
										yAxisRef = 0.17;												//Assign open loop speed reference 
										gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.20);					//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef );						//Left side motor speed reference	
										
										startCenterSwitchLeftTime[6] = autoSequenceTime;
										if ((autoSequenceTime - startCenterSwitchLeftTime[5]) > startCenterSwitchLeft[5] 
										|| ((gyroAngleFB < (358.0)) 
										|| (gyroAngleFB > 2.0)))
												{
													startCenterSwitchLeftSeq = 6;						//Move to step 6
												}
									}	//End of Step 5 Rotate CW to 0 degrees toward Switch							
									
						
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 4: Use gyro correction to move to correct angle
								 *   closer to the Switch
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
									{
										System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startCenterSwitchLeftTime[4]));

										yAxisRef = 0.20;												//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchLeftFirstAngle;					//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

										startCenterSwitchLeftTime[5] = autoSequenceTime;				//Start time of step 5 when this step exits
										if ((autoSequenceTime - startCenterSwitchLeftTime[4]) > startCenterSwitchLeft[4] 
										|| ((rightMotorPosition > startCenterSwitchLeftSecondPosition) ))
											{
												startCenterSwitchLeftSeq = 5;							//Move to step 5
											}
									}
									
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 3: Start Decel to min speed, 
								 * Start rotate CCW to 315 Deg during decel
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
									{
										System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startCenterSwitchLeftTime[3]));
										yAxisRef = 0.20;													//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchLeftFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * (yAxisRef);								//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.17 );						//Left side motor speed reference	
										
										startCenterSwitchLeftTime[4] = autoSequenceTime;					//Start time of step 4 when this step exits
										if (((autoSequenceTime - startCenterSwitchLeftTime[3]) > startCenterSwitchLeft[3]) 
										|| ((gyroAngleFB < (startCenterSwitchLeftFirstAngle+2)) && (gyroAngleFB > 200)))
											{
												startCenterSwitchLeftSeq = 4;								//Move to step 4
											}
									}	//End of Step 3: Start Decel to min speed, Start rotate CW to 315 Deg during decel	
									
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 2: Drop intake arms, start moving forward to 
								 * LEFT SCALE position, Raise elevator
								 ****************************************************/
								if (startCenterSwitchLeftSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									{
										System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
										
										intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
										intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
									
										leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Left Drive motors
										leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Left Drive motors
							
										yAxisRef = 0.20;												//Assign open loop speed reference 
										gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference
										
										if (rightMotorPosition > 2400)
											winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);	//Start magic motion for absolute position to move to SWITCH

										startCenterSwitchLeftTime[3] = autoSequenceTime;				//Start time of step 3 when this step exits
										if ((autoSequenceTime - startCenterSwitchLeftTime[2]) > startCenterSwitchLeft[2] || (rightMotorPosition > startCenterSwitchLeftFirstPosition) )
											{
												startCenterSwitchLeftSeq = 3;
											}
									}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									
									
								/****************************************************
								 * START CENTER - SWITCH LEFT
								 * Step 1: Close intake arms solenoid
								 ****************************************************/
								if ((startCenterSwitchLeftSeq == 1))			//Step 1: Close intake arms solenoid
									{
										System.out.println("Step 1 : " + autoSequenceTime);	
										intakeArmOpen.set(false);									//Intake arms OPEN solenoid
										intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
												
										startCenterSwitchLeftTime[2] = autoSequenceTime;
										if ((autoSequenceTime - startCenterSwitchLeftTime[1]) > startCenterSwitchLeft[1])
												{
													startCenterSwitchLeftSeq = 2;					//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
												}
									}	//End of Step 1: Close intake arms solenoid	

    					}	//End of START CENTER, go for SWITCH LEFT
    
				
	
						/**************************************************************
						 * Robot START=CENTER Side,  Go for RIGHT SWITCH
						 *   system clock is in milliseconds
						 **************************************************************/
						if ((ourGameData.equals("RL")) || (ourGameData.equals("RR")) || (goForScale))	//Go for Left switch starting from Left side
//						if (testStartLeftSwitchLeft)
							{
								
								startCenterSwitchRight[0]  = 0;							//Step 0: Start of sequence
								startCenterSwitchRight[1]  = 50;						//Step 1: Close intake arms solenoid
								startCenterSwitchRight[2]  = 2200;						//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								startCenterSwitchRight[3]  = 2200;						//Step 3: Start Decel to min speed, Start rotate CCW to 315 Deg during decel
								startCenterSwitchRight[4]  = 400;						//Step 4: Use gyro correction to move to correct angle 
								startCenterSwitchRight[5]  = 3000;						//Step 5 Rotate CW to 0 degrees toward Switch
								startCenterSwitchRight[6]  = 1800;						//Step 6: Start moving forward toward switch  1050
								startCenterSwitchRight[7]  = 600;						//Step 7: Stop forward movement and Open Arms 400
								startCenterSwitchRight[8]  = 1200;						//Step 8: Back up robot away from switch 1200
								startCenterSwitchRight[9]  = 500;						//Step 9: Move elevator down, close intake arms 500
								startCenterSwitchRight[10] = 0;							//Autonomous movement
								startCenterSwitchRight[11] = 0;							//Autonomous movement
								startCenterSwitchRight[12] = 0;							//Autonomous movement
								startCenterSwitchRight[13] = 0;							//Autonomous movement
								startCenterSwitchRight[14] = 14900;						//Autonomous movement is done
								startCenterSwitchRightFirstPosition = 6000;				//Encoder counts - first move forward
								startCenterSwitchRightFirstAngle = 55;					//Gyro angle for first turn toward switch
								startCenterSwitchRightSecondPosition = 15500;			//Encoder counts - second move forward
								startCenterSwitchRightSecondAngle = 4.0;				//Gyro angle for second turn toward switch
								startCenterSwitchRightIntakeSpdRef = -0.2;				//Intake motor speed ref to spit cube out
								startCenterSwitchRightThirdPosition = 32000;			//Encoder counts - third move forward
								startCenterSwitchRightThirdAngle = 0;					//Gyro angle for second turn toward switch
								startCenterSwitchRightForthPosition = -4000;			//Encoder counts - forth move backwards
								startCenterSwitchRightForthAngle = 300;					//Gyro angle for forth turn away switch
								startCenterSwitchRightIntakeSpdRef = -0.3;
								/****************************************************
								 * Step 0 of start on CENTER, SWITCH on Right INIT
								 ****************************************************/
								if (startCenterSwitchRightSeq == 0)			//time = 0
									{
										System.out.println("Step 0 : " + autoSequenceTime);
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										startCenterSwitchRightSeq = 1;								//Begin step 1
										startCenterSwitchRightTime[1] = autoSequenceTime;			//Start time of step 1 when this step exits
									}	//End of Step 0	
									
									
								
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 9: Move elevator down, close intake arms
								 ****************************************************/
								if (startCenterSwitchRightSeq == 9)		//Step 9: Move elevator down, close intake arms
									{
										System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[9]));
										leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										intakeArmOpen.set(false);								//Intake arms OPEN solenoid
										intakeArmClose.set(true);								//Intake arms CLOSE solenoid
												
										winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	//Start magic motion for absolute position to move to SWITCH

										startCenterSwitchRightTime[10] = autoSequenceTime;			//Start time of step 10 when this step exits			
										if (((autoSequenceTime - startCenterSwitchRightTime[9]) > startCenterSwitchRight[9]) )
											{
												startCenterSwitchRightSeq = 10;						//Move to step 10
											}
											
									}	//End of Step 9: Move elevator down, close intake arms
										
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 8: Back up robot away from switch
								 ****************************************************/
								if (startCenterSwitchRightSeq == 8)		//Step 8: Back up robot away from switch
									{
										System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[8]) + " , Gyro: " + gyroAngleFB + " Pos: " + rightMotorPosition);
										yAxisRef = -0.25;											//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchRightSecondAngle;			//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef - 0.15));			//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Left side motor speed reference
												
										leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
										rightIntake.set(0.0);										//Assign speed ref to Left PWM Talon

										startCenterSwitchRightTime[9] = autoSequenceTime;			//Start time of step 9 when this step exits			
										if ( ((autoSequenceTime - startCenterSwitchRightTime[8]) > startCenterSwitchRight[8])	//end on time 
											|| (rightMotorPosition < startCenterSwitchRightForthPosition) )		//end on position										//end on angle
											{
												leftMotorSpdRefAut=0;								//Stop motion
												rightMotorSpdRefAut=0;								//Stop motion
												startCenterSwitchRightSeq = 9;						//Move to step 9
											}
									}	//End of Step 8: Back up robot away from switch
										

								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 7: Stop forward movement and Open Arms
								 ****************************************************/
								if (startCenterSwitchRightSeq == 7)		//Step 7: Stop forward movement and Open Arms
									{
										System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[7]) + " , Gyro: " + gyroAngleFB);
										leftMotorSpdRefAut=0;										//Stop motion
										rightMotorSpdRefAut=0;										//Stop motion
											
										leftIntake.set(startCenterSwitchRightIntakeSpdRef);			//Assign speed ref to Left PWM Talon
										rightIntake.set(-1.0 * startCenterSwitchRightIntakeSpdRef);	//Assign speed ref to Left PWM Talon
											
										intakeArmOpen.set(true);									//Intake arms OPEN solenoid
										intakeArmClose.set(false);									//Intake arms CLOSE solenoid	

										startCenterSwitchRightTime[8] = autoSequenceTime;			//Start time of step 8 when this step exits			
										if (((autoSequenceTime - startCenterSwitchRightTime[7]) > startCenterSwitchRight[7]) )
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												startCenterSwitchRightSeq = 8;								//Move to step 8
											}
											
									}	//End of Step 7: Stop forward movement and Open Arms
										
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 6: Start moving forward toward switch
								 ****************************************************/
								if (startCenterSwitchRightSeq == 6)	//Step 5: Start moving forward toward switch
									{
										System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startCenterSwitchRightTime[6]) + " , Gyro: " + gyroAngleFB  + "Pos: " + rightMotorPosition);
										yAxisRef = 0.25;												//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchRightSecondAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

										startCenterSwitchRightTime[7] = autoSequenceTime;				//Start time of step 7 when this step exits	
										if (((autoSequenceTime - startCenterSwitchRightTime[6]) > startCenterSwitchRight[6]) 
										|| (rightMotorPosition > startCenterSwitchRightThirdPosition) )
											{
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
												startCenterSwitchRightSeq = 7;							//Move to step 7
											}
									}	//End of Step 6: Start moving forward toward switch	  
										
									
									
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 5: Rotate CW to 0 degrees toward Switch
								 ****************************************************/
								if ((startCenterSwitchRightSeq == 5))			//Step 5: Rotate CCW to 0 degrees toward Switch
									{
										System.out.println("Step 5 : " + (autoSequenceTime - startCenterSwitchRightTime[5])  + "Pos: " + rightMotorPosition + " gyro: " + gyroAngleFB);	
											
										yAxisRef = 0.17;												//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchRightSecondAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * (yAxisRef);							//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.20);					//Left side motor speed reference	
										
										startCenterSwitchRightTime[6] = autoSequenceTime;
										if ((autoSequenceTime - startCenterSwitchRightTime[5]) > startCenterSwitchRight[5] 
										|| (gyroAngleFB < (gyroAngleSP + 5))
										|| (gyroAngleFB > 358) )
												{
													leftMotorSpdRefAut=0;								//Stop motion
													rightMotorSpdRefAut=0;								//Stop motion
													startCenterSwitchRightSeq = 6;						//Move to step 6
												}
									}	//End of Step 5 Rotate CCW to 0 degrees toward Switch							
									
						
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 4: Use gyro correction to move to correct angle
								 *   closer to the Switch
								 ****************************************************/
								if (startCenterSwitchRightSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
									{
										System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startCenterSwitchRightTime[4]) + "Pos: " + rightMotorPosition);

										yAxisRef = 0.20;													//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchRightFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));			//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);						//Left side motor speed reference

										startCenterSwitchRightTime[5] = autoSequenceTime;					//Start time of step 5 when this step exits
										if ((autoSequenceTime - startCenterSwitchRightTime[4]) > startCenterSwitchRight[4] 
										|| ((rightMotorPosition > startCenterSwitchRightSecondPosition) ))
											{
												startCenterSwitchRightSeq = 5;								//Move to step 5
											}
									}
									
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 3: Start rotate CW to 45 Deg while continuing to move forward
								 ****************************************************/
								if (startCenterSwitchRightSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CW to 45 Deg during decel
									{
										System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startCenterSwitchRightTime[3])  + "Pos: " + rightMotorPosition );
										yAxisRef = 0.20;													//Assign open loop speed reference 
										gyroAngleSP = startCenterSwitchRightFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * (yAxisRef + 0.17);						//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);								//Left side motor speed reference	
										
										startCenterSwitchRightTime[4] = autoSequenceTime;					//Start time of step 4 when this step exits
										if (((autoSequenceTime - startCenterSwitchRightTime[3]) > startCenterSwitchRight[3]) 
										|| ((gyroAngleFB > (startCenterSwitchRightFirstAngle-2)) && (gyroAngleFB < 100)))
											{
												startCenterSwitchRightSeq = 4;								//Move to step 4
											}
									}	//End of Step 3: Start Decel to min speed, Start rotate CW to 45 Deg during decel	
									
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 2: Drop intake arms, start moving forward to 
								 * LEFT SCALE position, Raise elevator
								 ****************************************************/
								if (startCenterSwitchRightSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									{
										System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB  + "Pos: " + rightMotorPosition);
										
										intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
										intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
									
										leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Left Drive motors
										leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Left Drive motors
							
										yAxisRef = 0.20;												//Assign open loop speed reference 
										gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference
										
										if (rightMotorPosition > 2400)
											winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);	//Start magic motion for absolute position to move to SWITCH

										startCenterSwitchRightTime[3] = autoSequenceTime;				//Start time of step 3 when this step exits
										if ((autoSequenceTime - startCenterSwitchRightTime[2]) > startCenterSwitchRight[2] || (rightMotorPosition > startCenterSwitchRightFirstPosition) )
											{
												startCenterSwitchRightSeq = 3;
											}
									}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									
									
								/****************************************************
								 * START CENTER - SWITCH RIGHT
								 * Step 1: Close intake arms solenoid
								 ****************************************************/
								if ((startCenterSwitchRightSeq == 1))			//Step 1: Close intake arms solenoid
									{
										System.out.println("Step 1 : " + autoSequenceTime);	
										intakeArmOpen.set(false);									//Intake arms OPEN solenoid
										intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
												
										startCenterSwitchRightTime[2] = autoSequenceTime;
										if ((autoSequenceTime - startCenterSwitchRightTime[1]) > startCenterSwitchRight[1])
												{
													startCenterSwitchRightSeq = 2;					//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
												}
									}	//End of Step 1: Close intake arms solenoid	

    					}	//End of START CENTER, go for SWITCH LEFT
						
						
						
				
      		}	//End of robot starts in center
      			
      			
      	if (matchStartPosition == 2)		//Robot start position is right (right bumper is aligned with right point of straight part of back wall
      		{
      		
						/**************************************************************
						 * Robot Start=Right Side,  Go for Right Switch
						 *  	system clock is in milliseconds
						 **************************************************************/
						if (((ourGameData.equals("RL")) || (ourGameData.equals("RR"))))	//Go for Left switch starting from Left side
	//						if (testStartRightSwitchRight)
							{
								startRightSwitchRight[0]  = 0;					//Step 0: Start of sequence
								startRightSwitchRight[1]  = 50;					//Step 1: Close intake arms solenoid
								startRightSwitchRight[2]  = 2850;				//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
								startRightSwitchRight[3]  = 2550;				//Step 3: Start Decel to min speed, Start rotate CCW to 270 Deg during decel
								startRightSwitchRight[4]  = 400;				//Step 4: Use gyro correction to move to correct angle 
								startRightSwitchRight[5]  = 100;				//Step 5: Reset Drive encoders to zero
								startRightSwitchRight[6]  = 2800;				//Step 6: Start moving forward toward switch  1050
								startRightSwitchRight[7]  = 600;				//Step 7: Stop forward movement and Open Arms 400
								startRightSwitchRight[8]  = 1200;				//Step 8: Back up robot away from switch 1200
								startRightSwitchRight[9]  = 500;				//Step 9: Move elevator down, close intake arms 500
								startRightSwitchRight[10] = 0;					//Autonomous movement
								startRightSwitchRight[11] = 0;					//Autonomous movement
								startRightSwitchRight[12] = 0;					//Autonomous movement
								startRightSwitchRight[13] = 0;					//Autonomous movement
								startRightSwitchRight[14] = 14900;				//Autonomous movement is done
								startRightSwitchRightFirstPosition = 35500;		//Encoder counts - first move forward
								startRightSwitchRightFirstAngle = 275;			//Gyro angle for first turn toward switch
								startRightSwitchRightSecondPosition = 3000;		//Encoder counts - second move forward
								startRightSwitchRightIntakeSpdRef = -0.5;		//Intake motor speed ref to spit cube out
								startRightSwitchRightThirdPosition = -4000;		//Encoder counts - third move backward
								startRightSwitchRightSecondAngle = 272;			//Gyro angle for second turn toward switch
								/****************************************************
								 * Step 0 of start on Right, Switch on Right INIT
								 ****************************************************/
								if (startRightSwitchRightSeq == 0)			//time = 0
									{
										System.out.println("Step 0 : " + autoSequenceTime);
										leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);				//Open loop ramp rate
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);		//Zero the position encoder
										rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs);			//Open loop ramp rate
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);		//Zero the position encoder
										startRightSwitchRightSeq = 1;									//Begin step 1
										startRightSwitchRightTime[1] = autoSequenceTime;				//Start time of step 1 when this step exits
									}	//End of Step 0	
									
									
								
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 9: Move elevator down, close intake arms
								 ****************************************************/
								if (startRightSwitchRightSeq == 9)		//Step 9: Move elevator down, close intake arms
									{
										System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[9]));
										leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
										intakeArmOpen.set(false);								//Intake arms OPEN solenoid
										intakeArmClose.set(true);								//Intake arms CLOSE solenoid
											
										winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	//Start magic motion for absolute position to move to SWITCH
		
										startRightSwitchRightTime[10] = autoSequenceTime;		//Start time of step 8 when this step exits			
										if (((autoSequenceTime - startRightSwitchRightTime[9]) > startRightSwitchRight[9]) )
											{
												startRightSwitchRightSeq = 10;					//Move to step 10
											}
										
									}	//End of Step 9: Move elevator down, close intake arms
										
									/****************************************************
									 * START RIGHT - SWITCH RIGHT
									 * 	Step 8: Back up robot away from switch
									 ****************************************************/
								if (startRightSwitchRightSeq == 8)		//Step 8: Back up robot away from switch
									{
										System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[8]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = -0.25;											//Assign open loop speed reference 
										gyroAngleSP = startRightSwitchRightFirstAngle;				//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));	//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Right side motor speed reference
												
										leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
								   	rightIntake.set(0.0);											//Assign speed ref to Right PWM Talon
		
										startRightSwitchRightTime[9] = autoSequenceTime;			//Start time of step 9 when this step exits			
										if (((autoSequenceTime - startRightSwitchRightTime[8]) > startRightSwitchRight[8])
										|| (rightMotorPosition < startRightSwitchRightThirdPosition) )
											{
												leftMotorSpdRefAut=0;								//Stop motion
												rightMotorSpdRefAut=0;								//Stop motion
												startRightSwitchRightSeq = 9;						//Move to step 9
											}
									}	//End of Step 8: Back up robot away from switch
										
		
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 7: Stop forward movement and Open Arms
								 ****************************************************/
								if (startRightSwitchRightSeq == 7)		//Step 7: Stop forward movement and Open Arms
									{
										System.out.println("Step 7 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[7]) + " , Gyro: " + gyroAngleFB);
										leftMotorSpdRefAut=0;										//Stop motion
										rightMotorSpdRefAut=0;										//Stop motion
											
										leftIntake.set(startRightSwitchRightIntakeSpdRef);			//Assign speed ref to Left PWM Talon
								   	rightIntake.set(-1.0 * startRightSwitchRightIntakeSpdRef);		//Assign speed ref to Right PWM Talon
											
								   	intakeArmOpen.set(true);										//Intake arms OPEN solenoid
										intakeArmClose.set(false);									//Intake arms CLOSE solenoid	
		
										startRightSwitchRightTime[8] = autoSequenceTime;			//Start time of step 6 when this step exits			
										if (((autoSequenceTime - startRightSwitchRightTime[7]) > startRightSwitchRight[7]) )
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												startRightSwitchRightSeq = 8;								//Move to step 8
											}
									}	//End of Step 7: Stop forward movement and Open Arms
										
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 6: Start moving forward toward switch
								 ****************************************************/
								if (startRightSwitchRightSeq == 6)	//Step 6: Start moving forward toward switch
									{
										System.out.println("Step 6 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightSwitchRightTime[6]) + " , Gyro: " + gyroAngleFB);
										yAxisRef = 0.25;																	//Assign open loop speed reference 
										gyroAngleSP = startRightSwitchRightFirstAngle;					//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Right side motor speed reference
		
										startRightSwitchRightTime[7] = autoSequenceTime;				//Start time of step 7 when this step exits	
										if (((autoSequenceTime - startRightSwitchRightTime[6]) > startRightSwitchRight[6]) 
										|| (rightMotorPosition > startRightSwitchRightSecondPosition) )
											{
												leftMotorSpdRefAut=0;									//Stop motion
												rightMotorSpdRefAut=0;									//Stop motion
							  				startRightSwitchRightSeq = 7;								//Move to step 7
											}
									}	//End of Step 5: Start moving forward toward switch	  
									
									
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 5: Reset Drive encoders to zero
								 ****************************************************/
								if ((startRightSwitchRightSeq == 5))			//Step 5: Reset Drive encoders to zero
									{
										System.out.println("Step 5 : " + (autoSequenceTime - startRightSwitchRightTime[5]));	
										leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
										rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												
										startRightSwitchRightTime[6] = autoSequenceTime;
										if ((autoSequenceTime - startRightSwitchRightTime[5]) > startRightSwitchRight[5])
												{
													startRightSwitchRightSeq = 6;					//Move to step 6
												}
									}	//End of Step 5 Reset Drive encoders to zero								
																
		
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 4: Use gyro correction to move to correct angle 
								 ****************************************************/
								if (startRightSwitchRightSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
									{
										System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startRightSwitchRightTime[4]));
		
										yAxisRef = 0.0;													//Assign open loop speed reference 
										gyroAngleSP = startRightSwitchRightFirstAngle;					//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Right side motor speed reference
		
										startRightSwitchRightTime[5] = autoSequenceTime;				//Start time of step 3 when this step exits
										if ((autoSequenceTime - startRightSwitchRightTime[4]) > startRightSwitchRight[4] || ((gyroAngleFB < startRightSwitchRightFirstAngle) && (gyroAngleFB > 200)))
											{
												leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
												rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												leftMotorSpdRefAut=0;										//Stop motion
												rightMotorSpdRefAut=0;										//Stop motion
												startRightSwitchRightSeq = 5;								//Move to step 5
											}
									}	//End of Step 4: Use gyro correction to move to correct angle
									
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 3: Start Decel to min speed, 
								 * Start rotate CCW to 270 Deg during decel
								 ****************************************************/
								if (startRightSwitchRightSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel
									{
										System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightSwitchRightTime[3]));
										yAxisRef = 0.00;													//Assign open loop speed reference 
										gyroAngleSP = startRightSwitchRightFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
								
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef - .17));						//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.17 );						//Right side motor speed reference	
		
										startRightSwitchRightTime[4] = autoSequenceTime;					//Start time of step 4 when this step exits
										if (((autoSequenceTime - startRightSwitchRightTime[3]) > startRightSwitchRight[3]) || ((gyroAngleFB < (startRightSwitchRightFirstAngle+2)) && (gyroAngleFB > 200)))
											{
								    			startRightSwitchRightSeq = 4;								//Move to step 4
											}
									}	//End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
									
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 2: Drop intake arms, start moving forward to 
								 * position left of scale,start raising elevator
								 ****************************************************/
								if (startRightSwitchRightSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									{
										System.out.println("Step 2 pos: " + rightMotorPosition + "," + (autoSequenceTime - startRightSwitchRightTime[2]));
										intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
										intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
								
										leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Right Drive motors
										leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Right Drive motors
							
										yAxisRef = 0.58;												//Assign open loop speed reference 
										gyroAngleSP = 3.0;												//Assign gyro angle setpoint in 0-359 degrees
									
										gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
										leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
										rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Right side motor speed reference
										
										if (rightMotorPosition > 12000)
											winchMotor.set(ControlMode.MotionMagic, winchSwitchRef);	//Start magic motion for absolute position to move to SWITCH
											
										startRightSwitchRightTime[3] = autoSequenceTime;				//Start time of step 3 when this step exits
										if ((autoSequenceTime - startRightSwitchRightTime[2]) > startRightSwitchRight[2] || (rightMotorPosition > startRightSwitchRightFirstPosition) )
											{
												startRightSwitchRightSeq = 3;
											}
									}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									
									
								/****************************************************
								 * START RIGHT - SWITCH RIGHT
								 * Step 1: Close intake arms solenoid
								 ****************************************************/
								if ((startRightSwitchRightSeq == 1))			//Step 1: Close intake arms solenoid
									{
										System.out.println("Step 1 : " + (autoSequenceTime - startRightSwitchRightTime[1]));	
										intakeArmOpen.set(false);									//Intake arms OPEN solenoid
										intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
											
										startRightSwitchRightTime[2] = autoSequenceTime;
										if ((autoSequenceTime - startRightSwitchRightTime[1]) > startRightSwitchRight[1])
								    		{
								    			startRightSwitchRightSeq = 2;						//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
								    		}
									}	//End of Step 1: Close intake arms solenoid	
	
							}	//End of Go for Right switch starting from Right side
				
						
	    				/**************************************************************
	    				 * Robot START=RIGHT Side,  Go for RIGHT SCALE
	    				 *   system clock is in milliseconds
	    				 **************************************************************/
							if ((ourGameData.equals("LR")) || (goForScale))	//Go for Left scale starting from Left side
//							if (testStartLeftSwitchLeft)
	    						{
									
									startRightScaleRight[0]  = 0;							//Step 0: Start of sequence
									startRightScaleRight[1]  = 50;						//Step 1: Close intake arms solenoid
									startRightScaleRight[2]  = 3500;						//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
									startRightScaleRight[3]  = 2550;						//Step 3: Start Decel to min speed, Start rotate CW to 90 Deg during decel
									startRightScaleRight[4]  = 400;						//Step 4: Use gyro correction to move to correct angle 
									startRightScaleRight[5]  = 1000;						//Step 5: Reset Drive encoders to zero, Raise elevator
									startRightScaleRight[6]  = 2800;						//Step 6: Start moving forward toward scale  1050
									startRightScaleRight[7]  = 600;						//Step 7: Stop forward movement and Open Arms 400
									startRightScaleRight[8]  = 1200;						//Step 8: Back up robot away from scale 1200
									startRightScaleRight[9]  = 1500;						//Step 9: Move elevator down, close intake arms 500
									startRightScaleRight[10] = 500;							//Autonomous movement
									startRightScaleRight[11] = 0;							//Autonomous movement
									startRightScaleRight[12] = 0;							//Autonomous movement
									startRightScaleRight[13] = 0;							//Autonomous movement
									startRightScaleRight[14] = 14900;						//Autonomous movement is done
									startRightScaleRightFirstPosition = 48500;			//Encoder counts - first move forward
									startRightScaleRightFirstAngle = 310;					//Gyro angle for first turn toward scale
									startRightScaleRightSecondPosition = 8000;			//Encoder counts - second move forward
									startRightScaleRightIntakeSpdRef = -0.2;				//Intake motor speed ref to spit cube out
									startRightScaleRightThirdPosition = -8500;			//Encoder counts - third move backward
									startRightScaleRightSecondAngle = 330;					//Gyro angle for second turn toward scale
									/****************************************************
									 * Step 0 of start on Left, SCALE on Left INIT
									 ****************************************************/
									if (startRightScaleRightSeq == 0)			//time = 0
										{
											System.out.println("Step 0 : " + autoSequenceTime);
											leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
											leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);			//Open loop ramp rate
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
											startRightScaleRightSeq = 1;									//Begin step 1
											startRightScaleRightTime[1] = autoSequenceTime;				//Start time of step 1 when this step exits
										}	//End of Step 0	
										
										
									
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 10: Move elevator down, close intake arms
									 ****************************************************/
									if (startRightScaleRightSeq == 10)		//Step 10: Move elevator down, close intake arms
										{
											System.out.println("Step 10 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[10]));
											leftMotorSpdRefAut = 0.0;								//Assign open loop speed reference
											rightMotorSpdRefAut = 0.0;								//Assign open loop speed reference
											intakeArmOpen.set(false);								//Intake arms OPEN solenoid
											intakeArmClose.set(true);								//Intake arms CLOSE solenoid
													
											winchMotor.set(ControlMode.MotionMagic, winchBottomRef);	//Start magic motion for absolute position to move to SWITCH

											startRightScaleRightTime[11] = autoSequenceTime;			//Start time of step 10 when this step exits			
											if (((autoSequenceTime - startRightScaleRightTime[10]) > startRightScaleRight[10]) )
												{
													startRightScaleRightSeq = 11;					//Move to step 11
												}
												
										}	//End of Step 10: Move elevator down, close intake arms
											
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 9: Back up robot away from scale
									 ****************************************************/
									if (startRightScaleRightSeq == 9)		//Step 9: Back up robot away from scale
										{
											System.out.println("Step 9 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[9]) + " , Gyro: " + gyroAngleFB);
											yAxisRef = -0.30;											//Assign open loop speed reference 
											gyroAngleSP = startRightScaleRightSecondAngle;				//Assign gyro angle setpoint in 0-359 degrees
									
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);		//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));	//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);				//Left side motor speed reference
													
											leftIntake.set(0.0);										//Assign speed ref to Left PWM Talon
											rightIntake.set(0.0);										//Assign speed ref to Left PWM Talon

											startRightScaleRightTime[10] = autoSequenceTime;				//Start time of step 9 when this step exits			
											if (((autoSequenceTime - startRightScaleRightTime[9]) > startRightScaleRight[9])
											|| (rightMotorPosition < startRightScaleRightThirdPosition) )
												{
													leftMotorSpdRefAut=0;								//Stop motion
													rightMotorSpdRefAut=0;								//Stop motion
													startRightScaleRightSeq = 10;							//Move to step 10
												}
										}	//End of Step 9: Back up robot away from scale
											

									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 8: Stop forward movement and Open Arms
									 ****************************************************/
									if (startRightScaleRightSeq == 8)		//Step 8: Stop forward movement and Open Arms
										{
											System.out.println("Step 8 : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[8]) + " , Gyro: " + gyroAngleFB);
											leftMotorSpdRefAut=0;										//Stop motion
											rightMotorSpdRefAut=0;										//Stop motion
												
											leftIntake.set(startRightScaleRightIntakeSpdRef);				//Assign speed ref to Left PWM Talon
											rightIntake.set(-1.0 * startRightScaleRightIntakeSpdRef);		//Assign speed ref to Left PWM Talon
												
											intakeArmOpen.set(true);									//Intake arms OPEN solenoid
											intakeArmClose.set(false);									//Intake arms CLOSE solenoid	

											startRightScaleRightTime[9] = autoSequenceTime;				//Start time of step 8 when this step exits			
											if (((autoSequenceTime - startRightScaleRightTime[8]) > startRightScaleRight[8]) )
												{
													leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
													rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
													startRightScaleRightSeq = 9;							//Move to step 9
												}
										}	//End of Step 8: Stop forward movement and Open Arms
										
									
									
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 7: Start rotate CCW to 0 Deg during decel
									 ****************************************************/
									if (startRightScaleRightSeq == 7)	//Step 7: Start Decel to min speed, Start rotate CW to 30 Deg during decel
										{
											System.out.println("Step 7 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightScaleRightTime[3]));
											yAxisRef = 0.15;													//Assign open loop speed reference 
											gyroAngleSP = 0.0;							//Assign gyro angle setpoint in 0-359 degrees
									
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.0));					//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.25 );						//Left side motor speed reference	

//											winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);			//Start magic motion for absolute position to move to SWITCH
											
											startRightScaleRightTime[8] = autoSequenceTime;						//Start time of step 4 when this step exits
											if (((autoSequenceTime - startRightScaleRightTime[7]) > startRightScaleRight[7])
											|| ((gyroAngleFB < (gyroAngleSP+2)) && (gyroAngleFB < 100)))
												{
													startRightScaleRightSeq = 8;									//Move to step 4
												}
										}	//End of Step 7: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
									

									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 6: Start moving forward toward scale
									 ****************************************************/
									if (startRightScaleRightSeq == 6)	//Step 6: Start moving forward toward scale
										{
											System.out.println("Step 6LC : " + rightMotorPosition + ", " + (autoSequenceTime - startRightScaleRightTime[6]) + " , Gyro: " + gyroAngleFB);
											yAxisRef = 0.20;												//Assign open loop speed reference 
											gyroAngleSP = startRightScaleRightFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
									
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

											startRightScaleRightTime[7] = autoSequenceTime;					//Start time of step 7 when this step exits	
											if (((autoSequenceTime - startRightScaleRightTime[6]) > startRightScaleRight[6]) 
											|| (rightMotorPosition > startRightScaleRightSecondPosition) )
												{
													leftMotorSpdRefAut=0;									//Stop motion
													rightMotorSpdRefAut=0;									//Stop motion
													startRightScaleRightSeq = 7;								//Move to step 7
												}
										}	//End of Step 6: Start moving forward toward scale	  
											
										
										
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 5: Reset Drive encoders to zero, start raising elevator
									 ****************************************************/
									if ((startRightScaleRightSeq == 5))			//Step 5: Reset Drive encoders to zero
										{
											//System.out.println("Step 5 : " + (autoSequenceTime - startRightScaleRightTime[5]));	
											//System.out.println("Winch Pos: " + winchMotorPosition);
											leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
											rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
												
											//winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);	//Start magic motion for absolute position to move to SWITCH
											
											startRightScaleRightTime[6] = autoSequenceTime;
											if ((autoSequenceTime - startRightScaleRightTime[5]) > startRightScaleRight[5])
													{
														startRightScaleRightSeq = 6;						//Move to step 6
													}
										}	//End of Step5 Reset Drive encoders to zero								
										
							
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 4: Use gyro correction to move to correct angle 
									 ****************************************************/
									if (startRightScaleRightSeq == 4)			//Step 4: Use gyro correction to move to correct angle 
										{
											System.out.println("Step 4g pos: " + gyroAngleFB + "," + (autoSequenceTime - startRightScaleRightTime[4]));

											yAxisRef = 0.2;													//Assign open loop speed reference 
											gyroAngleSP = startRightScaleRightFirstAngle;						//Assign gyro angle setpoint in 0-359 degrees
										
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef + 0.0);					//Left side motor speed reference

											startRightScaleRightTime[5] = autoSequenceTime;					//Start time of step 5 when this step exits
///////////////////////////////////////////
											if ((autoSequenceTime - startRightScaleRightTime[4]) > startRightScaleRight[4] 
											|| ((gyroAngleFB > startRightScaleRightFirstAngle) && (gyroAngleFB < 100)))
												{
													leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder
													rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);	//Zero the position encoder	
													leftMotorSpdRefAut=0;									//Stop motion
													rightMotorSpdRefAut=0;									//Stop motion
													startRightScaleRightSeq = 5;								//Move to step 5
												}
										}
										
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 3: Start Decel to min speed, 
									 * Start rotate CW to 30 Deg during decel
									 ****************************************************/
									if (startRightScaleRightSeq == 3)	//Step 3: Start Decel to min speed, Start rotate CW to 30 Deg during decel
										{
											System.out.println("Step 3 : " + gyroAngleFB + ", " + (autoSequenceTime - startRightScaleRightTime[3]));
											yAxisRef = 0.20;													//Assign open loop speed reference 
											gyroAngleSP = startRightScaleRightFirstAngle;							//Assign gyro angle setpoint in 0-359 degrees
									
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);				//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + 0.15));					//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef - 0.0 );						//Left side motor speed reference	

											System.out.println("Step 3 : " + (autoSequenceTime - startRightScaleRightTime[5]));	
					//						System.out.println("Winch Pos: " + winchMotorPosition);
											
//											winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);			//Start magic motion for absolute position to move to SWITCH
											winchMotor.set(ControlMode.MotionMagic, winchScaleTopRef);	//Start magic motion for absolute position to move to SWITCH
											
											startRightScaleRightTime[4] = autoSequenceTime;						//Start time of step 4 when this step exits
											if (((autoSequenceTime - startRightScaleRightTime[3]) > startRightScaleRight[3])
											|| ((gyroAngleFB < (startRightScaleRightFirstAngle+2)) && (gyroAngleFB > 200)))
												{
													startRightScaleRightSeq = 4;									//Move to step 4
												}
										}	//End of Step 3: Start Decel to min speed, Start rotate CW to 270 Deg during decel	
										
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 2: Drop intake arms, start moving forward to 
									 * position left of SCALE
									 ****************************************************/
									if (startRightScaleRightSeq == 2)			//Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
										{
											System.out.println("Step 2 pos: " + rightMotorPosition + "," + autoSequenceTime + " gyro: " + gyroAngleFB);
											
											intakeArmPivotDOWN.set(true);									//Enable Intake Arm to pivot DOWN
											intakeArmPivotUP.set(false);									//Enable Intake Arm to pivot UP
										
											leftMotorShiftHigh.set(false);									//Enable high speed gear on Left/Left Drive motors
											leftMotorShiftLow.set(true);									//Disable high speed gear on Left/Left Drive motors
								
											yAxisRef = 0.85;												//Assign open loop speed reference 
											gyroAngleSP = 0.0;												//Assign gyro angle setpoint in 0-359 degrees
										
											gyroAngleSync = syncAngle( gyroAngleSP, gyroAngleFB);			//Sync robot to gyro angle setpoint
											leftMotorSpdRefAut = -1.0 * ((yAxisRef + gyroAngleSync));		//Left side motor speed reference
											rightMotorSpdRefAut = 1.0 * (yAxisRef);							//Left side motor speed reference

											startRightScaleRightTime[3] = autoSequenceTime;					//Start time of step 3 when this step exits
											if ((autoSequenceTime - startRightScaleRightTime[2]) > startRightScaleRight[2] 
											|| (rightMotorPosition > startRightScaleRightFirstPosition) )
												{
													startRightScaleRightSeq = 3;
												}
										}	//End of Step 2: Drop intake arms, start moving forward to position left of scale,start raising elevator
										
										
									/****************************************************
									 * START RIGHT - SCALE RIGHT
									 * Step 1: Close intake arms solenoid
									 ****************************************************/
									if ((startRightScaleRightSeq == 1))			//Step 1: Close intake arms solenoid
										{
											System.out.println("Step 1 : " + autoSequenceTime);	
											intakeArmOpen.set(false);									//Intake arms OPEN solenoid
											intakeArmClose.set(true);									//Intake arms CLOSE solenoid	
											
											
											if (gyroYaw < 0)										//Left half of compass
												firstGyroScan = 180 + (180 + gyroYaw);				//gyro raw -179 to 0 = gyro compass angle +181 to 359
											else													//Right half of compass
												firstGyroScan = gyroWithDrift;						//gyro raw 0 to 179 = gyro compass angle 0 to 179
											
													
											startRightScaleRightTime[2] = autoSequenceTime;
											if ((autoSequenceTime - startRightScaleRightTime[1]) > startRightScaleRight[1])
													{
														startRightScaleRightSeq = 2;						//Enable step 2 Drop intake arms, start moving forward to position left of scale,start raising elevator
													}
										}	//End of Step 1: Close intake arms solenoid	

	    					}	//End of START RIGHT, go for SCALE RIGHT
	
						

      		}	//End of robot starts on right
    			
    			
      	//		System.out.println("Gyro = " + gyroAngleFB);
      	leftMotor.set(ControlMode.PercentOutput, leftMotorSpdRefAut);	//Assign speed ref to Left CAN bus Talon
				leftSlaveMotor.follow(leftMotor);								//Assign Left Slave motor to follow Left Talon motor
				rightMotor.set(ControlMode.PercentOutput, rightMotorSpdRefAut);	//Assign speed ref to Right CAN bus Talon
				rightSlaveMotor.follow(rightMotor);								//Assign Right Slave motor to follow Left Talon motor
				updateNetworkTables();

		
			}	//End of Autonomous Periodic

		public void teleopPeriodic() 		//Action routine: code called periodically during teleop
			{
				/**********************************************************************************************************************
				 * Variable Declarations for Teleop Periodic
				 **********************************************************************************************************************/
			
				/*****************************************
				 * XBox Controller assignments
				 *  1=A
				 *  2=B
				 *  3=X
				 *  4=Y
				 *  5=Left bumper
				 *  6= Right Bumper
				 *  Axes2= Left Trigger
				 *  Axes3= Right trigger
				 *  
				 *  POV = -1 when not moving
				 *  0 = Top
				 *  90 = Right
				 *  180 = Bottom
				 *  270 = Left
				 *****************************************/
				xBoxAnalog[0] = xBox.getRawAxis(0);				//Left joystick left-right motion
				xBoxAnalog[1] = xBox.getRawAxis(1);				//Left joystick up-down motion 
				xBoxAnalog[2] = xBox.getRawAxis(2);				//Press Left Trigger to adjust speed of climber motor
	    		xBoxAnalog[3] = xBox.getRawAxis(3);				//Press Right Trigger to turn on shooter if trigger passes threashold
	    		xBoxAnalog[4] = xBox.getRawAxis(4);				//Right joystick left-right motion
				xBoxAnalog[5] = xBox.getRawAxis(5);				//Right joystick up-down motion 
				
	    		xBoxButtons[1] = xBox.getRawButton(1);			//Press button 1 "A" on 
	    		xBoxButtons[2] = xBox.getRawButton(2);			//Press button 2 "B" on 
	    		xBoxButtons[3] = xBox.getRawButton(3);			//Press button 3 "X" on 
	    		xBoxButtons[4] = xBox.getRawButton(4);			//Press button 4 "Y" on 
	    		xBoxButtons[5] = xBox.getRawButton(5);			//Press button 5 "Left Bumper" on 
	    		xBoxButtons[6] = xBox.getRawButton(6);			//Press button 6 "Right Bumper" on 
	    		xBoxPOV = xBox.getPOV();						//Read the flat joystick POV on the X-Box controller
			
	    		
				/*****************************************
				 * Joystick variable assignments
				 *****************************************/
	    		double stickScale;								//0-1.0  Linearly scale all axis of motion
	    		double xAxis =  stick.getRawAxis(0);			//L:-1.0 - R:1.0 Left-to-Right motion
	    		double yAxis =  stick.getRawAxis(1);			//F:-1.0 - R:1.0 Forward-to-Reverse motion
	    		double zAxis =  stick.getRawAxis(2);			//L:-1.0 - R:1.0 Twist CCW-to-CW
	    		double slider = stick.getRawAxis(3);			//F:-1.0 - R:1.0 Slider forward-to-reverse 
	    		stickButtons[1] = stick.getRawButton(1);		//Press joystick trigger button
	    		stickButtons[2] = stick.getRawButton(2);		//Press joystick button 2
	    		stickButtons[3] = stick.getRawButton(3);		//Press joystick button 3
	    		stickButtons[4] = stick.getRawButton(4);		//Press joystick button 4
	    		stickButtons[5] = stick.getRawButton(5);		//Press joystick button 5
	    		stickButtons[6] = stick.getRawButton(6);		//Press joystick button 6
	    		stickButtons[7] = stick.getRawButton(7);		//Press joystick button 7
	    		stickButtons[8] = stick.getRawButton(8);		//Press joystick button 8
	    		stickButtons[9] = stick.getRawButton(9);		//Press joystick button 9
	    		stickButtons[10] = stick.getRawButton(10);		//Press joystick button 10
	    		stickButtons[11] = stick.getRawButton(11);		//Press joystick button 11
	    		stickButtons[12] = stick.getRawButton(12);		//Press joystick button 12
	    		

				leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);				//Open loop ramp rate
				leftMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);		//Zero the position encoder
				rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs);			//Open loop ramp rate
				rightMotor.setSelectedSensorPosition(0, 0, leftKTimeoutMs);		//Zero the position encoder
				
				
				
	    		

	    		/**********************************************************************************************************************
				 * Action Code for Teleop Periodic
				 **********************************************************************************************************************/
				count = count + 1;
				
				

				
			    /****************************************************************************************************
				 **  Read raw gyro angle from the input mod. Compensate for gyro drift by adding an offset value to
				 **  Compensate for gyro drift by adding an offset value to summation gyroDriftTotal
				 **  Scale +/- gyro values and compensate for rollover so scalled to 0-360 degrees and 
				 ****************************************************************************************************/
				gyroPitch = navxGyro.getPitch();								//Get gyro pitch because roboRIO is mounted on side
				gyroYaw = navxGyro.getYaw();									//Read gyro yaw value from navx board
				gyroDriftTotal = gyroDriftTotal + gyroDriftIncrement;			//Keep track of total gyro angle drift
				gyroWithDrift = gyroYaw + gyroDriftTotal;						//Real angle is read gyro angle plus drift compensation 

				
				if (gyroWithDrift < 0)
					gyroAngleMod = 180 + (180 + gyroWithDrift);					//gyro raw -179 to 0 = gyro compass angle +181 to 359
				else
					gyroAngleMod = gyroWithDrift;								//gyro raw 0 to 179 = gyro compass angle 0 to 179
				gyroAngleOffset = gyroAngleMod - firstGyroScan;					//Normalize gyro first scan to be zero
				
				
				
				if (gyroAngleOffset < 0)
					gyroAngleFB = gyroAngleOffset + 360;
				else
					gyroAngleFB = gyroAngleOffset;


				//System.out.println(" Gyro Yaw= " + gyroYaw + "FB: " + gyroAngleFB);
				
				
	    		
	    		/*****************************************
				 * Turn on the air compressor
				 *****************************************/
				airCompressor.setClosedLoopControl(true);					//Enable Air Compressor to be controlled by pressure switch input
	    		
	    			
	    		String autonChooser = SmartDashboard.getString("Auton Start Position/selected", "Nothing selected");
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
	    		
	    		
	    		
	    		/**************************************************
				 * Power Distribution Panel
				 * Read the current from each channel
				 *
				 * Get Talon motors position and speed feedback
				 * Use for calculating motor speed references
				 ***************************************************/
//	    		currentFdbkPDP[0] = ourPDP.getCurrent(0);									//Get PDP motor channel 0 current
//	    		currentFdbkPDP[1] = ourPDP.getCurrent(1);									//Get PDP motor channel 1 current
//	    		currentFdbkPDP[2] = ourPDP.getCurrent(2);									//Get PDP motor channel 2 current
//	    		currentFdbkPDP[3] = ourPDP.getCurrent(3);									//Get PDP motor channel 3 current
//	    		currentFdbkPDP[4] = ourPDP.getCurrent(4);									//Get PDP motor channel 4 current
//	    		currentFdbkPDP[5] = ourPDP.getCurrent(5);									//Get PDP motor channel 5 current
//	    		currentFdbkPDP[6] = ourPDP.getCurrent(6);									//Get PDP motor channel 6 current
//	    		currentFdbkPDP[7] = ourPDP.getCurrent(7);									//Get PDP motor channel 7 current
//	    		currentFdbkPDP[8] = ourPDP.getCurrent(8);									//Get PDP motor channel 8 current
//	    		currentFdbkPDP[9] = ourPDP.getCurrent(9);									//Get PDP motor channel 9 current
//	    		currentFdbkPDP[10] = ourPDP.getCurrent(10);									//Get PDP motor channel 10 current
//	    		currentFdbkPDP[11] = ourPDP.getCurrent(11);									//Get PDP motor channel 11 current
//	    		currentFdbkPDP[12] = ourPDP.getCurrent(12);									//Get PDP motor channel 12 current
//	    		currentFdbkPDP[13] = ourPDP.getCurrent(13);									//Get PDP motor channel 13 current
//	    		currentFdbkPDP[14] = ourPDP.getCurrent(14);									//Get PDP motor channel 14 current
//	    		currentFdbkPDP[15] = ourPDP.getCurrent(15);									//Get PDP motor channel 15 current
	    		leftMotorVelocity = leftMotor.getSelectedSensorVelocity(0);					//Get Left drive motor Speed from Talon SRX 
	    		leftMotorPosition = leftMotor.getSelectedSensorPosition(0);					//Get Left drive motor Position from Talon SRX
	    		rightMotorVelocity = rightMotor.getSelectedSensorVelocity(0);				//Get Right drive motor Speed from Talon SRX
	    		rightMotorPosition = rightMotor.getSelectedSensorPosition(0);				//Get Right drive motor Position from Talon SRX
	    		winchMotorVelocity = winchMotor.getSelectedSensorVelocity(0);				//Get Winch drive motor Speed from Talon SRX
	    		winchMotorPosition = winchMotor.getSelectedSensorPosition(0);				//Get Winch drive motor Position from Talon SRX
	    		winchPositionErr = winchMotor.getClosedLoopError(0);
	    		winchMotorCurrent = currentFdbkPDP[2];										//Assign current feedback
	    		
	    		
	    		/***************************************************
				 * Get navX Gyro data
				 * Use for calculating motor speed references
				 ***************************************************/
	    		gyroCompass = navxGyro.getCompassHeading();	
	    		gyroPitch = navxGyro.getPitch();
	    		gyroRoll = navxGyro.getRoll();
	    		gyroYaw = navxGyro.getYaw();
				
  		
	    		/********************************************************
	    		 * X-BOX DIGITAL TRIGGER BUTTONS 5 & 6 ON TOP
				 * Press X-BOX controller Trigger button on top for solenoids
				 * Left to Close Pickup Arms 
				 * Right to Open Pickup Arms
				 ********************************************************/
				if (xBoxButtons[5])
					{
						intakeArmOpenCommand = false;						//CLOSE the takeup arms
					}
				if (xBoxButtons[6])
					{
						intakeArmOpenCommand = true;						//OPEN the takeup arms
					}		
				intakeArmOpen.set(intakeArmOpenCommand);					//Intake arms Close solenoid
				intakeArmClose.set(!(intakeArmOpenCommand));				//Intake arms Open solenoid
				
	    		/**********************************************************
	    		 * X-BOX LEFT & RIGHT ANALOG TRIGGER BUTTONS ON BOTTOM - PWM 0 & 1
				 * Speed reference for cube pick-up motors
				 * Pressing left trigger more increases speed forward to pick-up
				 * Pressing right trigger more increases speed reverse to spit out
				 **********************************************************/
	    		if (xBoxAnalog[2] > 0.05)
	    			{
	    				cubePickUpMotorSpdRef = xBoxAnalog[2];				//Forward speed reference for cube pick-up
	    			}
	    		if (xBoxAnalog[3] > 0.05)
	    			{
	    				cubePickUpMotorSpdRef = -1.0 * xBoxAnalog[3];		//Reverse speed reference for cube pick-up
	    			}
	    		if ((xBoxAnalog[2] <= 0.05) & (xBoxAnalog[3] <= 0.05))		//Create a deadband +/- 0/05 around zero
	    			{
	    				cubePickUpMotorSpdRef = 0;							//Set speed reference to zero if in deadband
	    			}
	    		
	    		
	    		
	    		/****************************************************************************************************
	    		 * Talon/Victor motors ID = 1 & 2  and  3 & 4
	    		 * JOYSTICK POSITION processed with deadband filters to provide reference to 
	    		 * Arcade style Drive Talon 1&2 (left) & 3&4 (right)
	    		 * 	X axis = turn left-right movement, Y axis = forward movement, Z axis = rotation
	    		 ****************************************************************************************************/
	    		stickScale = (slider+1)/2;							//L:-1.0 - R:1.0 = 0-1.0  Linearly scale all axis of motion
	    	
	    		/****************************************************************************************************
	    		 * Add a dead band of movement on x-axis left-to-right
	    		 ****************************************************************************************************/
	    		if (xAxis < 0)										//x-Axis side-to-side from joystick = -1.0 to 0
	    			{
	    				if (xAxis <= -deadBandPercentX)
	    					xAxisMod = xAxis + deadBandPercentX; 	//x-Axis side-to-side outside of deadband, subtract deadband to remove DB gap
	    				else
	    					xAxisMod = 0;							//x-Axis side-to-side in deadband, set to zero		
	    			}
	    		else												//x-Axis side-to-side from joystick = 0 to 1.0
	    			{
	    				if (xAxis >= deadBandPercentX)
	    					xAxisMod = xAxis - deadBandPercentX; 	//x-Axis side-to-side outside of deadband, subtract deadband to remove DB gap
	    				else
	    					xAxisMod = 0;							//x-Axis side-to-side in deadband, set to zero
	    			}
	    	
	    		if (xAxisMod <= 0)									//Left turn - Speed up right side to turn to left
	    			{
	    				left_x_axisTurnMod = 0;
	    				right_x_axisTurnMod = 0.9 * xAxisMod;		//Speed up right side to turn to left
	    			}
	    		else												//Right turn - Speed up left side to turn to right
	    			{
	    				left_x_axisTurnMod = -0.9 * xAxisMod;		//Speed up left side to turn to right
	    				right_x_axisTurnMod = 0;		
	    			}
	  
	    		/****************************************************************************************************
	    		 * Forward and Backward movement on y-axis
	    		 ****************************************************************************************************/
	    		yAxisMod = yAxis;									//Allow normal front-to-back movement

	    		
	    		/****************************************************************************************************
	    		 * Add a dead band of movement on z-axis twist (Z-axis value from joystick = -1.0 - 1.0)
	    		 ****************************************************************************************************/
	    		if (zAxis < 0)										//z-Axis twist from joystick = -1.0 to 0
	    			{
	    				if (zAxis <= -deadBandPercentZ)
	    					zAxisMod = zAxis + deadBandPercentZ; 	//z-Axis outside of deadband, subtract deadband to remove DB gap
	    				else
	    					zAxisMod = 0;							//z-Axis in deadband, set to zero		
	    			}
	    		else												//z-Axis twist from joystick = 0 to 1.0
	    			{
	    				if (zAxis >= deadBandPercentZ)
	    					zAxisMod = zAxis - deadBandPercentZ; 	//z-Axis outside of deadband, subtract deadband to remove DB gap
	    				else
	    					zAxisMod = 0;							//z-Axis in deadband, set to zero
	    			}	
	    			/*****************************************
	    			 * Rotate the robot Counter Clock Wise
	    			 *****************************************/
	    		if (zAxisMod <= 0)									
					{
	    				left_z_axisRotMod = -1.1 * zAxisMod;		//Left side motor reverse
	    				right_z_axisRotMod = 1.1 * zAxisMod;		//Right side motor forward
					}
    				/*****************************************
    				 * Rotate the robot Clock Wise
    				 *****************************************/
	    		else
					{
	    				left_z_axisRotMod = -1.1 * zAxisMod;		//Left side motor reverse
	    				right_z_axisRotMod = 1.1 * zAxisMod;		//Right side motor forward		
					}												//End of Left/Right drive motors speed reference from JOYSTICK
	    		/**********  END of JOYSTICK position calculating  **********/
	    		
	    		
	    		/********************************************************
	    		 * X-Box LEFT JOYSTICK
				 * Winch motor manual control with 
				 * x-box left joystick up-down motion
				 ********************************************************/
	    		if ((xBoxAnalog[1] > deadBandWinch) | (xBoxAnalog[1] < -1.0 * deadBandWinch))	//joystick value is outside zero deadband
	    			{
	    				winchMotorSpdRef = -1.0 * xBoxAnalog[1];	//Winch motor manual speed refernce
	    			}
	    		else												//joystick value is inside zero deadband so ignore value
	    			{
	    				winchMotorSpdRef = 0;						//Winch speed ref is zero
	    			}
	    		

	    		
	    		/********************************************************
				 * X-BOX POV - INTAKE ARM PIVOT UP/DOWN
				 * Raise Arm Pivot if (POV>270) OR (POV<90) AND (POV>-1) 
				 * Lower Arm Pivot if (POV>90) AND (POV<270) AND (POV>-1)
				 ********************************************************/
	    		if ((xBoxPOV > -1) && ((xBoxPOV > 270) ||(xBoxPOV < 90)) )
	    			{
	    				intakeArmPivotDownCommand = false;				//Latch intake arm pivot to UP
	    			}
	    		if ( (xBoxPOV > -1) && (xBoxPOV > 90) && (xBoxPOV < 270) )
    				{
    					intakeArmPivotDownCommand = true;				//Latch intake arm pivot to UP
    				}
	    		intakeArmPivotDOWN.set(intakeArmPivotDownCommand);		//Enable Intake Arm to pivot DOWN
	    		intakeArmPivotUP.set(!(intakeArmPivotDownCommand));		//Enable Intake Arm to pivot UP
	    		
	    		/********************************************************
	    		 * JOYSTICK DIGITAL TRIGGER BUTTON ON TOP
				 * Press joystick controller Trigger button
				 * to fire both left and right motor 
				 * high speed solenoid to extend
				 ********************************************************/
	    		leftMotorShiftHigh.set(stickButtons[1]);				//Enable high speed gear on Left/Right Drive motors
	    		leftMotorShiftLow.set(!(stickButtons[1]));				//Disable high speed gear on Left/Right Drive motors
	    		
	    		
	    		/********************************************************
	    		 * JOYSTICK LEFT SIDE BUTTONS 7 & 8
				 * Press Button 7 to RUN (Disengage Lock) 
				 * Press Button 8 to LOCK (Cannot run elevator
				 ********************************************************/
				if (stickButtons[7])
					{
					winchRunCommand = true;								//Shifter solenoid ON forward to RUN
					}
				if (stickButtons[8])
					{
					winchRunCommand = false;							//Shifter solenoid OFF back to LOCK
					}		
				winchMotorRun.set(winchRunCommand);						//Intake arms Close solenoid
				winchMotorLock.set(!(winchRunCommand));					//Intake arms Open solenoid
	    		
	    		
	    		/***************************************************
				 * Calculate left & right motor speed references
				 ***************************************************/
	        	leftMotorSpdRef = 1.0 * (yAxis + left_x_axisTurnMod + left_z_axisRotMod);		//Left side motor speed reference
	        	rightMotorSpdRef = -1.0 * (yAxis + right_x_axisTurnMod + right_z_axisRotMod);	//Right side motor speed reference
	        									
	        	/***************************************************
				 * Assign speed references to PWM motors
				 ***************************************************/
	        	leftIntake.set(cubePickUpMotorSpdRef);								//Assign speed ref to Left PWM Talon
	        	rightIntake.set(-1.0 * cubePickUpMotorSpdRef);						//Assign speed ref to Right PWM Talon

	        	/***************************************************
				 * Assign speed references to CAN bus motors
				 ***************************************************/
	        	if (winchMotorPosition > winchSwitchRef)
	        		{
	        			leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);						//Rate = [1023-0] / [500 msec] * 10 msec = 20
	        			rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs);					//Rate = [1023-0] / [500 msec] * 10 msec = 20
	        		}
	        	else
	        		{
	        			leftMotor.configOpenloopRamp(0.5, leftKTimeoutMs);						//Rate = [1023-0] / [500 msec] * 10 msec = 20
	        			rightMotor.configOpenloopRamp(0.5, rightKTimeoutMs);					//Rate = [1023-0] / [500 msec] * 10 msec = 20
	        		}
	        	leftMotor.set(ControlMode.PercentOutput, (stickScale * leftMotorSpdRef));		//Assign speed ref to Left CAN bus Talon
	        	leftSlaveMotor.follow(leftMotor);												//Assign Left Slave motor to follow Left Talon motor
	        	rightMotor.set(ControlMode.PercentOutput, (stickScale * rightMotorSpdRef));		//Assign speed ref to Right CAN bus Talon
	        	rightSlaveMotor.follow(rightMotor);												//Assign Right Slave motor to follow Left Talon motor
	        	
	        	
	    		/*****************************************
	    		 * WINCH MOTOR POSITION REFERENCES
	    		 * 
	    		 * Press X-Box controller button[1]  "A"
				 * to start motion magic profile to run 
				 * winch motor to Bottom position
	    		 * 
	    		 * Press X-Box controller button[2]  "B"
				 * to start motion magic profile to run 
				 * winch motor to Scale position
				 * 
	    		 * Press X-Box controller button[3]  "X"
				 * to start motion magic profile to run 
				 * winch motor to Switch position
				 * 
				 * JOYSTICK DIGITAL BUTTON 11
				 * Press joystick controller button 11
				 * to start winch motor climbing
				 *****************************************/
	         	if (xBoxButtons[1])										//"A" = Drive the elevator to Bottom Position
					{		
	        			winchManualOffset = 0;
	        			winchPresetPosition = winchBottomRef;			//Assign absolute position to move to BOTTOM
	        			enableClimbMode = false;						//Disable Climbing mode if accidently set
					}
	        	else if (xBoxButtons[2])								// "B" = Drive the elevator to Scale Position
    				{
	        			winchManualOffset = 0;
	        			winchFeedFwdFix = 0;	
	        			winchPresetPosition =  winchScaleRef;			//Assign absolute position to move to SCALE
	        			enableClimbMode = false;						//Disable Climbing mode if accidently set
    				}
	        	else if (xBoxButtons[3])								//"X" = Drive the elevator to Scale Position
					{
	        			winchManualOffset = 0;
	        			winchFeedFwdFix = 0;		
	        			winchPresetPosition =  winchSwitchRef;			//Assign absolute position to move to SWITCH
	        			enableClimbMode = false;						//Disable Climbing mode if accidently set
					}
	        	else if (stickButtons[11])								//Start robot climb. Robot is hooked, drive elevator to bottom position
	        		{
		         		if (! climbStartTime)
		         			{
		         				climbTimeStartSnapshot = currentSystemClock;		//Snapshot system time at start of autonomous
		         				climbStartTime = true;								//turn on first scan flag
		         			}
	        			enableClimbMode = true;										//Enable Climbing mode
	    	    		winchMotorLock.set(false);								//Shift the winch motor to high speed
	    	    		winchMotorRun.set(true);								//Shift the winch motor to high speed
	        			winchManualOffset = 0;
	        			winchPresetPosition = winchBottomRef;						//Start magic motion for absolute position to move to BOTTOM
	        		}
	   
	         	
	        	else
    				{
	        			winchManualOffset = winchManualOffset + (150 * winchMotorSpdRef);
    					//winchMotor.set(ControlMode.PercentOutput, (stickScale * winchMotorSpdRef));	//Assign speed ref to Winch CAN bus Talon
    				}	
	        	
         	
	         	if (enableClimbMode)
	         		{
	         			climbTimeElapsed = currentSystemClock - climbTimeStartSnapshot;				//Calc time elapsed for doing climb
	         			if ((climbTimeElapsed > 1000) && (winchMotorPosition < (winchBottomRef + 300)))
	         			{
	         				//latchMotor.set(ControlMode.Position, latchEngagedRef);
	         			}
	         		}
	         	else
	         		{
	         		climbTimeElapsed = 0;															//Elapsed time is zero if not actually climbing
	         		}
	         	
	        	winchTargetAbsolutePosition =  winchPresetPosition + winchFeedFwdFix + winchManualOffset;
	      
	        	if (xBoxButtons[4]			//Press the X-Box "Y" button
	        		|| (stickButtons[7]) )
	        		winchMotor.set(ControlMode.PercentOutput, (stickScale * 1.0 * winchMotorSpdRef));	//Assign speed ref to Winch CAN bus Talon

	        	else
	        		winchMotor.set(ControlMode.MotionMagic, winchTargetAbsolutePosition);				//Start magic motion for absolute position move
	        	
	        	//winchMotor.set(ControlMode.Position, winchTargetAbsolutePosition);
	        	winchSlaveMotor.follow(winchMotor);														//Assign Winch Slave motor to follow Left Talon motor
	        	
	        	
	    		/***************************************************
				 * Output to console: buttons and pads from xbox and joystick
				 ***************************************************/
	    		//System.out.println( "X-Box Left stick left-right = " + xBoxAnalog[0]); 
	    		//System.out.println( "X-Box Left stick up-down = " + xBoxAnalog[1]); 
	    		//System.out.println( "Left Trigger= " + xBoxAnalog[2]); 
	    		//System.out.println( "Right Trigger= " + xBoxAnalog[3]); 
	    		//System.out.println( "X-Box Right stick left-right = " + xBoxAnalog[4]); 
	    		//System.out.println( "X-Box Right stick up-down = " + xBoxAnalog[5]);
	    		//System.out.println("Button A " + xBoxButtons[1]);
	    		//System.out.println("Button B " + xBoxButtons[2]);
	    		//System.out.println("Joystick Trigger " + stickButtons[1]);
	    		
	    		/***************************************************
				 * Output to console: Talon motors position and speed feedback 
				 ***************************************************/
	    		//System.out.println((currentSystemClock - firstClockScan) + ", " + slider);
	        	//System.out.println("System Clock = " + currentSystemClock);
	        	//System.out.println("Climb Start Time = " + climbTimeStartSnapshot);
	        	//System.out.println("Climb Time = " + climbTimeElapsed);
	        	//System.out.println("leftMotorSpdRef = " + leftMotorSpdRef);
	        	//System.out.println("rightMotorSpdRef = " + rightMotorSpdRef);
	    		//System.out.println("Left Motor Position = " + leftMotorPosition);
	    		//System.out.println("Left Motor Velocity = " + leftMotorVelocity);
	    		//System.out.println("Right Motor Position = " + rightMotorPosition);
	    		//System.out.println("Right Motor Velocity = " + rightMotorVelocity);
	    		System.out.println("Winch Motor Position = " + winchMotorPosition);
	    		//System.out.println("Winch Position Setpoint = " + winchTargetAbsolutePosition);
	    		//System.out.println("Winch Position Error = " + winchPositionErr);
	    		//System.out.println("Winch Motor Velocity = " + winchMotorVelocity);
	    		//System.out.println("Winch Manual Ref = " + winchManualOffset);
	    		//System.out.println("Winch Motor Current = " + currentFdbkPDP[2]);

	        	//System.out.println("Gyro Compass= " + gyroCompass);
	        	//System.out.println("Gyro Pitch= " + gyroPitch);
	        	//System.out.println("Gyro Roll= " + gyroRoll);
	        	//System.out.println("Gyro Yaw= " + gyroYaw);
				//System.out.println("Gyro Angle= " + gyroAngleFB);
	    		//System.out.println("Cube pickup ref = " + cubePickUpMotorSpdRef);
	        	//System.out.println("xBox POV = " + xBoxPOV);
	        	//System.out.println("Arm Pivot = " + intakeArmPivotDownCommand);
	    		//System.out.println("Arm Close = " + intakeArmOpenCommand);
	    		//System.out.println("match start position = " + matchStartPosition);
	    		
	        	//System.out.println("scan: " + count +" Gyro Yaw= " + gyroYaw + "FB: " + gyroAngleFB);
	        	

	    		updateNetworkTables();
			}	//End of Teleop Periodic


		public void testPeriodic() 
			{
			updateNetworkTables();
		
			}	//End of Test Periodic
		

		

		
		
		/***************************************************
		 *  Populate the network tables with data.
		 *  We can pull this data from the network tables and place it on the Shuffleboard.
		 *  This is placed inside Robot, but is its own method that can be called from the other iterative (periotic and init) methods
		 *  Simply call `updateNetworkTables()` in each method to place this data in the network tables. 
		 *  Add values by using `SmartDashboard.put[type]. For example:
		 *  SmartDashboard.putBoolean()
		 *  SmartDashboard.putNumber() // Doubles
		 *  SmartDashboard.putString()
		 *  
		 *  After these are placed, the Shuffleboard widgets can pull data from the network table entries.
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
					// Get PDP readings - WARNING: THIS MAY NOT WORK. Last time I tried it, it didn't work.
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
				
				//*****************************EXPERIMENTAL*****************************\\
				/**
				 * An experimental method that will allow us to place values in the network tables 
				 * depending on what mode we are in. Simply call it like so:
				 * _updateNetworkTables(_driveMode.auton); // If we are in auton
				 * _updateNetworkTables(_driveMode.teleop); // If we are in teleop
				 *  // Etc...
				 * @param driveMode The drive mode to set network table values according to.
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
				//*************************END EXPERIMENTAL*****************************\\
				
				
				public static double syncAngle( double angleSP, double angleFB )
				{
				double syncAngleOut = 0;								//calc'ed output for method
				double angleErr = 0;									//calc'ed angle err (SP-FB)
				double angleErrAdj=0;									//error adjusted to be +/- 180
				double angleErrMod = 0;									//Angle Error modified by scale factor
				double absAngleErrMod = 0;								//Absolute Value of Angle Error modified by scale factor
				double absSyncAngleOut = 0;								//Absolute value of reference to move robot
				double correctDirection = 1.0;							//Direction to rotate for shortest distance to setpoint
				final double zAxisCorrectScale = .022;					//Scale gyro rotation drift to send ref to z-axis
				final double gyroMaxZaxisCorrect = 0.24;				//Maximum amount of z-axis correction due to rotation drift	
				final double minRotationThreash = 0.01;					//Min Z-Axis rotation threashold to try to correct for
				final double minSpeedRef = 0.035;						//Min speed reference to move robot

				
				
				angleErr = (angleSP - angleFB);							//calc angle error modified value to get back on course

				angleErrAdj = angleErr;
				if ((angleSP > angleFB) && (angleFB < 180) && (angleSP > 180))
					angleErrAdj = angleErr -360;
				if ((angleSP <= angleFB) && (angleFB >= 180) && (angleSP < 180))
					angleErrAdj = angleErr + 360;

				angleErrMod = angleErrAdj * zAxisCorrectScale;			//calc angle error modified value to get back on course
				absAngleErrMod = Math.abs(angleErrMod);					//Find absolute value of angle error modified by scale factor

				//System.out.println("angleSP=" + angleSP);
				//System.out.println("angleFB=" + angleFB);
				//System.out.println("angleErr" + angleErr);
				//System.out.println("angleErrAdj=" + angleErrAdj);
				//System.out.println("angleErrMod=" + angleErrMod);
				//System.out.println("");
				
				/****************************************************************************************************
				 **  Max positive correction is limited to max positive correction reference value
				 **  Max negative correction is limited to max negative correction reference value
				 ****************************************************************************************************/
				if (absAngleErrMod >= gyroMaxZaxisCorrect)				//Limit max output
					{
						if (angleErrMod > 0)							//Max positive reference
							syncAngleOut = gyroMaxZaxisCorrect;			//Limit angular reference to send to z-axis
						else											//Max negative reference
							syncAngleOut = (-1.0 * gyroMaxZaxisCorrect);//Limit angular reference to send to z-axis
						//System.out.println("max correct=" + syncAngleOut);	
					}
				/****************************************************************************************************
				 **  In between Max correction and Min correction - reference is scaled with angle error value
				 ****************************************************************************************************/	
				else if (absAngleErrMod < gyroMaxZaxisCorrect)			//Error is less than max output limit
					syncAngleOut = angleErrMod;							//Scale angular reference to send to z-axis
				/****************************************************************************************************
				 **  Correction is less than threshold to care about - reference is set to zero
				 ****************************************************************************************************/	
				else if (absAngleErrMod < minRotationThreash)			//Error is less than max output limit
					syncAngleOut = 0;									//Scale angular reference to send to z-axis	
				/****************************************************************************************************
				 **  If the speed reference is too low, the robot won't move so set set a min speed ref
				 ****************************************************************************************************/	
				absSyncAngleOut = Math.abs(syncAngleOut);				//Absolute value of speed reference
				if (absSyncAngleOut < minSpeedRef)						//Speed reference is less than value to cause movement
					{
						if (syncAngleOut < 0.0)							//Scale negative angular reference to send to z-axis
							syncAngleOut = (-1.0 * minSpeedRef);
						else
							syncAngleOut = minSpeedRef;					//Scale positive angular reference to send to z-axis
					}
			
				return syncAngleOut;									//Assign method returned value - must be last line
				
			}// end of syncAngle method
		
		
}	// End of Robot class Iterative
