package frc.robot;

import frc.robot.Autonomous.AutoStates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {	
	static AutoStates autoSelected;
	SendableChooser<AutoStates> chooser = new SendableChooser<>();
	//static Camera camera;
	static Drivebase drivebase;
	static Timer timer = new Timer();
	static Autonomous auto;
	static NavX navX;
	static boolean autoRunOnce = false;
	static PowerDistributionPanel pdp;
	static NetworkTableInstance inst;
	static NetworkTable visionTable;
	private static NetworkTableEntry widthEntry;
	private static NetworkTableEntry centerXEntry;
	private static NetworkTableEntry areaEntry;
	private static NetworkTableEntry visionStringArrEntry;

	private static double width;
	private static double centerX;
	private static double area;
	public static Joystick left;
	public static Camera camera;
	public static Joystick right;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		timer.start();
		drivebase = new Drivebase();
		left = new Joystick(0);
		right = new Joystick(1);
		camera = new Camera();

		navX = new NavX();
		pdp = new PowerDistributionPanel(RobotMap.PDP);
		navX.navX.zeroYaw();
		inst = NetworkTableInstance.getDefault();

		Robot.navX.zeroYaw();
		Robot.drivebase.zeroEncoder();

		/*visionTable = inst.getTable("JetsonTable");	
		widthEntry = visionTable.getEntry("Width");	
		centerXEntry = visionTable.getEntry("Center X");
		areaEntry = visionTable.getEntry("Area");
		visionStringArrEntry = visionTable.getEntry("String Array");*/

		width = 0.0;
		centerX = 0.0;
		area = 0.0;
	}

	@Override
	public void robotPeriodic() {
		updateSmartDashboard();
		
	}
	
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		auto = new Autonomous();
		auto.assembleTest();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {		
		auto.run();
	}

	@Override
	public void teleopInit() {
		navX.navX.zeroYaw();
		timer.reset();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		OI.update();

		if (OI.lBtn[2]) {
			navX.navX.zeroYaw();
		}

		if (OI.lBtn[3]) {
			Robot.drivebase.zeroEncoder();
		}


		Robot.drivebase.drive(OI.lY, OI.rY);
	}

	public static String[] getStringArr() {
		return visionStringArrEntry.getStringArray(new String[0]);
	}


	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledPeriodic() {
		autoRunOnce = false;
	}

	public void updateSmartDashboard() {
		
		SmartDashboard.putNumber("Left Encoder", Robot.drivebase.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.drivebase.getRightEncoder());
		SmartDashboard.putNumber("NavX Yaw", Robot.navX.getYaw());

	}


}

