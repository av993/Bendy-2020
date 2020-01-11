package frc.robot;

import frc.robot.Autonomous.AutoStates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// TEST IMPORTS FOR TRAJECTORIES \\


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
	public static Commands container;
	public static DriveSubsystem driveSubsystem;

	@Override
	public void robotInit() {
		
		timer.start();
		drivebase = new Drivebase();
		left = new Joystick(0);
		right = new Joystick(1);
		camera = new Camera();
		container = new Commands();
		navX = new NavX();
		pdp = new PowerDistributionPanel(RobotMap.PDP);
		navX.navX.zeroYaw();
		driveSubsystem = new DriveSubsystem();
		
		Robot.navX.zeroYaw();
		Robot.drivebase.zeroEncoder();

		inst = NetworkTableInstance.getDefault();
		width = 0.0;
		centerX = 0.0;
		area = 0.0;
	}

	@Override
	public void robotPeriodic() {
		updateSmartDashboard();
	}
	

	@Override
	public void autonomousInit() {
		navX.navX.zeroYaw();
		Robot.drivebase.zeroEncoder();
		container.getAutonomousCommand().schedule();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {		
		CommandScheduler.getInstance().run();
	}

	/**
	 * Run once during operator control
	 */
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
		} else if (OI.lBtn[3]) {
			Robot.drivebase.zeroEncoder();
		}

		Robot.drivebase.drive(OI.lY, OI.rY);
		Robot.driveSubsystem.periodic();

	}



	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Left Encoder", Robot.drivebase.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.drivebase.getRightEncoder());
		SmartDashboard.putNumber("NavX Yaw", Robot.navX.getYaw());
		SmartDashboard.putNumber("Subsystem Left", Robot.driveSubsystem.getLeftEncoder());
		SmartDashboard.putNumber("Subsystem Right", Robot.driveSubsystem.getRightEncoder());


	}
}