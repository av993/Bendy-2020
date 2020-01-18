package frc.robot;

import frc.robot.Autonomous.AutoStates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;


public class Robot extends TimedRobot {	
	static AutoStates autoSelected;
	SendableChooser<AutoStates> chooser = new SendableChooser<>();
	//static Drivebase driveSubsystem;
	static Timer timer = new Timer();
	static Autonomous auto;
	static NavX navX;
	private static double width;
	private static double centerX;
	private static double area;
	public static Joystick left;
	public static Joystick right;
	public static Commands commands;
	public static Drivebase drivebase;
	public static Command autoCommand;

	@Override
	public void robotInit() {
		
		timer.start();
		drivebase = new Drivebase();
		commands = new Commands();
		navX = new NavX();
		navX.navX.zeroYaw();
		Robot.drivebase.zeroEncoder();
	}

	@Override
	public void robotPeriodic() {
		updateSmartDashboard();
		OI.update();
		if (OI.lBtn[2]) {
			navX.navX.zeroYaw();
		}
	}
	

	@Override
	public void autonomousInit() {

		Robot.drivebase.zeroEncoder();
		Robot.drivebase.reset();
		autoCommand = commands.getAutonomousCommand();

		System.out.println("------------------------------------------START-----------------------------------------------");
		System.out.println("------------------------------------------START-----------------------------------------------");
		System.out.println("------------------------------------------START-----------------------------------------------");
		System.out.println("------------------------------------------START-----------------------------------------------");
		System.out.println("------------------------------------------START-----------------------------------------------");
		System.out.println("------------------------------------------START-----------------------------------------------");

		autoCommand.schedule();


	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {	
		Robot.drivebase.periodic();
		CommandScheduler.getInstance().run();
		
		//System.out.println("LEFT ENC:" + Robot.drivebase.getLeftEncoder());
		//System.out.println("RIGHT ENC:" + Robot.drivebase.getRightEncoder());
		System.out.println("X Pose:" + Robot.drivebase.getPoseX());
		System.out.println("Y Pose:" + Robot.drivebase.getPoseY());
		System.out.println("Left Speed:" + Robot.drivebase.getLeftVelocity());
		System.out.println("Right Speed:" + Robot.drivebase.getRightVelocity());
		System.out.println("NavX:" + Robot.navX.getYaw());
		System.out.println("LEFT PWR:" + Robot.drivebase.leftFront.getMotorOutputVoltage());
		System.out.println("RIGHT PWR:" + Robot.drivebase.rightFront.getMotorOutputVoltage());
		System.out.println("----------BREAK-------");
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
		Robot.drivebase.periodic();

		if (OI.lBtn[2]) {
			navX.navX.zeroYaw();
		} else if (OI.lBtn[3]) {
			Robot.drivebase.zeroEncoder();
		} else {
			Robot.drivebase.drive(OI.lY, OI.rY);
		}


	}


	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Left Encoder", Robot.drivebase.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.drivebase.getRightEncoder());
		SmartDashboard.putNumber("NavX Yaw", Robot.navX.getYaw());
		SmartDashboard.putNumber("Original Heading", Robot.navX.getOriginalHeading());
		SmartDashboard.putNumber("GetPoseX", Robot.drivebase.getPoseX());
		SmartDashboard.putNumber("GetPoseY", Robot.drivebase.getPoseY());
		SmartDashboard.putNumber("Left Wheel Speed", Robot.drivebase.getLeftVelocity());
		SmartDashboard.putNumber("Right Wheel Speed", Robot.drivebase.getLeftVelocity());
		SmartDashboard.putNumber("Left Meters", Robot.drivebase.getLeftMeters());
		SmartDashboard.putNumber("Right Meters", Robot.drivebase.getRightMeters());
		SmartDashboard.putNumber("Left Feet", Units.metersToFeet(Robot.drivebase.getLeftMeters()));
		SmartDashboard.putNumber("Right Feet", Units.metersToFeet(Robot.drivebase.getRightMeters()));
		SmartDashboard.putNumber("Left Power", Robot.drivebase.leftFront.get());
		SmartDashboard.putNumber("Right Power", Robot.drivebase.rightFront.get());


	}
}