package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    
	  private WPI_TalonSRX rightBack;
	  private WPI_TalonSRX rightFront;
	  private WPI_TalonSRX leftFront;
    private WPI_TalonSRX leftBack;
    
	  SpeedControllerGroup rightMotors;
	  SpeedControllerGroup leftMotors;
	  DifferentialDrive drive;

    public final Encoder m_leftEncoder; 
    public final Encoder m_rightEncoder; 
    private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {
        
        rightBack = new WPI_TalonSRX(RobotMap.REAR_RIGHT);
        rightFront = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
        leftFront = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
        leftBack = new WPI_TalonSRX(RobotMap.REAR_LEFT);

        rightMotors = new SpeedControllerGroup(rightBack, rightFront);
        leftMotors = new SpeedControllerGroup(leftBack, leftFront);

        drive = new DifferentialDrive(leftMotors, rightMotors);
      
        rightBack.setInverted(RobotMap.REAR_RIGHT_INV);
        rightFront.setInverted(RobotMap.FRONT_RIGHT_INV); //true
        leftBack.setInverted(RobotMap.REAR_LEFT_INV);
        leftFront.setInverted(RobotMap.FRONT_LEFT_INV);

        m_leftEncoder = new Encoder(11, 10, true);
        m_rightEncoder = new Encoder(1, 2, false);
      
        m_leftEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);
    
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    }
    
    public void drive(double left, double right) {
      leftMotors.set(left);
      rightMotors.set(right);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
      leftMotors.setVoltage(rightVolts);
      rightMotors.setVoltage(leftVolts);      
    }

    public int getLeftEncoder() {
      return m_leftEncoder.get();
    }

    public int getRightEncoder() {
      return m_rightEncoder.get();
   }

    @Override
    public void periodic() {
      m_odometry.update(Rotation2d.fromDegrees(Robot.navX.getYaw()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }
    
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }
    
    public void resetEncoders() {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
    }   

}