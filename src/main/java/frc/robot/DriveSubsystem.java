package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.
    private final SpeedControllerGroup m_leftMotors = 
    new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.FROMT_LEFT),
    new WPI_TalonSRX(RobotMap.REAR_LEFT));

    // The motors on the right side of the drive.
    private final SpeedControllerGroup m_rightMotors =
    new SpeedControllerGroup(new PWMVictorSPX(RobotMap.FRONT_RIGHT),
    new PWMVictorSPX(RobotMap.REAR_RIGHT));

    // The robot's drive
     private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The left-side drive encoder
    private final Encoder m_leftEncoder =
    new Encoder(RobotMap.FROMT_LEFT,RobotMap.REAR_LEFT,
    RobotMap.REAR_LEFT_INV);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
    new Encoder(RobotMap.FRONT_RIGHT,RobotMap.REAR_RIGHT,
    RobotMap.REAR_RIGHT_INV);
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /**
    * Creates a new DriveSubsystem.
    */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders (We're getting this through WPI's Characterization tool)
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
      }
    
      @Override
      public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
      }
    
      /**
       * Returns the currently-estimated pose of the robot.
       *
       * @return The pose.
       */
      public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }
    
      /**
       * Returns the current wheel speeds of the robot.
       *
       * @return The current wheel speeds.
       */
      public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
      }
    
      /**
       * Resets the odometry to the specified pose.
       *
       * @param pose The pose to which to set the odometry.
       */
      public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
      }
    
      /**
       * Controls the left and right sides of the drive directly with voltages.
       *
       * @param leftVolts  the commanded left output
       * @param rightVolts the commanded right output
       */
      public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
      }
    
      /**
       * Resets the drive encoders to currently read a position of 0.
       */
      public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
      }
    
      /**
       * Gets the average distance of the two encoders.
       *
       * @return the average of the two encoder readings
       */
      public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
      }
    
      /**
       * Gets the left drive encoder.
       *
       * @return the left drive encoder
       */
      public Encoder getLeftEncoder() {
        return m_leftEncoder;
      }
    
      /**
       * Gets the right drive encoder.
       *
       * @return the right drive encoder
       */
      public Encoder getRightEncoder() {
        return m_rightEncoder;
      }
    
      /**
       * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
       *
       * @param maxOutput the maximum output to which the drive will be constrained
       */
      public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
      }
    
      /**
       * Zeroes the heading of the robot.
       */
      public void zeroHeading() {
        Robot.navX.zeroYaw();
      }
    
      /**
       * Returns the heading of the robot.
       *
       * @return the robot's heading in degrees, from -180 to 180
       */
      public double getHeading() {
        //We use a NavX, which I don't think can be reversed, but if it can, replace false with NavX reversed 
        return Math.IEEEremainder(Robot.navX.getYaw(), 360) * (false ? -1.0 : 1.0);
      }
    
    
      public double getTurnRate() {
        return Robot.navX.rate;
      }
    

}