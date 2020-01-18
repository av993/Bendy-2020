/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;


import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * Add your docs here.
 */
public class Commands {
    private final Drivebase m_robotDrive = new Drivebase();

    public Command getAutonomousCommand(){
       
       TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(10.0), Units.feetToMeters(6.56));
       config.setKinematics(RobotMap.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,new Rotation2d(0)),
            List.of(
                new Translation2d(1,1),
                new Translation2d(2,-1)
            ),
            new Pose2d(3,0,new Rotation2d(0)),
            config
        );


        
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_robotDrive::getPose,
            new RamseteController(2,0.7),
            new SimpleMotorFeedforward(RobotMap.ksVolts, 
                                           RobotMap.kvVoltSecondsPerMeter, 
                                           RobotMap.kaVoltSecondsSquaredPerMeter),
            RobotMap.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(0.0,0,0),
            new PIDController(0.0,0,0),
            m_robotDrive::setOutputVolts,
            m_robotDrive
        );
        return ramseteCommand.andThen(() -> m_robotDrive.setOutputVolts(0, 0));
    } 
}
