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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import java.io.IOException;
import java.nio.file.Paths;

/**
 * Add your docs here.
 */
public class Commands {
    private final Drivebase m_robotDrive = new Drivebase();

    public Command getAutonomousCommand() throws Exception {
       
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotMap.ksVolts, RobotMap.kvVoltSecondsPerMeter, RobotMap.kaVoltSecondsSquaredPerMeter), RobotMap.kDriveKinematics, 10);

       TrajectoryConfig config = new TrajectoryConfig(3.0, 3.0)
       .setKinematics(RobotMap.kDriveKinematics)
       .addConstraint(autoVoltageConstraint);

       /* Trajectory trajectory = null;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/Straight.wpilib.json"));

        } catch(Exception e){
            System.out.println("Path not found.");
        } */ 

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(1,1.5),
                new Translation2d(2,-1.5)
            ),
            new Pose2d(3,0,Rotation2d.fromDegrees(0)),
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
            new PIDController(SmartDashboard.getNumber("P value", 2.4),SmartDashboard.getNumber("I value", 0.0), SmartDashboard.getNumber("D value", 0.0)),
            new PIDController(SmartDashboard.getNumber("P value", 2.4),SmartDashboard.getNumber("I value", 0.0),SmartDashboard.getNumber("D value", 0.0)),
            m_robotDrive::setOutputVolts,
            m_robotDrive
        );
        return ramseteCommand.andThen(() -> m_robotDrive.setOutputVolts(0, 0));
    } 
}
