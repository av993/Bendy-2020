/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class ActionTrajectory implements Action {

    public void run() {


        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(14.0), Units.feetToMeters(6.00));

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
             // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),        
        List.of(new Translation2d(1,0), new Translation2d(3,0)),
        new Pose2d(5, 0, new Rotation2d(0)),
        config);





    }

   
    public boolean isFinished() {
        return false;
    }


}
