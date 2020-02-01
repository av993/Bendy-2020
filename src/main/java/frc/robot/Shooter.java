/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
public class Shooter {

    WPI_TalonSRX shooter;
    WPI_TalonSRX shooterSlave;
    WPI_TalonSRX agitator;

    double setpoint;
    
    public Shooter() {
        //shooterSlave = new WPI_TalonSRX(10);
        //shooterSlave.setInverted(false);
        
        /*shooter.setInverted(false);
        shooter.selectProfileSlot(0, 0);
        shooter.config_kP(0,0.3,1000);
        shooter.config_kI(0,0.00003,1000);
        shooter.config_kD(0,1,1000);
        shooter.config_kF(0,0.02475,1000);
        shooter.config_IntegralZone(0, 100, 1000);
    
        shooterSlave = new WPI_TalonSRX(10);
        shooterSlave.setInverted(false);        
        shooterSlave.set(ControlMode.Follower, 8);
        shooterSlave.follow(shooter);
       
        agitator = new WPI_TalonSRX(6);
        agitator.setInverted(false);
        
        setpoint = 0;
        setSetpoint(0);*/



    }


    public void control() {
        /*
        if (OI.xBtnY){
            shooterSlave.set(ControlMode.PercentOutput, 0);

        }
        else if (OI.xBtnX){
            shooterSlave.set(ControlMode.PercentOutput, 0.5);
        } */
        
        
        /*if (OI.xBtnY){
            setpoint = 0;
        }
        else if (OI.xBtnX){
            setpoint = 20250;
        } 
        
        if (OI.xBtnA){
            agitator.set(ControlMode.PercentOutput, 0.5);
        }

        else if (OI.xBtnB){
            agitator.set(ControlMode.PercentOutput, 0);
        }
        
        


        setSetpoint(setpoint);*/
    }

    public void setSetpoint(double setpoint){
        //shooter.set(ControlMode.Velocity, setpoint);
    }

    








}
