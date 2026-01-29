// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.security.PrivateKey;

import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

public class Limelight extends SubsystemBase {
  public Limelight(){
      SmartDashboard.putData("field2d",field2d);
    }
  private final SwerveDriveKinematics getKinematics= new SwerveDriveKinematics(  
    new Translation2d(0.32385, 0.32385), // 前左 (假設值, 請替換為實際值)
    new Translation2d(0.32385, -0.32385), // 前右
    new Translation2d(-0.32385, 0.32385), // 後左
    new Translation2d(-0.32385, -0.32385) // 後右
    );
  private  Rotation2d rotation2d = new Rotation2d(0);
  private final SwerveModulePosition[] swerveModulePosition = new SwerveModulePosition[]{
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private Pose2d pose2d = new Pose2d();
  private final Field2d field2d = new Field2d();
  private final PoseEstimate botpose = new PoseEstimate();
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(getKinematics, rotation2d, swerveModulePosition, pose2d);
  public SwerveDrivePoseEstimator getswSwerveDrivePoseEstimator(){
    return swerveDrivePoseEstimator;
  
  }

  @Override
  public void periodic() {
   
    LimelightHelpers.SetRobotOrientation("limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("Limelight");
     
    try{ 
      mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");  
      field2d.getObject("botpose").setPose(mT2Estimate.pose);
     }
     catch(Exception e){
      System.out.println(e);
     }

    swerveDrivePoseEstimator.update(rotation2d,swerveModulePosition);
    LimelightHelpers.SetRobotOrientation(
      "limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    try{
      swerveDrivePoseEstimator.addVisionMeasurement(botpose.pose , botpose.timestampSeconds);
      pose2d = swerveDrivePoseEstimator.getEstimatedPosition();
      field2d.setRobotPose(pose2d);}
      catch(Exception e){
        System.out.print(e);
      }

    } 
  }
  