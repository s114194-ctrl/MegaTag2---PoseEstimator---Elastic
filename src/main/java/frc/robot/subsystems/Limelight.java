// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.Driver;
import edu.wpi.first.wpilibj.Encoder;


public class Limelight extends SubsystemBase {
  private final SwerveDriveKinematics getKinematics= new SwerveDriveKinematics(  
    new Translation2d(0.32385, 0.32385), // FL
    new Translation2d(0.32385, -0.32385), // FR
    new Translation2d(-0.32385, 0.32385), // BF
    new Translation2d(-0.32385, -0.32385) // BR
    );
  
  private final Encoder drivEncoder;
  private final Encoder drivTurn;
  private Driver driver = new Driver();
  public Rotation2d rotation2d = new Rotation2d(0);
  public SwerveModulePosition[] swerveModulePosition = Driver.getModulePositions();
  private Pose2d pose2d = new Pose2d();
  private final Field2d field2d = new Field2d();
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(getKinematics, rotation2d, swerveModulePosition, pose2d);
  public Pigeon2 pigeon2 = new Pigeon2(0);
  

  public Limelight(Driver driver){
    this.driver = driver;
    this.drivEncoder = new Encoder(1, 2);
    this.drivTurn = new Encoder(3, 4);
    SmartDashboard.putData("field2d",field2d);
    }
    public SwerveDrivePoseEstimator getswSwerveDrivePoseEstimator(){
      return swerveDrivePoseEstimator;
    }

    
      
  @Override
  public void periodic() {
   
    LimelightHelpers.SetRobotOrientation("Limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("Limelight");
    rotation2d = new Rotation2d(pigeon2.getYaw().getValue()) ;

     
    try{  
      field2d.getObject("botpose").setPose(mT2Estimate.pose);
     }
     catch(Exception e){
      System.out.println(e);
     }

    swerveDrivePoseEstimator.update(rotation2d, swerveModulePosition);

    try{
      swerveDrivePoseEstimator.addVisionMeasurement(mT2Estimate.pose , mT2Estimate.timestampSeconds);
      pose2d = swerveDrivePoseEstimator.getEstimatedPosition();
      field2d.setRobotPose(pose2d);}
      catch(Exception e){
        System.out.print(e);
      }

    } 
    
    public SwerveModulePosition getSwerveModulePositions(){
      return new SwerveModulePosition(
        drivEncoder.getDistance() , new Rotation2d(drivTurn.getDistance()));
    } 
  }
  