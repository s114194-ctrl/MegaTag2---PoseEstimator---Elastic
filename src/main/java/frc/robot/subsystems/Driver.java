package frc.robot.subsystems;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Driver extends SubsystemBase{
    private Pigeon2 Gyro = new Pigeon2(0);
        
    public Driver(){
        Gyro = new Pigeon2(0);
        Gyro.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        return Gyro.getRotation2d(); 
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    }

}