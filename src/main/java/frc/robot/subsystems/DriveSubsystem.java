// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants;

public class DriveSubsystem extends SubsystemBase 
{
  /** Creates a new DriveSubsystem. */
    public Pigeon2 m_imu = new Pigeon2(CANBusIDs.k_pigeon2ID, "CANivorous_Rex");
    DriveSubsystem m_drivetrain;
  
    private PIDController m_rotController = new PIDController(.05, 0, 0);
    
    private SwerveModuleState[] m_states = new SwerveModuleState[]
    {
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0)),
        new SwerveModuleState(0.0, new Rotation2d(0))
    }; 
  
    public SwerveModule[] modules = new SwerveModule[]
    {
        new SwerveModule(new TalonFX(CANBusIDs.k_LeftFront_DriveMotor,"CANivorous_Rex"), new TalonFX(CANBusIDs.k_LeftFront_SteeringMotor,"CANivorous_Rex"), new CANCoder(CANBusIDs.leftFrontCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftFrontOffset)), // Left Front
        new SwerveModule(new TalonFX(CANBusIDs.k_RightFront_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_RightFront_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.rightFrontCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightFrontOffset)), // Right Front
        new SwerveModule(new TalonFX(CANBusIDs.k_LeftRear_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_LeftRear_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.leftRearCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftRearOffset)), // Left Rear
        new SwerveModule(new TalonFX(CANBusIDs.k_RightRear_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_RightRear_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.rightRearCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightRearOffset)), // Right Rear
    };
  
    /*private SwerveModulePosition[] swerveModulePosition = new SwerveModulePosition[]
    {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
   */
    public DriveSubsystem(boolean calibrateGyro) 
    {
      if(calibrateGyro) 
      {
        m_imu.setYaw(0); //recalibrates gyro offset
      }
      resetIMU();
          
      for(int i = 0; i < 4; i++)
      {
         modules[i].resetDriveEncoder();
      }
  
      m_imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 20);
  
      m_rotController.enableContinuousInput(-180, 180);
      m_rotController.setTolerance(2);

      //private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveDriveModuleConstants.k_AutoKinematics, getHeading(true), swerveModulePosition);

  
    }

    public void setSwerveModuleStatesTele(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, SwerveDriveModuleConstants.k_MaxTeleSpeed);
          modules[0].setDesiredState(desiredStates[0], true);
          modules[1].setDesiredState(desiredStates[1], true);
          modules[2].setDesiredState(desiredStates[2], true);
          modules[3].setDesiredState(desiredStates[3], true);
    }

    public void teleDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
    {       
      m_states =
        SwerveDriveModuleConstants.k_AutoKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading(true))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setSwerveModuleStatesTele(m_states);
    }
      
    public void resetEncoders()
    {
      modules[0].resetDriveEncoder();
      modules[1].resetDriveEncoder();
      modules[2].resetDriveEncoder();
      modules[3].resetDriveEncoder();
    }
  
    public void resetIMU()
    {
      m_imu.setYaw(0);
    }



    public Rotation2d getHeading(boolean positive)
    { if(positive)
      {
        return Rotation2d.fromDegrees(m_imu.getYaw());
      }
      else
      {
        return Rotation2d.fromDegrees(-m_imu.getYaw());
      }
    }

    public double turnToAngle(double desiredAngle)
    {
      double output = -m_rotController.calculate((getHeading(false).getDegrees() % 360), desiredAngle);
      double maxoutput = .8;

      if (output >= maxoutput)
      {
        output = maxoutput;
      }
      else if (output < -maxoutput)
      {
        output = -maxoutput;
      }
      return output;
    }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
