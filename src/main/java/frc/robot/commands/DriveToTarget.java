// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

public class DriveToTarget extends CommandBase 
{

  private final double KpAim = -0.2;
  private final double KpDistance = -0.1;
  private final double min_aim_command = 0.05;

  private VisionTargeting visionTargeting;
  private DriveSubsystem driveSubsystem;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(DriveSubsystem driveSubsystem, VisionTargeting visionTargeting) 
  {
    this.driveSubsystem = driveSubsystem;
    this.visionTargeting = visionTargeting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    visionTargeting.setPipeline(VisionConstants.detectorPipeline);
    SmartDashboard.putBoolean("Command Test", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double steering_adjust = 0;

    if (visionTargeting.getTx() > 1.0)
    {
      steering_adjust = (KpAim * visionTargeting.getTx()) - min_aim_command;
    }
    else if (visionTargeting.getTx() < -1.0)
    {
      steering_adjust = (KpAim * visionTargeting.getTx())  + min_aim_command;
    }

    double distance_adjust = KpDistance * visionTargeting.getDistanceToTarget();

    //driveSubsystem.teleDrive(0, 0, steering_adjust, false);
    driveSubsystem.teleDrive(0, distance_adjust, 0, false);
    SmartDashboard.putNumber("Steering Adjust", steering_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    SmartDashboard.putBoolean("Command Test", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }

}
