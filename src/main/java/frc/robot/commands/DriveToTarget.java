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


  private final double KpDistance = -.5;


  private VisionTargeting visionTargeting;
  private DriveSubsystem driveSubsystem;
  private boolean isAprilTag;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(DriveSubsystem driveSubsystem, VisionTargeting visionTargeting, boolean isAprilTag) 
  {
    this.driveSubsystem = driveSubsystem;
    this.visionTargeting = visionTargeting;
    this.isAprilTag = isAprilTag;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (isAprilTag)
    {
      visionTargeting.setPipeline(VisionConstants.aprilTagPipeline);
    }
    else
    {
      visionTargeting.setPipeline(VisionConstants.detectorPipeline);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double distanceToTarget = visionTargeting.getDistance();
    SmartDashboard.putNumber("distance to target", distanceToTarget);
    double distance_adjust = KpDistance * distanceToTarget;

    driveSubsystem.teleDrive(distance_adjust ,0, 0, false);
    SmartDashboard.putNumber("Distance Adjust", distance_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }

}
