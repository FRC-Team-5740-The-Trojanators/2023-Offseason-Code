// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;



public class DriveToTarget extends CommandBase 
{

  private final double KpAim = -0.1;
  private final double KpDistance = -0.1;
  private final double min_aim_command = 0.05;

  private VisionTargeting visionTargeting;
  private DriveSubsystem driveSubsystem;

  /** Creates a new DriveToTarget. */
  public DriveToTarget() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    visionTargeting.setPipeline(VisionConstants.detectorPipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double steering_adjust = 0;

    double left_command = 0;
    double right_command = 0;

    if (visionTargeting.getTx() > 1.0)
    {
      steering_adjust = -(KpAim * visionTargeting.getTx()) - min_aim_command;
    }
    else if (visionTargeting.getTx() < -1.0)
    {
      steering_adjust = (KpAim * visionTargeting.getTx())  + min_aim_command;
    }

    double distance_adjust = KpDistance * visionTargeting.getDistanceToTarget();

    left_command += (steering_adjust + distance_adjust); //TODO change so we only steering adjust
    right_command -= (steering_adjust + distance_adjust);

    driveSubsystem.teleDrive(0, distance_adjust, steering_adjust, false);
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
