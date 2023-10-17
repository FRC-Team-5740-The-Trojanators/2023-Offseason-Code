// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Motor;

public class MotorPIDCommand extends CommandBase 
{
  private double revolutions;
  private Motor motor;
  /** Creates a new MotorPIDCommand. */
  public MotorPIDCommand(double revolutions, Motor motor) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.revolutions = revolutions;
    this.motor = motor; 
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      motor.zeroEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    motor.goToPosition(revolutions);
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
    return motor.atPosition();
  }
}
