// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Motor;

public class MotorCommand extends CommandBase 
{
  private Motor m_motor;
  private String m_direction;
  /** Creates a new MotorCommand. */
  public MotorCommand(Motor motor, String direction) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_motor = motor;
    m_direction = direction;
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (m_direction == "forward")
    {
      m_motor.forceMotorExtend();
    }
    if (m_direction == "backward")
    {
      m_motor.forceMotorRetract();
    }
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_motor.forceMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
