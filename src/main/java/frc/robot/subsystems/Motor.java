// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase 
{
  private CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);
  /** Creates a new Motor. */
  public Motor() 
  {
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(10);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);
    m_motor.setOpenLoopRampRate(2);
    m_motor.setSmartCurrentLimit(80);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void forceMotorExtend()
  {
      m_motor.set(.1);
  }

  public void forceMotorRetract()
  {
    m_motor.set(-0.1);
  }

  public void forceMotorStop()
  {
    m_motor.set(0);
  }
}
