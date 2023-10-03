// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase 
{
  private CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);
  /** Creates a new Motor. */
  //private double m_counts = 0;
  private RelativeEncoder m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private double m_setpoint = 5 * m_encoder.getCountsPerRevolution();
  private PIDController m_motorPID = new PIDController( .0001, 0, 0 );
  
  //m_motorPID.setFeedbackDevice(m_encoder);
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
  public void periodic() 
  {
    // This method will be called once per scheduler run
    goToPosition(m_setpoint);
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

  public void goToPosition(double goalPosition)
  {
    double pidVal = m_motorPID.calculate(getRelativeEncoder(), goalPosition);
    m_motor.setVoltage(pidVal);
  }

  public double getRelativeEncoder()
  {
    return m_encoder.getPosition() * m_encoder.getCountsPerRevolution();
  }
  
}
