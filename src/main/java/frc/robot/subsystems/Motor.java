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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Motor extends SubsystemBase 
{
  private CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);
  /** Creates a new Motor. */
  //private double m_counts = 0;
  private RelativeEncoder m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private PIDController m_motorPID = new PIDController( 0.73, 0, 0 );
  private PIDController m_motorPIDvelocity = new PIDController(1, 0, 0 );
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, .0026, 0);

  //m_motorPID.setFeedbackDevice(m_encoder);
  public Motor() 
  {
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(10);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);
    m_motor.setOpenLoopRampRate(2);
    m_motor.setSmartCurrentLimit(80);
    m_motorPID.setTolerance(.02);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("setpoint", Constants.revs);
    SmartDashboard.putNumber("encodervalue", m_encoder.getPosition());
    SmartDashboard.putData("PID", m_motorPID);
    SmartDashboard.putBoolean("At Setpoint", m_motorPID.atSetpoint());
    SmartDashboard.putNumber("PID error", m_motorPID.getPositionError());
    SmartDashboard.putNumber("Velocity", getVelocity());
    SmartDashboard.putData("PIDF", m_motorPIDvelocity);
    SmartDashboard.putBoolean("Velocity Setpoint", m_motorPIDvelocity.atSetpoint());
    SmartDashboard.putNumber("PIDF error", m_motorPIDvelocity.getPositionError());
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
    double pidVal = m_motorPID.calculate(m_encoder.getPosition(), goalPosition);
    m_motor.setVoltage(pidVal);
  }
  public void setVelocity(double velocity)
  {
    double pidVal = m_motorPIDvelocity.calculate(getVelocity(), velocity);
    double FFVal = feedforward.calculate(velocity, 20);
    m_motor.setVoltage(pidVal+FFVal);
    SmartDashboard.putNumber("PIDval", pidVal);
    SmartDashboard.putNumber("FFval", FFVal);
  }
  public void zeroEncoder()
  {
    m_encoder.setPosition(0);
  }

  public boolean atPosition()
  {
      return m_motorPID.atSetpoint();
  }
  public boolean atVelocity()
  {
      return m_motorPIDvelocity.atSetpoint();
  }
  public double getVelocity()
  {
      return m_encoder.getVelocity();
  }
}
