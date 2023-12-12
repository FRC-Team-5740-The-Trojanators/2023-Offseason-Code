// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.MotorCommand;
import frc.robot.commands.MotorPIDCommand;
import frc.robot.commands.MotorPIDFCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.VisionTargeting;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  public final VisionTargeting m_VisionTargeting = new VisionTargeting();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(true);
  public final Motor m_motor = new Motor();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(HIDConstants.k_DriverControllerPort);

    private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
   
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.resetIMU();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    JoystickButton forwardMotor = new JoystickButton(m_driverController , HIDConstants.kA);
    JoystickButton backwardMotor = new JoystickButton(m_driverController , HIDConstants.kB);
    JoystickButton runPIDMotor = new JoystickButton(m_driverController , HIDConstants.kX);
    //JoystickButton runPIDMotorBackwards = new JoystickButton(m_driverController , HIDConstants.kBack);
    JoystickButton runPIDFMotor = new JoystickButton(m_driverController, HIDConstants.kRB);
    JoystickButton runPIDFMotorBackwards = new JoystickButton(m_driverController, HIDConstants.kLB);
    JoystickButton rotateTargetCommandButton = new JoystickButton(m_driverController, HIDConstants.kStart);
    JoystickButton driveTargetCommandButton = new JoystickButton(m_driverController, HIDConstants.kBack);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    forwardMotor.whileTrue(new MotorCommand(m_motor, "forward"));
    backwardMotor.whileTrue(new MotorCommand(m_motor, "backward"));
    runPIDMotor.onTrue(new MotorPIDCommand(Constants.revs, m_motor));
    //runPIDMotorBackwards.onTrue(new MotorPIDCommand((Constants.revs) * -1, m_motor));
    runPIDFMotor.whileTrue(new MotorPIDFCommand(2000, m_motor));
    runPIDFMotorBackwards.whileTrue(new MotorPIDFCommand(-2000, m_motor));
    driveTargetCommandButton.whileTrue(new DriveToTarget(m_driveSubsystem, m_VisionTargeting, true));
    rotateTargetCommandButton.whileTrue(new RotateToTarget(m_driveSubsystem, m_VisionTargeting, true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An example command will be run in autonomous
    return null;
  }
}
