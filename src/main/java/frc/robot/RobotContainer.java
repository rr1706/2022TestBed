// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Command m_autoCommand = new WaitCommand(15.0);
  private final Shooter m_shooter = new Shooter();
  private final RunShooter m_runShooter = new RunShooter(m_shooter);
  private final Intake m_intake = new Intake();
  private final RunIntake m_runIntake = new RunIntake(m_intake);
  private final ZeroHood m_zero = new ZeroHood(m_shooter);
  private final XboxController m_controller1 = new XboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller1, Button.kA.value).whenPressed(m_runShooter);
    new JoystickButton(m_controller1, Button.kB.value).whenPressed(()->m_runShooter.cancel());

    new JoystickButton(m_controller1, Button.kX.value).whenPressed(m_runIntake);
    new JoystickButton(m_controller1, Button.kY.value).whenPressed(()->m_runIntake.cancel());

    new POVButton(m_controller1, 0).whenPressed(m_zero);

    //new JoystickAnalogButton(m_controller1, false).whenPressed(m_runIntake).whenReleased(()->m_runIntake.cancel());

    //new JoystickAnalogButton(m_controller1, false).whenPressed(m_runIntake).whenReleased(()->m_runIntake.cancel());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
