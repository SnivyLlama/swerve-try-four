// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final CommandJoystick m_controller = new CommandJoystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Swerve Drive", m_drivetrain);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    m_chooser.addOption("Go Forward", Autos.driveForward(m_drivetrain, 5.0));
    m_chooser.addOption("Go Forward Longer", Autos.driveForward(m_drivetrain, 50.0));
    m_chooser.addOption("Spin In Place", Autos.spinInPlace(m_drivetrain, 50.0));
    m_chooser.setDefaultOption("Go Forward And Spin", Autos.driveThenTurn(m_drivetrain));
    m_chooser.addOption("Drive System Id", Autos.driveSystemId(m_drivetrain));
    m_chooser.addOption("Steer System Id", Autos.steerSystemId(m_drivetrain));
    SmartDashboard.putData("Autonomous Chooser", m_chooser);
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
  private void configureBindings() {
    m_controller.setXChannel(Constants.kAxisX);
    m_controller.setYChannel(Constants.kAxisY);
    m_controller.setZChannel(Constants.kAxisZ);
    m_controller.axisMagnitudeGreaterThan(m_controller.getXChannel(), 0.01)
      .or(m_controller.axisGreaterThan(m_controller.getYChannel(), 0.01)
      .or(m_controller.axisMagnitudeGreaterThan(m_controller.getZChannel(), 0.01)))
      .whileTrue(m_drivetrain.driveCommand(
        m_controller::getX,
        m_controller::getY,
        m_controller::getZ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
