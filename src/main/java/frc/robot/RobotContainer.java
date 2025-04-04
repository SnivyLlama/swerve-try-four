// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final CommandJoystick m_joystick = new CommandJoystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Swerve Drive", m_drivetrain);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    m_autoChooser.addOption("Go Forward", Autos.driveForward(m_drivetrain, 5.0));
    m_autoChooser.addOption("Go Forward Longer", Autos.driveForward(m_drivetrain, 50.0));
    m_autoChooser.addOption("Spin In Place", Autos.spinInPlace(m_drivetrain, 50.0));
    m_autoChooser.setDefaultOption("Go Forward And Spin", Autos.driveThenTurn(m_drivetrain));
    m_autoChooser.addOption("Go To Blue Start", Autos.goToBlueStart(m_drivetrain));
    m_autoChooser.addOption("Go Forward And Go Right", Autos.driveThenRight(m_drivetrain));
    m_autoChooser.addOption("Go To All April Tags", Autos.goToAllAprilTags(m_drivetrain));
    m_autoChooser.addOption("Drive System Id", Autos.driveSystemId(m_drivetrain));
    m_autoChooser.addOption("Steer System Id", Autos.steerSystemId(m_drivetrain));
    SmartDashboard.putData("Autonomous Chooser", m_autoChooser);
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  /**
   * Configures joystick bindings to commands.
   */
  private void configureBindings() {
    m_joystick.setXChannel(Constants.kAxisX);
    m_joystick.setYChannel(Constants.kAxisY);
    m_joystick.setZChannel(Constants.kAxisZ);
    if (Constants.kUseHeading) {
      m_joystick.axisMagnitudeGreaterThan(m_joystick.getXChannel(), 0.01)
        .or(m_joystick.axisMagnitudeGreaterThan(m_joystick.getYChannel(), 0.01))
        .or(m_joystick.axisMagnitudeGreaterThan(m_joystick.getZChannel(), 0.01))
        .or(m_joystick.axisMagnitudeGreaterThan(Constants.kAxisA, 0.01))
        .whileTrue(
          m_drivetrain.driveHeadingCommand(
          () -> Math.abs(m_joystick.getX()) < 1.0e-3 ? 0.0 : m_joystick.getX() * Constants.kMaxVelocity,
          () -> Math.abs(m_joystick.getY()) < 1.0e-3 ? 0.0 : m_joystick.getY() * Constants.kMaxVelocity,
          () ->
            Rotation2d.fromRadians(Math.atan2(m_joystick.getRawAxis(Constants.kAxisA), -m_joystick.getZ()))
            // TODO: CHECK FOR ALLIANCE ERROR
              .plus(!DriverStation.isFMSAttached() ? Constants.kElasticOffset : DriverStation.getAlliance().get().equals(Alliance.Blue) ? 
                Constants.kBlueOffset : Constants.kRedOffset))
          .withName("Joystick Controlling Robot"));
    } else {
      m_joystick.axisMagnitudeGreaterThan(m_joystick.getXChannel(), 0.01)
        .or(m_joystick.axisMagnitudeGreaterThan(m_joystick.getYChannel(), 0.01)
        .or(m_joystick.axisMagnitudeGreaterThan(m_joystick.getZChannel(), 0.01)))
        .whileTrue(
          m_drivetrain.driveCommand(
            () -> Math.abs(m_joystick.getX()) < 1.0e-3 ? 0.0 : m_joystick.getX() * Constants.kMaxVelocity,
            () -> Math.abs(m_joystick.getY()) < 1.0e-3 ? 0.0 : m_joystick.getY() * Constants.kMaxVelocity,
            () -> DegreesPerSecond.of(120 * m_joystick.getZ()))
          .withName("Joystick Controlling Robot"));
    }
  }

  /**
   * @return The command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public Voltage getCurrentDraw() {
    return m_drivetrain.getCurrentDraw();
  }
}
