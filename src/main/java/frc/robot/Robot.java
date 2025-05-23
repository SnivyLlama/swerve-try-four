// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;

  private final RobotContainer m_robotContainer;
  private final Timer m_autoTimer = new Timer();
  private final Timer m_teleopTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // TODO: figure out why this does not work outside of FMS
    if (DriverStation.isFMSAttached())
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    else if (DriverStation.isAutonomousEnabled())
      SmartDashboard.putNumber("Match Time", m_autoTimer.get());
    else if (DriverStation.isTeleopEnabled())
      SmartDashboard.putNumber("Match Time", m_teleopTimer.get());
    else
      SmartDashboard.putNumber("Match Time", 0.0);
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Joystick", DriverStation.isJoystickConnected(0));
    SmartDashboard.putBoolean("FMS", DriverStation.isFMSAttached());
    SmartDashboard.putBoolean("DS", DriverStation.isDSAttached());
    CommandScheduler.getInstance().run();
    
    if (RobotBase.isSimulation()) {
      double vbus = BatterySim.calculateDefaultBatteryLoadedVoltage(m_robotContainer.getCurrentDraw().in(Volts));
      RoboRioSim.setVInVoltage(vbus);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_autoTimer.stop();
    m_teleopTimer.stop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoCommand = m_robotContainer.getAutonomousCommand();
    m_autoTimer.reset();
    m_autoTimer.start();

    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_teleopTimer.reset();
    m_teleopTimer.start();
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
