// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public final class Autos {
  public static Command driveForward(SwerveDrivetrain drivetrain) {
    return drivetrain.driveCommand(()->1.0, ()->0.0, ()->0.0)
      .withTimeout(Seconds.of(2.0))
      .withName("Go Forward");
  }

  public static Command driveSystemId(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
      drivetrain.dynaDriveIdCommand(Direction.kForward),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.dynaDriveIdCommand(Direction.kReverse),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.quasiDriveIdCommand(Direction.kForward),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.quasiDriveIdCommand(Direction.kReverse),
      Commands.waitTime(Seconds.of(3.0))
    );
  }

  public static Command steerSystemId(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
      drivetrain.dynaSteerIdCommand(Direction.kForward),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.dynaSteerIdCommand(Direction.kReverse),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.quasiSteerIdCommand(Direction.kForward),
      Commands.waitTime(Seconds.of(3.0)),
      drivetrain.quasiSteerIdCommand(Direction.kReverse),
      Commands.waitTime(Seconds.of(3.0))
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
