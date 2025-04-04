// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public final class Autos {
  public static Command driveForward(SwerveDrivetrain drivetrain, double timeout) {
    return drivetrain.driveCommand(()->1.0, ()->0.0, ()->DegreesPerSecond.of(90))
      .withTimeout(Seconds.of(timeout))
      .withName(String.format("Go Forward (%.1fs)", timeout));
  }

  public static Command spinInPlace(SwerveDrivetrain drivetrain, double timeout) {
    return drivetrain.driveCommand(()->0.0, ()->0.0, ()->DegreesPerSecond.of(90))
      .withTimeout(Seconds.of(timeout))
      .withName(String.format("Spinning In Place (%.1fs)", timeout));
  }

  public static Command driveThenTurn(SwerveDrivetrain drivetrain) {
    return drivetrain.driveCommand(()->1.0, ()->0.0, ()->DegreesPerSecond.zero())
      .withTimeout(Seconds.of(5.0))
      .andThen(
        drivetrain.driveCommand(()->0.0, ()->0.0, ()->DegreesPerSecond.of(90))
          .withTimeout(Seconds.of(5.0)))
      .withName("Drive Then Spin");
  }

  public static Command driveThenRight(SwerveDrivetrain drivetrain) {
    return drivetrain.driveCommand(() -> 1.0, () -> 0.0, () -> RPM.zero())
      .withTimeout(Seconds.of(5.0))
      .andThen(
        drivetrain.driveHeadingCommand(() -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(-90))
          .withTimeout(Seconds.of(10.0)))
      .withName("Drive Then Right");
  }

  public static Command goToBlueStart(SwerveDrivetrain drivetrain) {
    return drivetrain.goToPosCommand(Constants.kBlueStart)
      .withName("Go To Blue Start");
  }

  /*
   * We don't need to go to all of them.
   */
  public static Command goToAllAprilTags(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
      drivetrain.goToAprilTag(1),
      drivetrain.goToAprilTag(2),
      drivetrain.goToAprilTag(3),
      drivetrain.goToAprilTag(4),
      drivetrain.goToAprilTag(5),
      drivetrain.goToAprilTag(6),
      drivetrain.goToAprilTag(7),
      drivetrain.goToAprilTag(8),
      drivetrain.goToAprilTag(9),
      drivetrain.goToAprilTag(10),
      drivetrain.goToAprilTag(11),
      drivetrain.goToAprilTag(12),
      drivetrain.goToAprilTag(13),
      drivetrain.goToAprilTag(14),
      drivetrain.goToAprilTag(15),
      drivetrain.goToAprilTag(16),
      drivetrain.goToAprilTag(17),
      drivetrain.goToAprilTag(18),
      drivetrain.goToAprilTag(19),
      drivetrain.goToAprilTag(20),
      drivetrain.goToAprilTag(21),
      drivetrain.goToAprilTag(22)
    );
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
    ).withName("Drive System Id");
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
    ).withName("Steer System Id");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
