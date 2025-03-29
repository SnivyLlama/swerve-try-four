package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
    // TODO: double check values
    private final double ROBOT_WIDTH = Units.inchesToMeters(22);
    private final double ROBOT_DEPTH = Units.inchesToMeters(22);
    // x = forward, y = strafe
    private final SwerveModule[] m_modules = new SwerveModule[]{
        new SwerveModule(0, 1),
        new SwerveModule(2, 3),
        new SwerveModule(4, 5),
        new SwerveModule(6, 7)
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(new Translation2d[]{
        new Translation2d(ROBOT_DEPTH, ROBOT_WIDTH), // top right
        new Translation2d(ROBOT_DEPTH, -ROBOT_WIDTH), // top left
        new Translation2d(-ROBOT_DEPTH, ROBOT_WIDTH), // bottom right
        new Translation2d(-ROBOT_DEPTH, -ROBOT_WIDTH), // bottom left
    });
    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(m_kinematics, Rotation2d.kZero, new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
    private final SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
        null, null
    );

    public SwerveDrivetrain() {

    }

    @Override
    public void periodic() {
        for (SwerveModule module : m_modules) module.updatePosition();
        m_odometry.update(null, new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
    }

    public Command quasiDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command dynaDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command driveCommand(DoubleSupplier fwd, DoubleSupplier strafe, DoubleSupplier turn) {
        return this.run(
            () -> {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(fwd.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble()));
                for (int i = 0; i < states.length; ++i) {
                    m_modules[i].goToState(states[i].speedMetersPerSecond, states[i].angle.getDegrees());
                }
            }
        );
    }
}
