package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        new SwerveModule(0, 1, 1),
        new SwerveModule(2, 3, 2),
        new SwerveModule(4, 5, 3),
        new SwerveModule(6, 7, 4)
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(new Translation2d[]{
        new Translation2d(ROBOT_DEPTH / 2, ROBOT_WIDTH / 2), // top right
        new Translation2d(ROBOT_DEPTH / 2, -ROBOT_WIDTH / 2), // top left
        new Translation2d(-ROBOT_DEPTH / 2, ROBOT_WIDTH / 2), // bottom right
        new Translation2d(-ROBOT_DEPTH / 2, -ROBOT_WIDTH / 2), // bottom left
    });
    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(m_kinematics, Rotation2d.kZero, new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
    private final Field2d m_field = new Field2d();
    private final SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                for (SwerveModule module : m_modules) module.setDrive(voltage);
            },
            log -> {
                for (int i = 0; i < m_modules.length; ++i) {
                    log.motor(String.format("swerve-drive-%d", i))
                        .linearPosition(Meters.of(m_modules[i].getDriveEncoder().getPosition()))
                        .linearVelocity(MetersPerSecond.of(m_modules[i].getDriveEncoder().getVelocity()))
                        .voltage(Volts.of(m_modules[i].getDriveVoltage()));
                }
            }, this)
    );
    private final SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                for (SwerveModule module : m_modules) module.setSteer(voltage);
            },
            log -> {
                for (int i = 0; i < m_modules.length; ++i) {
                    log.motor(String.format("swerve-drive-%d", i))
                        .angularPosition(Rotations.of(m_modules[i].getSteerEncoder().getPosition()))
                        .angularVelocity(RotationsPerSecond.of(m_modules[i].getSteerEncoder().getVelocity()))
                        .voltage(Volts.of(m_modules[i].getSteerVoltage()));
                } 
            }, this)
    );
    private final ADIS16470_IMU m_gyroscope = new ADIS16470_IMU();
    // Simulation variable
    private Rotation2d m_heading = Rotation2d.kZero;

    public SwerveDrivetrain() {
        SmartDashboard.putData(m_field);
        for (final SwerveModule module : m_modules) {
            SmartDashboard.putData(String.format("Swerve Module %d", module.swerveModNum), module);  
        }
        this.setDefaultCommand(this.run(
            () -> {
                for (final SwerveModule module : m_modules) module.goToState(0, 0);
            }
        ));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyroscope", getGyroscope().getDegrees());
        for (final SwerveModule module : m_modules) module.updatePosition();
        m_odometry.update(getGyroscope(), new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        for (final SwerveModule module : m_modules) module.simulationPeriodic();
        SwerveModuleState[] states = new SwerveModuleState[]{
            new SwerveModuleState(
                m_modules[0].getDriveEncoder().getVelocity(),
                Rotation2d.fromRotations(m_modules[0].getSteerEncoder().getPosition())),
            new SwerveModuleState(
                m_modules[1].getDriveEncoder().getVelocity(),
                Rotation2d.fromRotations(m_modules[1].getSteerEncoder().getPosition())),
            new SwerveModuleState(
                m_modules[2].getDriveEncoder().getVelocity(),
                Rotation2d.fromRotations(m_modules[2].getSteerEncoder().getPosition())),
            new SwerveModuleState(
                m_modules[3].getDriveEncoder().getVelocity(),
                Rotation2d.fromRotations(m_modules[3].getSteerEncoder().getPosition()))
        };
        m_heading = m_heading.plus(Rotation2d.fromRadians(m_kinematics.toChassisSpeeds(states).omegaRadiansPerSecond).times(0.02));
    }

    public Command quasiDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command dynaDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command quasiSteerIdCommand(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command dynaSteerIdCommand(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    /**
     * @param fwd Forward rate in m/s.
     * @param strafe Strafing rate in m/s.
     * @param turn Turning rate in deg/sec.
     * @return
     */
    public Command driveCommand(DoubleSupplier fwd, DoubleSupplier strafe, DoubleSupplier turn) {
        return this.run(
            () -> {
                final SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(fwd.getAsDouble(), strafe.getAsDouble(), Units.degreesToRadians(turn.getAsDouble())));
                for (int i = 0; i < states.length; ++i) {
                    m_modules[i].goToState(states[i].speedMetersPerSecond, states[i].angle.getDegrees());
                }
            }
        );
    }

    // get it to work with elastic's swerve drive widget
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", m_modules[1].getSteerEncoder()::getPosition, null);
        builder.addDoubleProperty("Front Left Velocity", m_modules[1].getDriveEncoder()::getVelocity, null);
        builder.addDoubleProperty("Front Right Angle", m_modules[0].getSteerEncoder()::getPosition, null);
        builder.addDoubleProperty("Front Right Velocity", m_modules[0].getDriveEncoder()::getVelocity, null);
        builder.addDoubleProperty("Back Left Angle", m_modules[3].getSteerEncoder()::getPosition, null);
        builder.addDoubleProperty("Back Left Velocity", m_modules[3].getDriveEncoder()::getVelocity, null);
        builder.addDoubleProperty("Back Right Angle", m_modules[2].getSteerEncoder()::getPosition, null);
        builder.addDoubleProperty("Back Right Velocity", m_modules[2].getDriveEncoder()::getVelocity, null);
        builder.addDoubleProperty("Robot Angle", () -> this.getGyroscope().getDegrees(), null);
    }

    private Rotation2d getGyroscope() {
        if (RobotBase.isSimulation()) return m_heading;
        return Rotation2d.fromDegrees(m_gyroscope.getAngle(m_gyroscope.getRollAxis()));
    }
}
