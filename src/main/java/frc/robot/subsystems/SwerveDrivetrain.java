package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
    // Odometry positions
    private final SendableChooser<Pose2d> m_chooser = new SendableChooser<>();
    private Pose2d m_lastChoice;
    // x = forward, y = strafe
    private final SwerveModule[] m_modules = new SwerveModule[]{
        new SwerveModule(Constants.kFRSteerId, Constants.kFRDriveId, 1), // top right
        new SwerveModule(Constants.kFLSteerId, Constants.kFLDriveId, 2), // top left
        new SwerveModule(Constants.kBRSteerId, Constants.kBRDriveId, 3), // bottom right
        new SwerveModule(Constants.kBLSteerId, Constants.kBLDriveId, 4) // bottom left
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(new Translation2d[]{
        new Translation2d(Constants.kRobotDepth / 2, Constants.kRobotWidth / 2), // top right
        new Translation2d(Constants.kRobotDepth / 2, -Constants.kRobotWidth / 2), // top left
        new Translation2d(-Constants.kRobotDepth / 2, Constants.kRobotWidth / 2), // bottom right
        new Translation2d(-Constants.kRobotDepth / 2, -Constants.kRobotWidth / 2), // bottom left
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
                        .linearPosition(Meters.of(m_modules[i].getDrivePosition()))
                        .linearVelocity(MetersPerSecond.of(m_modules[i].getVelocity()))
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
                        .angularPosition(Rotations.of(m_modules[i].getSteerVelocity()))
                        .angularVelocity(RotationsPerSecond.of(m_modules[i].getSteerVelocity()))
                        .voltage(Volts.of(m_modules[i].getSteerVoltage()));
                } 
            }, this)
    );
    private final ADIS16470_IMU m_gyroscope = new ADIS16470_IMU(
        Constants.kYawAxis,
        Constants.kPitchAxis,
        Constants.kRollAxis);
    // Simulation variables
    private ADIS16470_IMUSim m_gyroSim;

    public SwerveDrivetrain() {
        addDashboardEntries();
        m_odometry.resetPose(m_chooser.getSelected());

        this.setDefaultCommand(this.run(
            () -> {
                for (final SwerveModule module : m_modules) module.goToState(0, 0);
            }
        ).withName("Default Swerve Command"));

        if (RobotBase.isSimulation()) {
            m_gyroSim = new ADIS16470_IMUSim(m_gyroscope);
        }
    }

    public void addDashboardEntries() {
        SmartDashboard.putData(m_field);
        SmartDashboard.putData("Gyroscope", m_gyroscope);
        m_chooser.setDefaultOption("Blue Start", Constants.kBlueStart);
        m_chooser.addOption("Red Start", Constants.kRedStart);
        m_chooser.addOption("Zero", Constants.kZero);
        m_lastChoice = Constants.kBlueStart;
        SmartDashboard.putData("Starting Pose", m_chooser);
        for (final SwerveModule module : m_modules) {
            SmartDashboard.putData(String.format("Swerve Module %d", module.swerveModNum), module);  
        }
    }

    @Override
    public void periodic() {
        for (final SwerveModule module : m_modules) module.updatePosition();
        m_odometry.update(getGyroscope(), new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
        if (m_chooser.getSelected() != m_lastChoice) {
            m_odometry.resetPose(m_chooser.getSelected());
            m_lastChoice = m_chooser.getSelected();
        }
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        for (final SwerveModule module : m_modules) module.simulationPeriodic();
        SwerveModuleState[] states = new SwerveModuleState[]{
            new SwerveModuleState(
                m_modules[0].getVelocity(),
                Rotation2d.fromRadians(m_modules[0].getSteerAngle())),
            new SwerveModuleState(
                m_modules[1].getVelocity(),
                Rotation2d.fromRadians(m_modules[1].getSteerAngle())),
            new SwerveModuleState(
                m_modules[2].getVelocity(),
                Rotation2d.fromRadians(m_modules[2].getSteerAngle())),
            new SwerveModuleState(
                m_modules[3].getVelocity(),
                Rotation2d.fromRadians(m_modules[3].getSteerAngle()))
        };
        m_gyroSim.setGyroAngleY(Rotation2d.fromDegrees(m_gyroscope.getAngle()).plus(Rotation2d.fromRadians(m_kinematics.toChassisSpeeds(states).omegaRadiansPerSecond).times(0.02)).getDegrees());
        //m_gyroscope.setGyroAngle(Constants.kYawAxis, m_heading.getDegrees());
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
                    if (Constants.kCosineScale) {
                        states[i].cosineScale(m_modules[i].getPosition().angle);
                    }
                    m_modules[i].goToState(states[i].speedMetersPerSecond, states[i].angle.getRotations());
                }
            }
        );
    }

    // get it to work with elastic's swerve drive widget
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", m_modules[1]::getSteerAngle, null);
        builder.addDoubleProperty("Front Left Velocity", m_modules[1]::getVelocity, null);
        builder.addDoubleProperty("Front Right Angle", m_modules[0]::getSteerAngle, null);
        builder.addDoubleProperty("Front Right Velocity", m_modules[0]::getVelocity, null);
        builder.addDoubleProperty("Back Left Angle", m_modules[3]::getSteerAngle, null);
        builder.addDoubleProperty("Back Left Velocity", m_modules[3]::getVelocity, null);
        builder.addDoubleProperty("Back Right Angle", m_modules[2]::getSteerAngle, null);
        builder.addDoubleProperty("Back Right Velocity", m_modules[2]::getVelocity, null);
        builder.addDoubleProperty("Robot Angle", () -> this.getGyroscope().getRadians(), null);
    }

    /**
     * @return The value of the gyroscope.
     */
    private Rotation2d getGyroscope() {
        return Rotation2d.fromDegrees(m_gyroscope.getAngle());
    }

    public double getCurrentDraw() {
        return m_modules[0].getCurrentDraw() + m_modules[1].getCurrentDraw() + m_modules[2].getCurrentDraw() + m_modules[3].getCurrentDraw();
    }
}
