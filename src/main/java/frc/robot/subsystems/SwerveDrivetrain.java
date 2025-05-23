package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
    // All setup
    private final SwerveModule[] m_modules = new SwerveModule[]{
        new SwerveModule(Constants.kFRSteerId, Constants.kFRDriveId, 1), // top right
        new SwerveModule(Constants.kFLSteerId, Constants.kFLDriveId, 2), // top left
        new SwerveModule(Constants.kBRSteerId, Constants.kBRDriveId, 3), // bottom right
        new SwerveModule(Constants.kBLSteerId, Constants.kBLDriveId, 4) // bottom left
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(new Translation2d[]{
        new Translation2d(Constants.kRobotTrackDepth / 2, Constants.kRobotTrackWidth / 2), // top right
        new Translation2d(Constants.kRobotTrackDepth / 2, -Constants.kRobotTrackWidth / 2), // top left
        new Translation2d(-Constants.kRobotTrackDepth / 2, Constants.kRobotTrackWidth / 2), // bottom right
        new Translation2d(-Constants.kRobotTrackDepth / 2, -Constants.kRobotTrackWidth / 2), // bottom left
    });
    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(m_kinematics, Rotation2d.kZero, new SwerveModulePosition[]{
            m_modules[0].getPosition(),
            m_modules[1].getPosition(),
            m_modules[2].getPosition(),
            m_modules[3].getPosition()
        });
    private final AprilTagFieldLayout m_aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private final Field2d m_field = new Field2d();
    
    // Trajectory variables
    private final HolonomicDriveController m_controller = new HolonomicDriveController(
        new PIDController(0.01, 0, 0), new PIDController(0.01, 0, 0), 
        new ProfiledPIDController(
            Constants.kPGyro, 0.0, 0.0,
            new Constraints(Constants.kMaxAngularVelocity, Constants.kMaxAngularAcceleration)));
    private Trajectory m_currentTrajectory;
    private Timer m_trajectoryTimer = new Timer();

    // SYSTEM IDENTIFICATION ROUTINES
    private final SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                for (SwerveModule module : m_modules) module.setDrive(voltage);
            },
            log -> {
                for (int i = 0; i < m_modules.length; ++i) {
                    log.motor(String.format("swerve-drive-%d", i))
                        .linearPosition(m_modules[i].getDrivePosition())
                        .linearVelocity(MetersPerSecond.of(m_modules[i].getVelocity()))
                        .voltage(m_modules[i].getDriveVoltage());
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
                        .angularVelocity(RPM.of(m_modules[i].getSteerVelocity()))
                        .voltage(m_modules[i].getSteerVoltage());
                } 
            }, this)
    );

    // Gyroscope setup
    private final ADIS16470_IMU m_gyroscope = new ADIS16470_IMU(
        Constants.kYawAxis,
        Constants.kPitchAxis,
        Constants.kRollAxis);
    private final PIDController m_gyroPID = new PIDController(
        Constants.kPGyro,
        Constants.kIGyro,
        Constants.kDGyro);
    // Simulation variables
    private ADIS16470_IMUSim m_gyroSim;

    public SwerveDrivetrain() {
        addDashboardEntries();
        m_odometry.resetPose(m_chooser.getSelected());
        if (RobotBase.isSimulation()) {
            m_gyroSim = new ADIS16470_IMUSim(m_gyroscope);
            m_gyroSim.setGyroAngleZ(m_odometry.getPoseMeters().getRotation().getDegrees());
        } else
            m_gyroscope.setGyroAngleZ(m_odometry.getPoseMeters().getRotation().getDegrees());

        this.setDefaultCommand(this.run(
            () -> {
                for (final SwerveModule module : m_modules) module.goToState(MetersPerSecond.zero(), Rotation2d.kZero);
            }
        ).withName("Default Swerve Command"));
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
        for (int i = 1; i <= 22; ++i)
            SmartDashboard.putData(String.format("Go To AprilTag %d", i), this.goToAprilTag(i));
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
            if (RobotBase.isSimulation())
                m_gyroSim.setGyroAngleZ(m_odometry.getPoseMeters().getRotation().getDegrees());
            else
                m_gyroscope.setGyroAngleZ(m_odometry.getPoseMeters().getRotation().getDegrees());
        }
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        for (final SwerveModule module : m_modules) module.simulationPeriodic();
        SwerveModuleState[] states = new SwerveModuleState[]{
            new SwerveModuleState(
                m_modules[0].getVelocity(),
                m_modules[0].getSteerAngle()),
            new SwerveModuleState(
                m_modules[1].getVelocity(),
                m_modules[1].getSteerAngle()),
            new SwerveModuleState(
                m_modules[2].getVelocity(),
                m_modules[2].getSteerAngle()),
            new SwerveModuleState(
                m_modules[3].getVelocity(),
                m_modules[3].getSteerAngle())
        };
        m_gyroSim.setGyroAngleZ(Rotation2d.fromDegrees(m_gyroscope.getAngle()).plus(Rotation2d.fromRadians(m_kinematics.toChassisSpeeds(states).omegaRadiansPerSecond).times(0.02)).getDegrees());
        //m_gyroscope.setGyroAngle(Constants.kYawAxis, m_heading.getDegrees());
    }

    /**
     * Only for sysid.
     * @param direction
     * @return
     */
    public Command quasiDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    /**
     * Only for sysid.
     * @param direction
     * @return
     */
    public Command dynaDriveIdCommand(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    /**
     * Only for sysid.
     * @param direction
     * @return
     */
    public Command quasiSteerIdCommand(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    /**
     * Only for sysid.
     * @param direction
     * @return
     */
    public Command dynaSteerIdCommand(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    /**
     * @param fwd Forward rate in m/s.
     * @param strafe Strafing rate in m/s.
     * @param turn The angular velocity.
     * @return The driving command.
     */
    public Command driveCommand(DoubleSupplier fwd, DoubleSupplier strafe, Supplier<AngularVelocity> turn) {
        return this.run(
            () -> {
                double fwdVelocity = fwd.getAsDouble();
                double strafeVelocity = strafe.getAsDouble();
                double magnitude = Math.hypot(fwdVelocity, strafeVelocity);
                if (Math.hypot(fwdVelocity, strafeVelocity) > Constants.kMaxVelocity) {
                    fwdVelocity *= Constants.kMaxVelocity / magnitude;
                    strafeVelocity *= Constants.kMaxVelocity / magnitude;
                }
                final SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(fwdVelocity, strafeVelocity, turn.get().in(RadiansPerSecond)));
                for (int i = 0; i < states.length; ++i) {
                    if (Constants.kCosineScale) {
                        states[i].cosineScale(m_modules[i].getPosition().angle);
                    }
                    m_modules[i].goToState(MetersPerSecond.of(states[i].speedMetersPerSecond), states[i].angle);
                }
            }
        );
    }

    public Command driveHeadingCommand(DoubleSupplier fwd, DoubleSupplier strafe, Supplier<Rotation2d> heading) {
        return this.driveCommand(fwd, strafe, () -> 
            RadiansPerSecond.of(m_gyroPID.calculate(getGyroscope().getRadians(), SwerveModule.getClosestRotation(getGyroscope(), heading.get()).getRadians()))
        );
    }

    public Command followTrajectoryCommand(Supplier<Trajectory> trajectory) {
        return this.startRun(
            () -> {
                m_currentTrajectory = trajectory.get();
                m_trajectoryTimer.reset();
                m_trajectoryTimer.start();
            },
            () -> {
                ChassisSpeeds speeds = m_controller.calculate(m_odometry.getPoseMeters(), m_currentTrajectory.sample(m_trajectoryTimer.get()), m_currentTrajectory.sample(m_currentTrajectory.getTotalTimeSeconds()).poseMeters.getRotation());
                // NOTE: maybe make this cleaner
                this.driveCommand(
                    () -> speeds.vxMetersPerSecond,
                    () -> speeds.vyMetersPerSecond,
                    () -> RadiansPerSecond.of(speeds.omegaRadiansPerSecond)).execute();;
            }
        ).onlyWhile(() -> m_currentTrajectory.getTotalTimeSeconds() > m_trajectoryTimer.get());
    }

    /**
     * @param pose The pose to make a trajectory to
     * @return
     */
    public Command goToPosCommand(Pose2d pose) {
        return this.followTrajectoryCommand(
            () -> TrajectoryGenerator.generateTrajectory(m_odometry.getPoseMeters(), List.of(), pose, new TrajectoryConfig(Constants.kMaxVelocity, Constants.kMaxAcceleration))
        );
    }

    public Command goToAprilTag(int aprilTag) {
        Pose2d target = m_aprilTags.getTagPose(aprilTag).get().toPose2d();
        return this.goToPosCommand(target
            .rotateAround(target.getTranslation(), Rotation2d.k180deg)
            .transformBy(new Transform2d(-Constants.kRobotDepth/2, 0.0, Rotation2d.kZero)))
            .withName(String.format("Go To AprilTag %d", aprilTag));
    }

    // get it to work with elastic's swerve drive widget
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", () -> m_modules[1].getSteerAngle().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", m_modules[1]::getVelocity, null);
        builder.addDoubleProperty("Front Right Angle", () -> m_modules[0].getSteerAngle().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", m_modules[0]::getVelocity, null);
        builder.addDoubleProperty("Back Left Angle", () -> m_modules[3].getSteerAngle().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", m_modules[3]::getVelocity, null);
        builder.addDoubleProperty("Back Right Angle", () -> m_modules[2].getSteerAngle().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", m_modules[2]::getVelocity, null);
        builder.addDoubleProperty("Robot Angle", () -> this.getGyroscope().plus(Rotation2d.kCCW_Pi_2).getRadians(), null);
    }

    /**
     * @return The value of the gyroscope.
     */
    private Rotation2d getGyroscope() {
        return Rotation2d.fromDegrees(m_gyroscope.getAngle());
    }

    /**
     * @return The current voltage drawn from the swerve drivetrain.
     */
    public Voltage getCurrentDraw() {
        return m_modules[0].getCurrentDraw().plus(m_modules[1].getCurrentDraw()).plus(m_modules[2].getCurrentDraw()).plus(m_modules[3].getCurrentDraw());
    }
}
