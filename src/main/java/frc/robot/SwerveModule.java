package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SwerveModule implements Sendable {
    // Motor and encoder setup
    // STEER is in terms of rots and RPM
    // DRIVE is in terms of meters and m/s
    private final SparkMax m_steerMotor;
    private final SparkMax m_driveMotor;
    private final RelativeEncoder m_steerEncoder;
    private final RelativeEncoder m_driveEncoder;

    public final int swerveModNum;
    private Rotation2d steerSetpoint = Rotation2d.kZero;
    private LinearVelocity driveSetpoint = MetersPerSecond.zero();
    
    private SwerveModulePosition m_position = new SwerveModulePosition();

    // Simulation purposes
    private SparkSim m_steerSparkSim;
    private SparkSim m_driveSparkSim;
    private FlywheelSim m_steerFlysim;
    private FlywheelSim m_driveFlysim;

    public SwerveModule(int steerId, int driveId, int swerveNum) {
        swerveModNum = swerveNum;
        m_steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        //configureMotor(m_steerMotor, 0.01, 0, 0, 0.02, STEER_GEARING);
        configureMotor(m_steerMotor,
            Constants.kPSteer, Constants.kISteer, Constants.kDSteer, Constants.kFFSteer,
            Constants.kSteerGearing, Constants.kSteerGearing);
        configureMotor(m_driveMotor,
            Constants.kPDrive, Constants.kIDrive, Constants.kDDrive, Constants.kFFDrive,
            Constants.kWheelCircum.times(Constants.kDriveGearing).in(Meters), Constants.kWheelCircum.times(Constants.kDriveGearing).div(60).in(Meters));
        m_steerEncoder = m_steerMotor.getEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();

        if (RobotBase.isSimulation() ) {
            m_steerSparkSim = new SparkSim(m_steerMotor, DCMotor.getNEO(1));
            m_driveSparkSim = new SparkSim(m_driveMotor, DCMotor.getNEO(1));
            m_steerFlysim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(Constants.kVSteer, Constants.kASteer),
                DCMotor.getNEO(1),
                Constants.kSimNoise);
            m_driveFlysim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(Constants.kVDrive * Constants.kWheelRadius.in(Meters), Constants.kADrive * Constants.kWheelRadius.in(Meters)),
                DCMotor.getNEO(1),
                Constants.kSimNoise);
        }
    }

    /**
     * @param motor Motor to configure.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param ff Feed forward gain.
     * @param posFactor Position conversion factor.
     * @param velFactor Velocity conversion factor.
     */
    private void configureMotor(SparkMax motor, double p, double i, double d, double ff, double posFactor, double velFactor) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pidf(p, i, d, ff)
            .positionWrappingEnabled(false)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(posFactor)
            .velocityConversionFactor(velFactor);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Reset the encoders.
     */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0.0);
        m_steerEncoder.setPosition(0.0);
    }

    /**
     * Update position of swerve module.
     */
    public void updatePosition() {
        m_position = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(m_steerEncoder.getPosition()));
        // push to smartdashboard
    }

    /**
     * @return The position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return m_position;
    }

    /**
     * Only used for sysid.
     * @param volts The voltage to apply.
     */
    public void setDrive(Voltage volts) {
        m_driveMotor.setVoltage(volts);
    }

    /**
     * Only used for sysid.
     * @param volts The voltage to apply.
     */
    public void setSteer(Voltage volts) {
        m_steerMotor.setVoltage(volts);
    }

    /**
     * Pretty much used only for sysid.
     * @return Drive encoder distance.
     */
    public Distance getDrivePosition() {
        return Meters.of(m_driveEncoder.getPosition());
    }

    /**
     * @return Velocity of the drive motor.
     */
    public double getVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Only used for sysid.
     * @return Velocity of the steer motor.
     */
    public double getSteerVelocity() {
        return m_steerEncoder.getVelocity();
    }

    /**
     * @return Angle of the swerve module.
     */
    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(m_steerEncoder.getPosition());
    }

    /**
     * Only used for sysid.
     * @return Applied voltage of drive motor.
     */
    public Voltage getDriveVoltage() {
        return Volts.of(m_driveMotor.getBusVoltage()).times(m_driveMotor.getAppliedOutput());
    }

    /**
     * Only used for sysid.
     * @return Applied voltage of steer motor.
     */
    public Voltage getSteerVoltage() {
        return Volts.of(m_steerMotor.getBusVoltage()).times(m_steerMotor.getAppliedOutput());
    }

    public Voltage getCurrentDraw() {
        return Volts.of(getDriveVoltage().abs(Volts)).plus(Volts.of(getSteerVoltage().abs(Volts)));
    }

    /**
     * @param current The current heading.
     * @param setpoint The setpoint heading.
     * @return The setpoint adjusted to be the closest rotation to the current.
     */
    public static Rotation2d getClosestRotation(Rotation2d current, Rotation2d setpoint) {
        return Rotation2d.fromDegrees(Rotation2d.fromDegrees(360).times(
            Math.round(current.getRotations() - setpoint.getRotations())
        ).getDegrees() + setpoint.getDegrees());
    }

    /**
     * Uses built-in PID to go to a certain position.
     * @param drive The desired m/s of the drive motor.
     * @param steer The desired rotation of the steer motor.
     */
    public void goToState(LinearVelocity drive, Rotation2d steer) {
        driveSetpoint = drive;
        steerSetpoint = steer;
        if (drive != MetersPerSecond.zero()) m_steerMotor.getClosedLoopController().setReference(
            getClosestRotation(getSteerAngle(), steer).getRotations(), ControlType.kPosition);
        //    steer.getRotations(), ControlType.kPosition);
        m_driveMotor.getClosedLoopController().setReference(drive.in(MetersPerSecond), ControlType.kVelocity);
    }

    public void simulationPeriodic() {
        double busVoltage = RoboRioSim.getVInVoltage();
        m_steerFlysim.setInputVoltage(busVoltage * m_steerSparkSim.getAppliedOutput());
        m_driveFlysim.setInputVoltage(busVoltage * m_driveSparkSim.getAppliedOutput());
        m_steerFlysim.update(0.02);
        m_driveFlysim.update(0.02);
        m_steerSparkSim.iterate(
            m_steerMotor.configAccessor.encoder.getVelocityConversionFactor() * m_steerFlysim.getAngularVelocityRPM(),
            busVoltage,
            0.02);
        m_driveSparkSim.iterate(
            m_driveMotor.configAccessor.encoder.getVelocityConversionFactor() * m_driveFlysim.getAngularVelocityRPM(),
            busVoltage,
            0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //builder.setSmartDashboardType();
        builder.addDoubleProperty("Drive Setpoint", () -> driveSetpoint.in(MetersPerSecond), drive -> this.goToState(MetersPerSecond.of(drive), steerSetpoint));
        builder.addDoubleProperty("Steer Setpoint", () -> steerSetpoint.getDegrees(), steer -> this.goToState(driveSetpoint, Rotation2d.fromDegrees(steer)));
        builder.addDoubleProperty("Drive Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Steer Position", () -> this.getSteerAngle().getDegrees(), null);
    }
}
