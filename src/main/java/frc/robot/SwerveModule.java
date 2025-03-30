package frc.robot;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SwerveModule implements Sendable {
    // Gearing
    private final double STEER_GEARING = 21.5;
    private final double DRIVE_GEARING = 1.0;
    private final double DRIVE_CIRCUM = 2 * Math.PI;
    private final double NOISE = 1e-5;
    // Motor and encoder setup
    private final SparkMax m_steerMotor;
    private final SparkMax m_driveMotor;
    private final RelativeEncoder m_steerEncoder;
    private final RelativeEncoder m_driveEncoder;

    public final int swerveModNum;
    private double steerSetpoint = 0.0;
    private double driveSetpoint = 0.0;
    
    private SwerveModulePosition m_position = new SwerveModulePosition();

    // Simulation purposes
    private SparkSim m_steerSparkSim;
    private SparkSim m_driveSparkSim;
    private FlywheelSim m_steerSim;
    private FlywheelSim m_driveSim;

    public SwerveModule(int steerId, int driveId, int swerveNum) {
        swerveModNum = swerveNum;
        m_steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        //configureMotor(m_steerMotor, 0.01, 0, 0, 0.02, STEER_GEARING);
        configureMotor(m_steerMotor, 0.0075, 0, 0.001, 0.0, STEER_GEARING);
        configureMotor(m_driveMotor, 0.01, 0, 0, 0.0, DRIVE_GEARING * DRIVE_CIRCUM);
        m_steerEncoder = m_steerMotor.getEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();

        if (RobotBase.isSimulation()) {
            m_steerSparkSim = new SparkSim(m_steerMotor, DCMotor.getNEO(1));
            m_driveSparkSim = new SparkSim(m_driveMotor, DCMotor.getNEO(1));
            m_steerSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(2.0, 0.1),
                DCMotor.getNEO(1),
                NOISE);
            m_driveSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(2.0, 0.1),
                DCMotor.getNEO(1),
                NOISE);
        }
    }

    /**
     * @param motor Motor to configure.
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param ff Feed Forward gain
     * @param convFactor Conversion factor
     */
    private void configureMotor(SparkMax motor, double p, double i, double d, double ff, double convFactor) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pidf(p, i, d, ff)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(convFactor);
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
     * @return Drive encoder.
     */
    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    /**
     * @return Velocity of the drive motor.
     */
    public double getVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Only used for sysid.
     * @return Steering encoder.
     */
    public RelativeEncoder getSteerEncoder() {
        return m_steerEncoder;
    }

    /**
     * @return Angle in radians.
     */
    public double getSteerAngle() {
        return Units.rotationsToRadians(m_steerEncoder.getPosition());
    }

    /**
     * Only used for sysid.
     * @return Applied voltage of drive motor.
     */
    public double getDriveVoltage() {
        return m_driveMotor.getAppliedOutput();
    }

    /**
     * Only used for sysid.
     * @return Applied voltage of steer motor.
     */
    public double getSteerVoltage() {
        return m_steerMotor.getAppliedOutput();
    }

    /**
     * Uses built-in PID to go to a certain position.
     * @param drive The desired m/s of the drive motor.
     * @param steer The desired ?? of the steer motor.
     */
    public void goToState(double drive, double steer) {
        driveSetpoint = drive;
        steerSetpoint = Units.rotationsToDegrees(steer);
        m_steerMotor.getClosedLoopController().setReference(steer, ControlType.kPosition);
        m_driveMotor.getClosedLoopController().setReference(drive, ControlType.kVelocity);
    }

    public void simulationPeriodic() {
        m_steerSim.setInputVoltage(RobotController.getBatteryVoltage() * m_steerSparkSim.getAppliedOutput());
        m_driveSim.setInputVoltage(RobotController.getBatteryVoltage() * m_driveSparkSim.getAppliedOutput());
        m_steerSim.update(0.02);
        m_driveSim.update(0.02);

        double busVoltage = RoboRioSim.getVInVoltage();
        m_steerSparkSim.iterate(
            m_steerSim.getAngularVelocityRPM(),
            busVoltage,
            0.02);
        m_driveSparkSim.iterate(
            m_driveSim.getAngularVelocityRPM(),
            busVoltage,
            0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //builder.setSmartDashboardType();
        builder.addDoubleProperty("Drive Setpoint", () -> driveSetpoint, null);
        builder.addDoubleProperty("Steer Setpoint", () -> steerSetpoint, null);
        builder.addDoubleProperty("Drive Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Steer Position", () -> Units.radiansToDegrees(this.getSteerAngle()), null);
    }
}
