package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final SparkMax m_steerMotor;
    private final SparkMax m_driveMotor;
    private final RelativeEncoder m_steerEncoder;
    private final RelativeEncoder m_driveEncoder;

    private final int swerveModNum;
    
    private SwerveModulePosition m_position = new SwerveModulePosition();

    // Simulation purposes
    private FlywheelSim m_steerSim;
    private FlywheelSim m_driveSim;

    public SwerveModule(int steerId, int driveId, int swerveNum) {
        swerveModNum = swerveNum;
        m_steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        configureMotor(m_steerMotor, 0, 0, 0, 0);
        configureMotor(m_driveMotor, 0, 0, 0, 0);
        m_steerEncoder = m_steerMotor.getEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();

        if (RobotBase.isSimulation()) {
            m_steerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
                    0.5,
                    1),
                DCMotor.getNEO(1));
            m_driveSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
                    0.5,
                    1),
                DCMotor.getNEO(1));
        }
    }

    /**
     * @param motor Motor to configure.
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param ff Feed Forward gain
     */
    private void configureMotor(SparkMax motor, double p, double i, double d, double ff) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pidf(p, i, d, ff);
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
        m_position = new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRotations(m_steerEncoder.getPosition()));
        // push to smartdashboard
        SmartDashboard.putNumber(String.format("Swerve Position %d", swerveModNum), m_driveEncoder.getPosition());
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
     * Only used for sysid.
     * @return Drive encoder.
     */
    public RelativeEncoder getDriveEncoder() {
        return m_driveEncoder;
    }

    /**
     * Only used for sysid.
     * @return Steering encoder.
     */
    public RelativeEncoder getSteerEncoder() {
        return m_steerEncoder;
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
        m_steerMotor.getClosedLoopController().setReference(steer, ControlType.kPosition);
        m_driveMotor.getClosedLoopController().setReference(drive, ControlType.kVelocity);
        if (RobotBase.isSimulation()) {
            m_steerSim.setInputVoltage(RobotController.getBatteryVoltage() * m_steerMotor.get());
            m_driveSim.setInputVoltage(RobotController.getBatteryVoltage() * m_driveMotor.get());
            m_steerSim.update(0.02);
            m_driveSim.update(0.02);
            m_steerEncoder.setPosition(
                m_steerEncoder.getPosition() + 0.02 * m_steerSim.getAngularVelocityRPM());
            m_driveEncoder.setPosition(
                m_driveEncoder.getPosition() + 0.02 * m_driveSim.getAngularVelocityRPM());
        }
    }
}
