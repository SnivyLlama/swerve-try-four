package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private final SparkMax m_steerMotor;
    private final SparkMax m_driveMotor;

    private final RelativeEncoder m_steerEncoder;
    private final RelativeEncoder m_driveEncoder;
    
    private SwerveModulePosition m_position = new SwerveModulePosition();

    public SwerveModule(int steerId, int driveId) {
        m_steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        m_steerEncoder = m_steerMotor.getEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();
    }

    public void resetEncoder() {
        m_steerMotor.getEncoder().setPosition(0.0);
    }

    public void updatePosition() {
        m_position = new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRotations(m_steerEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return m_position;
    }

    public void goToState(double drive, double steer) {
        m_steerMotor.getClosedLoopController().setReference(steer, ControlType.kPosition);
        m_driveMotor.getClosedLoopController().setReference(drive, ControlType.kVelocity);
    }
}
