package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.math.Gearbox;
import frc.robot.Constants;

public class Module {
    
    private WPI_TalonFX m_driveMotor;
    private WPI_TalonSRX m_turnMotor;

    private Gearbox m_driveGearbox;

    public Module(WPI_TalonFX driveMotor, WPI_TalonSRX turnMotor, Gearbox driveGearbox) {
        m_driveMotor = driveMotor;
        m_turnMotor = turnMotor;
        m_driveGearbox = driveGearbox;

        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_turnMotor.setNeutralMode(NeutralMode.Brake);

        // m_driveMotor.configAllSettings(allConfigs) 
        // TODO: motor config

        m_turnMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
            0, 20);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), getAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentRot = getAngle();
        state = SwerveModuleState.optimize(state, currentRot);
        
        m_driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / Constants.Drivetrain.WHEEL_CIRCUMFERENCE / 10 * 4096);
        m_turnMotor.set(ControlMode.Position, state.angle.getDegrees() / 360 * 4096);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(m_turnMotor.getSelectedSensorPosition() / 4096 * 360); // check if +- is correct
    }

    /**
     * 
     * @return drive wheel speed in m/s
     */
    public double getDriveSpeed() {
        return m_driveGearbox
            .drivingRotationToDriven((m_driveMotor.getSelectedSensorVelocity() * 10) / 4096) // check if encoder is inverted
            * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;
    }
}
