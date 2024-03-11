// This is the climber subsystem

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
public class Climber extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ClimbLeftID = 17;
    private final int c_ClimbRightID = 19;
    private final double c_SetClimbSpeed = 1.0;
    private final double c_SetReleaseSpeed = -1.0;

    /* MOTORS (prefix: m) */
    private final TalonFX m_ClimbLeft;
    private final TalonFX m_ClimbRight;

    /* OTHER VARIABLES */
    private double d_ClimbSpeed = 0.0;
    private double d_EncoderOffset = 0.0;

    /**
     * This is the constructor for the Climber subsystem.
     * @param led The LED subsystem. CURRENTLY UNUSED.
     */
    public Climber(LED led) {
        m_ClimbLeft = new TalonFX(c_ClimbLeftID, "3658CANivore");
        m_ClimbRight = new TalonFX(c_ClimbRightID, "3658CANivore");
        m_ClimbLeft.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbRight.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbLeft.setInverted(false);
        m_ClimbRight.setInverted(true);
        d_EncoderOffset = m_ClimbLeft.getPosition().getValueAsDouble();
    }

    /**
     * This function runs repeatedly. It sets the motor speeds of the climber motors and outputs telemetry.
     */
    @Override
    public void periodic() {
        m_ClimbLeft.set(d_ClimbSpeed);
        m_ClimbRight.set(d_ClimbSpeed);

        outputTelemetry();
    }

    public void stop() {
        d_ClimbSpeed = 0.0;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber - Extend", getExtend());
    }

    public void setNeutralMode(NeutralModeValue neutral) {
        m_ClimbLeft.setNeutralMode(neutral);
        m_ClimbRight.setNeutralMode(neutral);
    }

    /**
     * Set both climber motors to lower, causing the robot to climb.
     */
    public void climb() {
        if (getExtend() < 0.00) {
            d_ClimbSpeed = c_SetClimbSpeed;
        }
        else {
            d_ClimbSpeed = 0.0;
        }
    }

    public void setMotorSpeed(double speed) {
        d_ClimbSpeed = speed;
    }


    /**
     * Set both climber motors to raise, causing the robot to lower.
     */
    public void release() {
        if (getExtend() > -687.00) {
            d_ClimbSpeed = c_SetReleaseSpeed;
        }
        else {
            d_ClimbSpeed = 0.0;
        }
    }
    
    /**
     * Zero all climber motors.
     */
    public void stopClimb() {
        d_ClimbSpeed = 0.0;
    }

    private double getExtend() {
        return m_ClimbLeft.getPosition().getValueAsDouble() - d_EncoderOffset;
    }

    /**
     * Get devices in this subsystem that the orchestra can use.
     * @return ParentDevices. This includes TalonFX, Pigeon2, and other CTRE devices.
     */
    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_ClimbLeft, m_ClimbRight};
        return pd;
    }

    /**
     * Get the combined output of this subsytem's motors.
     * @return The sum of motor outputs as a double.
     */
    public double pollOrchOutput() {
        return Math.abs(m_ClimbLeft.get()) + Math.abs(m_ClimbRight.get());
    }
}
  
