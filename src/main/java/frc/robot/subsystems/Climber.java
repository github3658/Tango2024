// This is the climber subsystem

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
public class Climber extends SubsystemBase {
    // And for my next trick, I will create an enum in Climber.java!
    public enum ClimbState {
        Idle,
        Climb,
        Release,
        TiltLeft,
        TiltRight,
        ManualLower, // DOES NOT USE ENCODER CHECKS! USE WITH CAUTION!
    }
    ClimbState e_ClimbState = ClimbState.Idle;

    /* CONSTANTS (prefix: c) */
    private final int c_ClimbLeftID  = 17;
    private final int c_ClimbRightID = 19;
    private final double c_SetClimbSpeed   = 1.0;
    private final double c_SetReleaseSpeed = -1.0;
    private final double c_MaxEncoderValue = 0.0;
    private final double c_MinEncoderValue = -687.0;

    /* MOTORS (prefix: m) */
    private final TalonFX m_ClimbLeft;
    private final TalonFX m_ClimbRight;

    /* OTHER VARIABLES */
    private double d_EncoderLeftOffset  = 0.0;
    private double d_EncoderRightOffset = 0.0;

    /**
     * This is the constructor for the Climber subsystem.
     * @param led The LED subsystem. CURRENTLY UNUSED.
     */
    public Climber(LED led) {
        m_ClimbLeft  = new TalonFX(c_ClimbLeftID,  "3658CANivore");
        m_ClimbRight = new TalonFX(c_ClimbRightID, "3658CANivore");
        m_ClimbLeft. setNeutralMode(NeutralModeValue.Brake);
        m_ClimbRight.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbLeft. setInverted(false);
        m_ClimbRight.setInverted(true);
        d_EncoderLeftOffset  = m_ClimbLeft. getPosition().getValueAsDouble();
        d_EncoderRightOffset = m_ClimbRight.getPosition().getValueAsDouble();
    }

    /**
     * This function runs repeatedly. It sets the motor speeds of the climber motors and outputs telemetry.
     */
    @Override
    public void periodic() {
        switch (e_ClimbState) {
            case Climb:
                if (!(getExtendLeft() > c_MaxEncoderValue)) {
                    m_ClimbLeft.set(c_SetClimbSpeed);
                }
                if (!(getExtendRight() > c_MaxEncoderValue)) {
                    m_ClimbRight.set(c_SetClimbSpeed);
                }
                break;
            case Release:
                if (!(getExtendLeft() < c_MinEncoderValue)) {
                    m_ClimbLeft.set(c_SetReleaseSpeed);
                }
                if (!(getExtendRight() > c_MinEncoderValue)) {
                    m_ClimbRight.set(c_SetReleaseSpeed);
                }
                break;
            case TiltLeft:
                if (!(getExtendLeft() > c_MaxEncoderValue)) {
                    m_ClimbLeft.set(c_SetClimbSpeed);
                }
                if (!(getExtendRight() < c_MinEncoderValue)) {
                    m_ClimbRight.set(c_SetReleaseSpeed);
                }
                break;
            case TiltRight:
                if (!(getExtendLeft() < c_MinEncoderValue)) {
                    m_ClimbLeft.set(c_SetReleaseSpeed);
                }
                if (!(getExtendRight() > c_MaxEncoderValue)) {
                    m_ClimbRight.set(c_SetClimbSpeed);
                }
                break;
            case ManualLower:
                m_ClimbLeft.set(c_SetClimbSpeed);
                m_ClimbRight.set(c_SetClimbSpeed);
                break;
            default:
                m_ClimbLeft.set(0.0);
                m_ClimbRight.set(0.0);
                break;
        }

        outputTelemetry();
    }

    public void stop() {
        e_ClimbState = ClimbState.Idle;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber - Extend (L)", getExtendLeft());
        SmartDashboard.putNumber("Climber - Extend (R)", getExtendRight());
    }

    public void setNeutralMode(NeutralModeValue neutral) {
        m_ClimbLeft. setNeutralMode(neutral);
        m_ClimbRight.setNeutralMode(neutral);
    }

    /**
     * Sets the climber's current state, determining speed of motors.
     * @param cs
     */
    public void setClimbState(ClimbState cs) {
        e_ClimbState = cs;
    }

    /**
     * Get the encoder value of the left climbing motor, adjusted such that 0 is the lowest point.
     * @return Adjusted encoder value as double
     */
    private double getExtendLeft() {
        return m_ClimbLeft.getPosition().getValueAsDouble() - d_EncoderLeftOffset;
    }

    /**
     * Get the encoder value of the right climbing motor, adjusted such that 0 is the lowest point.
     * @return Adjusted encoder value as double
     */
    private double getExtendRight() {
        return m_ClimbRight.getPosition().getValueAsDouble() - d_EncoderRightOffset;
    }

    /**
     * Get devices in this subsystem that the orchestra can use.
     * @return ParentDevices. This includes TalonFX, Pigeon2, and other CTRE devices.
     */
    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {};//{m_ClimbLeft, m_ClimbRight};
        return pd;
    }

    /**
     * Get the combined output of this subsytem's motors.
     * @return The sum of motor outputs as a double.
     */
    public double pollOrchOutput() {
        return 0;//Math.abs(m_ClimbLeft.get()) + Math.abs(m_ClimbRight.get());
    }
}
  
