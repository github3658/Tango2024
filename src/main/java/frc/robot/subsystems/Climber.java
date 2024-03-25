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
        ManualLower, // DOES NOT USE ENCODER CHECKS! USE WITH CAUTION!
    }
    ClimbState e_ClimbStateLeft = ClimbState.Idle;
    ClimbState e_ClimbStateRight = ClimbState.Idle;

    /* CONSTANTS (prefix: c) */
    private final int c_ClimbLeftID  = 17;
    private final int c_ClimbRightID = 19;
    private final double c_SetClimbSpeed   = 1.0;
    private final double c_SetReleaseSpeed = -1.0;
    private final double c_MaxEncoderValue = 0.0;
    private final double c_MinEncoderValueLeft = -620.0;
    private final double c_MinEncoderValueRight = -455.0;

    /* MOTORS (prefix: m) */
    private final TalonFX m_ClimbLeft;
    private final TalonFX m_ClimbRight;

    /* OTHER VARIABLES */
    private double d_EncoderLeftOffset  = 0.0;
    private double d_EncoderRightOffset = 0.0;
    private boolean b_IsResetting = false;

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
        switch (e_ClimbStateLeft) {
            case Climb:
                if (!(getExtendLeft() > c_MaxEncoderValue)) {
                    m_ClimbLeft.set(c_SetClimbSpeed);
                }
                else {
                    e_ClimbStateLeft = ClimbState.Idle;
                }
                break;
            case Release:
                if (!(getExtendLeft() < c_MinEncoderValueLeft)) {
                    m_ClimbLeft.set(c_SetReleaseSpeed);
                }
                else {
                    e_ClimbStateLeft = ClimbState.Idle;
                }
                break;
            case ManualLower:
                m_ClimbLeft.set(c_SetClimbSpeed);
                b_IsResetting = true;
                break;
            default:
                m_ClimbLeft.set(0.0);
                break;
        }
        switch (e_ClimbStateRight) {
            case Climb:
                if (!(getExtendRight() > c_MaxEncoderValue)) {
                    m_ClimbRight.set(c_SetClimbSpeed);
                }
                else {
                    e_ClimbStateRight = ClimbState.Idle;
                }
                break;
            case Release:
                if (!(getExtendRight() < c_MinEncoderValueRight)) {
                    m_ClimbRight.set(c_SetReleaseSpeed);
                }
                else {
                    e_ClimbStateRight = ClimbState.Idle;
                }
                break;
            case ManualLower:
                m_ClimbRight.set(c_SetClimbSpeed);
                b_IsResetting = true;
                break;
            default:
                m_ClimbRight.set(0.0);
                break;
        }

        if ((e_ClimbStateLeft != ClimbState.ManualLower && e_ClimbStateRight != ClimbState.ManualLower) && b_IsResetting) {
            d_EncoderLeftOffset += getExtendLeft();
            d_EncoderRightOffset += getExtendRight();
            b_IsResetting = false;
        }

        outputTelemetry();
    }

    public void stop() {
        e_ClimbStateLeft  = ClimbState.Idle;
        e_ClimbStateRight = ClimbState.Idle;
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
     * Sets the climber's current state, determining speed of motors. If you want to set both climber motors individually, use setLeftState and setRightState.
     * @param cs ClimbState value
     */
    public void setState(ClimbState cs) {
        e_ClimbStateLeft = cs;
        e_ClimbStateRight = cs;
    }

    /**
     * Sets the left climber motors's current state, determining its speed.
     * @param cs ClimbState value
     */
    public void setLeftState(ClimbState cs) {
        e_ClimbStateLeft = cs;
    }

    /**
     * Sets the right climber motors's current state, determining its speed.
     * @param cs ClimbState value
     */
    public void setRightState(ClimbState cs) {
        e_ClimbStateRight = cs;
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
  
