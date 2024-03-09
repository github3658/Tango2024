// TODO: Finalize climber controls with drive team

// This is the climber subsystem

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
public class Climber extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ClimbID = 17;
    private final double c_SetClimbSpeed = 1.0;
    private final double c_SetReleaseSpeed = -1.0;

    /* MOTORS (prefix: m) */
    private final TalonFX m_Climb;

    /* OTHER VARIABLES */
    private double d_ClimbSpeed = 0.0;
    private double d_EncoderOffset = 0.0;

    //private LED s_LED;
    public Climber(LED led) {
        //s_LED = led;
        m_Climb = new TalonFX(c_ClimbID, "3658CANivore");
        m_Climb.setNeutralMode(NeutralModeValue.Brake);
        m_Climb.setInverted(false);
        d_EncoderOffset = m_Climb.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // TODO: Create climber periodic.

        m_Climb.set(d_ClimbSpeed);

        outputTelemetry();
    }

    public void stop() {
        d_ClimbSpeed = 0.0;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber - Extend", getExtend());
    }

    public void setNeutralMode(NeutralModeValue neutral) {
        m_Climb.setNeutralMode(neutral);
    }

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


    public void release() {
        if (getExtend() > -687.00) {
            d_ClimbSpeed = c_SetReleaseSpeed;
        }
        else {
            d_ClimbSpeed = 0.0;
        }
    }
    
    public void stopClimb() {
        d_ClimbSpeed = 0.0;
    }

    private double getExtend() {
        return m_Climb.getPosition().getValueAsDouble() - d_EncoderOffset;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_Climb};
        return pd;
    }

    public double pollOrchOutput() {
        return Math.abs(m_Climb.get());
    }
}
  
