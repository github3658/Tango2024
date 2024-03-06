// TODO: Code to calculate motor speed to note distance
// TODO: Finalize shooter controls with drive team

// This is the shooter subsystem.
// Currently, it shoots. We'd like for it to move and pivot to shoot at specific angles with accuracy.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
	/* CONSTANTS (prefix: c) */
	private final int c_ShootLeftID   = 14;
	private final int c_ShootRightID  = 15;
	private final int c_ShootPivotID  = 16; // Encoder range 0 - -36.5
	private final int c_ShootExtendID = 13; // Encoder range 0 - 20

	/* MOTORS (prefix: m) */
	private final TalonFX m_ShootLeft;
	private final TalonFX m_ShootRight;
	private final TalonFX m_ShootPivot;
	private final TalonFX m_ShootExtend;

	/* OTHER VARIABLES */
	private double d_ShooterSpeed = 0.0;
	private double d_ExtensionPosition = 0.0;
	private double d_PivotPosition = 0.0;

	private LED s_LED;
	public Shooter(LED led) {
		s_LED = led;
		m_ShootLeft = new TalonFX(c_ShootLeftID, "3658CANivore");
		m_ShootRight = new TalonFX(c_ShootRightID, "3658CANivore");
		m_ShootPivot = new TalonFX(c_ShootPivotID, "3658CANivore");
		m_ShootExtend = new TalonFX(c_ShootExtendID, "3658CANivore");
		m_ShootLeft.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootRight.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootPivot.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootExtend.getConfigurator().apply(new TalonFXConfiguration());

		m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);
		m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);
		m_ShootPivot.setNeutralMode(NeutralModeValue.Coast);
		m_ShootExtend.setNeutralMode(NeutralModeValue.Coast);

		m_ShootLeft.setInverted(true);
		m_ShootRight.setInverted(false);

		m_ShootPivot.setPosition(0);
	}

	@Override
	public void periodic() {
		m_ShootLeft.set(d_ShooterSpeed);
		m_ShootRight.set(d_ShooterSpeed);
		
		// Extension Control
        double d_CurrentExtension = m_ShootExtend.getPosition().getValueAsDouble();
        m_ShootExtend.set(Math.max(Math.min(((d_ExtensionPosition - d_CurrentExtension) / 5 * 0.2),0.30),-0.30));

		// Pivot Control
        double d_CurrentPivot = m_ShootPivot.getPosition().getValueAsDouble();
        m_ShootPivot.set(Math.max(Math.min(((d_PivotPosition - d_CurrentPivot) / 5 * 0.2),0.30),-0.30));

		outputTelemetry();
	}

	public void stop() {
		setSpeed(0.0);
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Shooter Speed", d_ShooterSpeed);
		SmartDashboard.putNumber("Left Speed", m_ShootLeft.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Right Speed", m_ShootRight.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Shooter Extension",m_ShootExtend.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Shooter Pivot",m_ShootPivot.getPosition().getValueAsDouble());
	}

	public void setSpeed(double speed) {
		d_ShooterSpeed = speed;
	}

	public void setExtension(double position, boolean relative) {
		if (relative) {
			d_ExtensionPosition = Math.min(Math.max(d_ExtensionPosition+position,0),23);
		}
		else {
			d_ExtensionPosition = Math.min(Math.max(position,0),23);
		}
	}

	public void setExtension(double position) {
		setExtension(position, false);
	}

	public void setPivot(double position, boolean relative) {
		if (relative) {
			d_PivotPosition = Math.min(Math.max(d_PivotPosition+position,-36.5),0);
		}
		else {
			d_PivotPosition = Math.min(Math.max(position,-36.5),0);
		}
	}

	public void setPivot(double position) {
		setPivot(position, false);
	}

	public ParentDevice[] requestOrchDevices() {
		ParentDevice[] pd = {m_ShootLeft, m_ShootRight};
		return pd;
	}
}