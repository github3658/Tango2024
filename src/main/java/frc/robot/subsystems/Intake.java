// TODO: Finalize intake controls with drive team

// This is the intake subsystem. 
// We use two motors and an absolute encoder to intake notes and pivot to various positions.
// Most Intake logic exists in the command IntakeTeleop

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.LED.Color;

public class Intake extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_IntakeNoteID = 10;
    private final int c_IntakePivotID = 9;
    private final int c_CANCoderID = 18;

    /**
     * An enum of possible targets for the intake pivot. Includes:
     * <p> None (Default state, no target)
     * <p> Ground
     * <p> Source (Not implemented)
     * <p> Amp
     * <p> Stow
     */
    public enum PivotTarget {
        None,
        Ground,
        StageShot,
        Amp,
        Stow
    }

    /**
     * An enum of possible states for the intake. Includes:
     * <p> None (Default state)
     * <p> Intake (Cancelled when a note is detected)
     * <p> Eject (Feeding the shooter)
     * <p> FastEject (For the Amp. Cancelled when a note is detected)
     * <p> FeedShooter (Unused)
     */
    public enum IntakeState {
        None,
        Intake,
        Eject,
        FastEject,
        FeedShooter,
    }

    /* MOTORS (prefix: m) */
    private final TalonFX m_IntakeNote;
    private final TalonFX m_IntakePivot;

    /* SENSORS (prefix: n) */
    private final CANcoder n_Encoder;
    private final DigitalInput n_NoteDetect;

    /* OTHER VARIABLES */
    private double d_EncoderOffset    = 0.0;
    private double d_IntakeSpeed      = 0.0;
    private double d_IntakePivotSpeed = 0.0;
    private int i_IntakeSwitchDelay = 0;
    private boolean b_HasNote = false;
    private PivotTarget e_PivotTarget = PivotTarget.None;
    private IntakeState e_IntakeState     = IntakeState.None;
    private IntakeState e_IntakeStateGOAL = IntakeState.None;

    private LED s_LED;
    public Intake(LED led) {
        s_LED = led;
        m_IntakeNote = new TalonFX(c_IntakeNoteID, "3658CANivore");
        m_IntakeNote.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakeNote.setNeutralMode(NeutralModeValue.Coast);

        m_IntakePivot = new TalonFX(c_IntakePivotID, "3658CANivore");
        m_IntakePivot.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakePivot.setNeutralMode(NeutralModeValue.Brake);

        n_Encoder = new CANcoder(c_CANCoderID, "3658CANivore");
        n_NoteDetect = new DigitalInput(9);
        s_LED.SetColor(Color.Yellow);

        d_EncoderOffset = n_Encoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * This function runs repeatedly. It handles the following intake logic:
     * <p> Sets the pivot motor speed to reach target
     * <p> Switches state automatically depending on specific conditions
     * <p> Sets LED status depending on possession of note
     */
    @Override
    public void periodic() {
        // Pivot Control
        double d_PivotAngle = pivotTargetToAngle(e_PivotTarget);
        
        // Intake Control
        if (i_IntakeSwitchDelay > 0) {  // Sometimes it is necessary to delay the intake state for power management or to ensure we have a note
            i_IntakeSwitchDelay--;
        }
        else {
            e_IntakeState = e_IntakeStateGOAL;
        }

        d_IntakeSpeed = intakeStateToSpeed(e_IntakeState);
        if (isPivotAtTarget() || i_IntakeSwitchDelay > 0) {
            m_IntakeNote.set(d_IntakeSpeed);
        }
        else {
            m_IntakeNote.set(0.0);
        }

        // Pivot control
        if (e_PivotTarget != PivotTarget.None) {
            double d_CurrentPivot = getPivotAngle();
            d_IntakePivotSpeed = Math.max(Math.min(((d_CurrentPivot - d_PivotAngle) * 1.5),0.40),-0.40);
        }
        m_IntakePivot.set(d_IntakePivotSpeed);

        // Stow on detect ground note
        if (e_PivotTarget == PivotTarget.Ground && intakeDetectsNote()) { // Automatically revert to stow state
            b_HasNote = true;
            i_IntakeSwitchDelay = 12;
            s_LED.SetColor(Color.Green);
            setStateToStow();
        }
        else if (e_IntakeState == IntakeState.Intake && intakeDetectsNote()) { // Stop manual intake
            b_HasNote = true;
            // i_IntakeSwitchDelay = 25;
            s_LED.SetColor(Color.Green);
            setIntake(IntakeState.None);
        }
        else if (e_IntakeState == IntakeState.Eject && intakeDetectsNote()) { // ShootGeneric response
            b_HasNote = false;
        }
        else if (e_IntakeState == IntakeState.FastEject && intakeDetectsNote()) { // Stop manual eject
            b_HasNote = false;
            s_LED.SetColor(Color.Yellow);
            i_IntakeSwitchDelay = 25;
            e_IntakeStateGOAL = IntakeState.None;
        }

        outputTelemetry();
    }

    public void stop() {
        d_IntakeSpeed = 0.0;
        //m_IntakePivot.setNeutralMode(NeutralModeValue.Coast);
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake - Pivot Angle",getPivotAngle()); 
        SmartDashboard.putBoolean("Intake - Pivot in place?", isPivotAtTarget());
        SmartDashboard.putString("Intake - State", e_IntakeState.name());
    }

    /**
     * Converts a PivotTarget to its corresponding CANcoder target value.
     * @param target A PivotTarget value
     * @return A double representing what value the CANcoder should have when the target is reached
     */
    public double pivotTargetToAngle(PivotTarget target) {
        switch (target) {
            case Ground:
                return 0.59;
            case StageShot:
                return 0.065;
            case Amp:
                return 0.37;
            case Stow:
            default:
                return 0.0;
        }
    }

    /**
     * Converts an IntakeState to a motor speed (double)
     * @param state An IntakeState value
     * @return A double representing the speed at which the IntakeNote motor should run in this state
     */
    public double intakeStateToSpeed(IntakeState state) {
        switch (state) {
            case Intake:
                return 0.35;
            case Eject:
                return -0.30;
            case FastEject:
                return -0.55;
            case FeedShooter:
            default:
                return 0.0;
        }
    }

    public IntakeState getIntakeState() {
        return e_IntakeState;
    }

    public PivotTarget getPivotTarget() {
        return e_PivotTarget;
    }

    /**
     * Returns the CANcoder value, modified such that 0 is the Stow state and ~0.6 is the Ground state.
     * @return A modified CANcoder value
     */
    public double getPivotAngle() {
        double raw = (n_Encoder.getAbsolutePosition().getValueAsDouble());
        if (raw > 0) {
            return 1-raw+d_EncoderOffset;
        }
        else {
            return -raw+d_EncoderOffset;
        }
    }

    /**
     * Gets whether the Intake sensor currently detects a note
     * @return If a note is detected
     */
    public boolean intakeDetectsNote() {
        return !n_NoteDetect.get();
    }

    /**
     * Gets whether the Intake currently has a note stowed
     * @return If a note is stowed
     */
    public boolean intakeHasNote() {
        return b_HasNote;
    }

    // Pivot functions
    public void setStateToGround() {
        e_PivotTarget = PivotTarget.Ground;
        e_IntakeStateGOAL = IntakeState.Intake;
    }

    public void setStateToAmp() {
        e_PivotTarget = PivotTarget.Amp;
        e_IntakeStateGOAL = IntakeState.None;
    }

    public void setStateToStow() {
        e_PivotTarget = PivotTarget.Stow;
        e_IntakeStateGOAL = IntakeState.None;
    }

    public void setPivot(PivotTarget target) {
        e_PivotTarget = target;
    }

    public void overridePivotSpeed(double speed) {
        e_PivotTarget = PivotTarget.None;
        d_IntakePivotSpeed = speed;
    }

    // Intake functions
    public void intake() {
        e_IntakeStateGOAL = IntakeState.Intake;
        i_IntakeSwitchDelay = 0;
    }

    public void eject() {
        e_IntakeStateGOAL = IntakeState.Eject;
        i_IntakeSwitchDelay = 0;
    }

    public void feedShooter() {
        e_IntakeStateGOAL = IntakeState.FeedShooter;
        i_IntakeSwitchDelay = 0;
    }

    public void stopIntake() {
        e_IntakeStateGOAL = IntakeState.None;
        d_IntakeSpeed = 0.0;
        i_IntakeSwitchDelay = 0;
    }

    public void setIntake(IntakeState state) {
        e_IntakeStateGOAL = state;
        i_IntakeSwitchDelay = 0;
    }

    // Private functions
    private boolean isPivotAtTarget() {
        return Math.abs(getPivotAngle() - pivotTargetToAngle(e_PivotTarget)) < 0.05;
    }

    /**
     * Get devices in this subsystem that the orchestra can use.
     * @return ParentDevices. This includes TalonFX, Pigeon2, and other CTRE devices.
     */
    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_IntakeNote, m_IntakePivot, n_Encoder};
        return pd;
    }

    /**
     * Get the combined output of this subsytem's motors.
     * @return The sum of motor outputs as a double.
     */
    public double pollOrchOutput() {
        return Math.abs(m_IntakeNote.get()) + Math.abs(m_IntakePivot.get());
    }
}