// This command is the default Intake command.
// It accepts operator input and modifies the Intake state from that.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class IntakeTeleop extends Command {
    private final Intake s_Intake;

    private GenericHID xb_Operator;

    //private final int ctrl_IntakeMain = XboxController.Button.kLeftBumper.value;
    private final int ctrl_Intake = XboxController.Button.kA.value;
    private final int ctrl_Eject = XboxController.Button.kB.value;
    private final int ctrl_Stop = XboxController.Button.kY.value;

    public IntakeTeleop(Intake subsystem, GenericHID port1) {
        s_Intake = subsystem;
        xb_Operator = port1;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //if (xb_Operator.getRawButton(ctrl_IntakeMain)) {
            // Up - Ground State
            if (xb_Operator.getPOV() == 0 && s_Intake.getPivotTarget() != PivotTarget.Ground) {
                s_Intake.setStateToGround();
            }
            // Right - Amp State
            else if (xb_Operator.getPOV() == 90 && s_Intake.getPivotTarget() != PivotTarget.Amp) {
                s_Intake.setStateToAmp();
            }
            // Down - Stow State
            else if (xb_Operator.getPOV() == 180 && s_Intake.getPivotTarget() != PivotTarget.Stow) {
                s_Intake.setStateToStow();
            }

            // A - Force Intake
            if (xb_Operator.getRawButtonPressed(ctrl_Intake)) {
                s_Intake.setIntake(IntakeState.Intake);
            }
            // B - Force Eject
            if (xb_Operator.getRawButtonPressed(ctrl_Eject)) {
                s_Intake.setIntake(IntakeState.FastEject);
            }
            // Y - Force Stop
            if (xb_Operator.getRawButtonPressed(ctrl_Stop)) {
                s_Intake.setIntake(IntakeState.None);
            }
        //}

        // if (xb_Operator.getRawButtonPressed(XboxController.Button.kStart.value)) {
        //     s_Intake.resetOffset();
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}