// This command is the default Climber command.
// It accepts DRIVER input and converts that to the climber state.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimbState;

public class ClimberTeleop extends Command {
    private final Climber s_Climber;

    private GenericHID xb_Driver;

    private final int ctrl_ClimbLeft = XboxController.Axis.kLeftTrigger.value;
    private final int ctrl_ReleaseLeft = XboxController.Button.kLeftBumper.value;
    private final int ctrl_ClimbRight = XboxController.Axis.kRightTrigger.value;
    private final int ctrl_ReleaseRight = XboxController.Button.kRightBumper.value;
    private final int ctrl_ManualLowerLeft = XboxController.Button.kX.value;
    private final int ctrl_ManualLowerRight = XboxController.Button.kY.value;

    /**
     * This is the constructor for the ClimberTeleop command.
     * @param subsystem The Climber Subsystem
     * @param port0 The driver controller
     */
    public ClimberTeleop(Climber subsystem, GenericHID port0) {
        s_Climber = subsystem;
        xb_Driver = port0;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
    }

    /**
     * This function runs repeatedly while ClimberTeleop is scheduled and active.
     * It gets the driver's trigger inputs and raises and lowers the climber from that.
     * If the robot ends in a position where the climber does not end in its fully-down state, 
     * the driver can hold the X button to lower the climber manually. 
     * Be aware this does not check the motors' encoder values!
     */
    @Override
    public void execute() {
        // This converts inputs for each climbing motor into three possible states:
        // -1   Release
        //  0   Idle
        //  1   Climb
        int i_LeftState = 0 + ((xb_Driver.getRawAxis(ctrl_ClimbLeft) > 0.9) ? 1 : 0) + (xb_Driver.getRawButton(ctrl_ReleaseLeft) ? -1 : 0);
        int i_RightState = 0 + ((xb_Driver.getRawAxis(ctrl_ClimbRight) > 0.9) ? 1 : 0) + (xb_Driver.getRawButton(ctrl_ReleaseRight) ? -1 : 0);
        switch(i_LeftState) {
            case  1: s_Climber.setLeftState(ClimbState.Climb); break;
            case  0: s_Climber.setLeftState(ClimbState.Idle); break;
            case -1: s_Climber.setLeftState(ClimbState.Release); break;
        }
        switch(i_RightState) {
            case  1: s_Climber.setRightState(ClimbState.Climb); break;
            case  0: s_Climber.setRightState(ClimbState.Idle); break;
            case -1: s_Climber.setRightState(ClimbState.Release); break;
        }
        if (xb_Driver.getRawButton(ctrl_ManualLowerLeft)) {
            s_Climber.setLeftState(ClimbState.ManualLower);
        }
        if (xb_Driver.getRawButton(ctrl_ManualLowerRight)) {
            s_Climber.setRightState(ClimbState.ManualLower);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}