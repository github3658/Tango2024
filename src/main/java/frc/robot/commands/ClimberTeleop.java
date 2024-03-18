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

    private final int ctrl_Climb = XboxController.Axis.kRightTrigger.value;
    private final int ctrl_Release = XboxController.Axis.kLeftTrigger.value;
    private final int ctrl_ManualLower = XboxController.Button.kX.value;

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
        if (xb_Driver.getRawAxis(ctrl_Climb) > 0.9) {
            s_Climber.setClimbState(ClimbState.Climb);
        }
        else if (xb_Driver.getRawAxis(ctrl_Release) > 0.9) {
            s_Climber.setClimbState(ClimbState.Release);
        }
        else {
            s_Climber.setClimbState(ClimbState.Idle);
        }
        if(xb_Driver.getRawButton(ctrl_ManualLower))
        {
            s_Climber.setClimbState(ClimbState.ManualLower);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}