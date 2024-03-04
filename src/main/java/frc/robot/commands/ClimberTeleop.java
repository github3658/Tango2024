// This command is the default Climber command.
// It accepts DRIVER input and converts that to the climber state.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberTeleop extends Command {
    private final Climber s_Climber;

    private GenericHID xb_Driver;

    private final int ctrl_Climb = XboxController.Axis.kRightTrigger.value;
    private final int ctrl_Release = XboxController.Axis.kLeftTrigger.value;

    public ClimberTeleop(Climber subsystem, GenericHID port0) {
        s_Climber = subsystem;
        xb_Driver = port0;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xb_Driver.getRawAxis(ctrl_Climb) > 0.9) {
            s_Climber.climb();
        }
        else if (xb_Driver.getRawAxis(ctrl_Release) > 0.9) {
            s_Climber.release();
        }
        else {
            s_Climber.stopClimb();
        }
        if(xb_Driver.getRawButton(XboxController.Button.kX.value))
        {
            s_Climber.setMotorSpeed(1.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}