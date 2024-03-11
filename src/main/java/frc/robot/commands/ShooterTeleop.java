// This command is the default shooter command.
// Its extension and pivot is handled in here, similarly to the Intake.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTeleop extends Command {
    private final Shooter s_Shooter;

    private GenericHID xb_Operator;

    private final int ctrl_ShooterExtend = XboxController.Button.kLeftBumper.value;
    private final int ctrl_ShooterRetract = XboxController.Button.kRightBumper.value;
    private final int ctrl_ShooterPivot = XboxController.Axis.kRightY.value;

    //private final int ctrl_Operator_ButtonB = XboxController.Button.kB.value;

    /**
     * This is the constructor for the ShooterTeleop command.
     * @param subsystem The Shooter subsystem
     * @param port1 The operator controller
     */
    public ShooterTeleop(Shooter subsystem, GenericHID port1) {
        s_Shooter = subsystem;
        xb_Operator = port1;
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
    }

    /**
     * This function runs repeatedly while ShooterTeleop is scheduled and active.
     * <p> The operator's bumpers are used to move the shooter extension.
     * <p> The operator's right stick (up/down) is used to adjust the pivot.
     */
    @Override
    public void execute() {
        if (xb_Operator.getRawButton(ctrl_ShooterExtend)) {
            s_Shooter.setExtension(0.5,true);
        }
        if (xb_Operator.getRawButton(ctrl_ShooterRetract)) {
            s_Shooter.setExtension(-0.5,true);
        }
        s_Shooter.setPivot(-xb_Operator.getRawAxis(ctrl_ShooterPivot)*0.5,true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}