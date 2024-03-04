// This command is the default shooter command.
// It currently doesn't do anything as everything the shooter does also requires the Intake.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTeleop extends Command {
    private final Shooter s_Shooter;

    private GenericHID xb_Operator;

    //private final int ctrl_Operator_ButtonB = XboxController.Button.kB.value;

    public ShooterTeleop(Shooter subsystem, GenericHID port1) {
        s_Shooter = subsystem;
        xb_Operator = port1;
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xb_Operator.getRawButton(XboxController.Button.kLeftBumper.value)) {
            s_Shooter.setExtension(0.5,true);
        }
        if (xb_Operator.getRawButton(XboxController.Button.kRightBumper.value)) {
            s_Shooter.setExtension(-0.5,true);
        }
        s_Shooter.setPivot(-xb_Operator.getRawAxis(XboxController.Axis.kRightY.value)*0.5,true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}