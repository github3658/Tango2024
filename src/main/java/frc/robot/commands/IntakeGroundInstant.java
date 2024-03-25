package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeGroundInstant extends InstantCommand {
    private final Intake s_Intake;

    public IntakeGroundInstant(Intake intake) {
        s_Intake = intake;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setStateToGround();
    }
}