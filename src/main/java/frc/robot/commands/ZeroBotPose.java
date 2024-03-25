package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ZeroBotPose extends InstantCommand {
    private Swerve s_Swerve;
    public ZeroBotPose(Swerve subsystem) {
        s_Swerve = subsystem;
    }

    public void initialize() {
        s_Swerve.zeroHeading();
    }
}
