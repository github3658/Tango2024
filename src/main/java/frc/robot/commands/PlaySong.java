// This command is a wrapper for some orchestra functions.
// It relieves the other subsystems of their duties to safely play music.
// It also monitors the power supplied to music devices to activate LEDs

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class PlaySong extends Command {
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Shooter s_Shooter;
    private final Climber s_Climber;

    private final Orchestra o_Orchestra;
    private final String str_song;
    private final GenericHID xb_Operator;
    private int i_Delay;

    public PlaySong(Orchestra orchestra, Swerve swerve, Intake intake, Shooter shooter, Climber climber, String song, GenericHID xbox) {
        o_Orchestra = orchestra;
        s_Swerve = swerve;
        s_Intake = intake;
        s_Shooter = shooter;
        s_Climber = climber;
        str_song = song;
        xb_Operator = xbox;
        addRequirements(s_Swerve,s_Intake,s_Shooter,s_Climber);
    }

    @Override
    public void initialize() {
        System.out.println("Playing "+str_song+"!");
        o_Orchestra.loadMusic(str_song);
        o_Orchestra.play();
        i_Delay = 50;
    }

    @Override
    public void execute() {
        i_Delay --;
        if (i_Delay < 0 && xb_Operator.getRawButton(XboxController.Button.kStart.value)) {
            o_Orchestra.stop();
            System.out.println("Orchestra finished!");
        }
        SmartDashboard.putNumber("Orchestra Get", 1);
    }

    @Override
    public boolean isFinished() {
        return !o_Orchestra.isPlaying();
    }
}