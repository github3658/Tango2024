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
import frc.robot.subsystems.LED.Pattern;

public class PlaySong extends Command {
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Shooter s_Shooter;
    private final Climber s_Climber;
    private final LED s_LED;

    private final Orchestra o_Orchestra;
    private final String str_song;
    private final GenericHID xb_Operator;
    private int i_Delay;

    private final int ctrl_SongCancel = XboxController.Button.kStart.value;

    /**
     * This is the constructor for the PlaySong command. 
     * Be aware that scheduling this command will enter the robot into a vegetative state as it enjoys the music too.
     * @param orchestra The Orchestra object
     * @param swerve The Swerve Subsystem
     * @param intake The Intake Subsystem
     * @param shooter The Shooter Subsystem
     * @param climber The Climber Subsystem
     * @param song The song to play, provided as a string path
     * @param xbox The xbox controller that can disable the song
     * @param led The LED subsystem
     */
    public PlaySong(Orchestra orchestra, Swerve swerve, Intake intake, Shooter shooter, Climber climber, String song, GenericHID xbox, LED led) {
        o_Orchestra = orchestra;
        s_Swerve = swerve;
        s_Intake = intake;
        s_Shooter = shooter;
        s_Climber = climber;
        s_LED = led;
        str_song = song;
        xb_Operator = xbox;
        addRequirements(s_Swerve,s_Intake,s_Shooter,s_Climber);
    }

    /**
     * This runs when the PlaySong command initializes.
     * It prints the current song to the console and loads the provided song into the orchestra.
     * Then, it plays and creates a delay before the song can be disabled.
     * Also, the LEDS are set to a rainbow pattern.
     */
    @Override
    public void initialize() {
        System.out.println("Playing "+str_song+"!");
        o_Orchestra.loadMusic(str_song);
        o_Orchestra.play();
        i_Delay = 50;
        s_LED.SetPattern(Pattern.Rainbow);
    }

    /**
     * This function runs repeatedly while PlaySong is scheduled.
     * It converts the song output into LED brightness (in theory, still needs testing).
     * It also awaits controller input to be disabled.
     */
    @Override
    public void execute() {
        i_Delay --;
        double output = pollOrchOutput();
        s_LED.setBrightness((float) output);
        if (i_Delay < 0 && xb_Operator.getRawButton(ctrl_SongCancel)) {
            o_Orchestra.stop();
            System.out.println("Orchestra finished!");
            s_LED.setBrightness(1);
        }
        SmartDashboard.putNumber("Orchestra - Output", output);
    }

    @Override
    public boolean isFinished() {
        return !o_Orchestra.isPlaying();
    }

    /**
     * This function gets every motor output and sums it. In theory, this should correlate to the intensity of the song.
     * If not, we should try voltage instead.
     * @return The combined output of each subsystem
     */
    private double pollOrchOutput() {
        return (s_Climber.pollOrchOutput() + s_Intake.pollOrchOutput() + s_Shooter.pollOrchOutput() + s_Swerve.pollOrchOutput())/18.0;
    }
}