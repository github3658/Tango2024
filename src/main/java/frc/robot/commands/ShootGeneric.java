// This command uses the intake and the shooter to expel notes at a given speed.
// While this command is running, the intake and the shooter are not controllable.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.LED.Color;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class ShootGeneric extends Command {
    private final Shooter s_Shooter;
    private final Intake s_Intake;
    private final LED s_LED;

    private int i_ShooterWarmupDelay = 999;
    private int i_ShutdownDelay = 999;
    private boolean b_SeenNote;
    private double d_ShooterSpeed = 0.0;

    /**
     * This is the constructor for the ShootGeneric command.
     * @param shooter The Shooter subsystem
     * @param intake The Intake subsystem
     * @param speed The speed at which to shoot
     * @param led The LED subsystem
     */
    public ShootGeneric(Shooter shooter, Intake intake, double speed, LED led) {
        s_LED = led;
        s_Shooter = shooter;
        s_Intake = intake;
        d_ShooterSpeed = speed;
        addRequirements(s_Shooter, s_Intake);
    }

    /**
     * When ShootGeneric is initialized, delays are configured and the LEDs are set to a strobe pattern.
     */
    @Override
    public void initialize() {
        i_ShooterWarmupDelay = 50;
        i_ShutdownDelay = 50;
        b_SeenNote = false;
        s_LED.SetColor(Color.Blue);
        s_LED.SetPattern(Pattern.Strobe);
        if (d_ShooterSpeed > 0.65) {
            s_Intake.setPivot(PivotTarget.StageShot);
            s_Shooter.setPivot(-9.075);
        } 
    }

    /**
     * This function runs repeatedly while ShootGeneric is scheduled.
     * It runs the shooter motors, waits for a delay, then runs the intake motors.
     * After a second delay, the command is finished.
     */
    @Override
    public void execute() {
        s_Shooter.setSpeed(d_ShooterSpeed);
        if (i_ShooterWarmupDelay > 0) {
            i_ShooterWarmupDelay--;
        }
        else {
            s_Intake.eject();
            b_SeenNote = true;
        }

        if (b_SeenNote) {
            i_ShutdownDelay--;
        }
    }

    /**
     * When ShootGeneric ends, all motors stop and the LEDs return to their normal state.
     */
    @Override
    public void end(boolean interrupted) {
        s_Shooter.setSpeed(0.0);
        s_Shooter.setPivot(0.0);
        s_Intake.setStateToStow();
        s_LED.SetColor(Color.Yellow);
        s_LED.SetPattern(Pattern.Solid);
    }

    @Override
    public boolean isFinished() {
        return i_ShutdownDelay < 0;
    }
}