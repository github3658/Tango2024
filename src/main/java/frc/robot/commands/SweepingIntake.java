package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;

public class SweepingIntake extends Command {
    private final Swerve s_Swerve;
    private final Intake s_Intake;

    private final double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps;

    private double d_Safety;
    private double d_Ramp;
    private int failsafe;

    private final SwerveRequest.FieldCentric drive_field = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public SweepingIntake(Swerve swerve, Intake intake) {
        s_Swerve = swerve;
        s_Intake = intake;
        addRequirements(s_Swerve, s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setStateToGround();
        failsafe = 250;
        d_Safety = 1.0;
        d_Ramp = 0.05;
        System.out.println("Called SweepingIntake");
    }

    public void execute() {
        s_Swerve.setControl(drive_field
            .withVelocityX(0.5 * c_MaxSwerveSpeed * d_Safety * d_Ramp) // Drive forward with negative Y (forward)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
        if (!s_Intake.intakeHasNote()) {
            d_Ramp = Math.min(d_Ramp+0.05, 1);
        }
        else {
            d_Ramp = Math.max(d_Ramp-0.05,0);
        }
        if (failsafe < 210) {
            d_Safety = 0.5;
        }
        if (failsafe < 0) {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            s_Intake.setStateToStow();
        }
    }

    @Override
    public boolean isFinished() {
        failsafe--;
        return d_Ramp == 0;
    }
}