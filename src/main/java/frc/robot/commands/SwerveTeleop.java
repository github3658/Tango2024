// This command is the default swerve command.
// It takes input from the driver controller and applies a slight acceleration value.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class SwerveTeleop extends Command {
    private final Swerve s_Swerve;

    private final double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final double c_SwerveRampDeadzone = 0.05;
    private final double c_AccelTime = 25.0;
    private double d_SwerveRamp = 0.0;
    private boolean b_Automatic = false;
    private double d_AutoRotate = 0.0;

    private final int ctrl_Forward = XboxController.Axis.kLeftY.value;
    private final int ctrl_Strafe = XboxController.Axis.kLeftX.value;
    private final int ctrl_Rotate = XboxController.Axis.kRightX.value;
    private final int ctrl_Slow = XboxController.Button.kA.value;
    private final int ctrl_ResetFOC = XboxController.Button.kLeftStick.value;
    private final int ctrl_ToggleControl = XboxController.Button.kRightStick.value;

    private GenericHID xb_Driver;

    private final SwerveRequest.FieldCentric drive_field = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric drive_robot = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * This is the constructor for the SwerveTeleop command.
     * @param subsystem The Swerve subsystem
     * @param port0 The driver's controller
     */
    public SwerveTeleop(Swerve subsystem, GenericHID port0) {
        s_Swerve = subsystem;
        xb_Driver = new GenericHID(0);
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
    }

    /**
     * This function runs repeatedly while SwerveTeleop is scheduled and active.
     * The following inputs lead to the following actions:
     * <p> Left Stick pressed - Reset field-oriented control
     * <p> Right Stick pressed - Toggle between field-oriented and robot-oriented control
     * <p> Left Stick up/down - Forward / Backward Swerve motion
     * <p> Left Stick left/right - Left / Right Swerve motion
     * <p> Right Stick left/right - Swerve rotation
     * <p> A Button - Precision driving mode
     */
    @Override
    public void execute() {
        // Reset FOC
		if (xb_Driver.getRawButtonPressed(ctrl_ResetFOC)) {
			s_Swerve.zeroHeading();
		}

		// Toggle FOC
		if (xb_Driver.getRawButtonPressed(ctrl_ToggleControl)) {
			s_Swerve.toggleFieldOrient();
		}

        double forward;
        double strafe;
        double rotate;

        forward = -Math.pow(xb_Driver.getRawAxis(ctrl_Forward),3);
        strafe = -Math.pow(xb_Driver.getRawAxis(ctrl_Strafe),3);

        if (b_Automatic) {
            rotate = d_AutoRotate;
        }
        else {
            rotate = -Math.pow(xb_Driver.getRawAxis(ctrl_Rotate),3);
        }

        if (xb_Driver.getRawButton(ctrl_Slow)) {
            forward *= 0.35;
            strafe *= 0.35;
            rotate *= 0.35;
        }
        else {
            //forward *= 0.75;
            //strafe *= 0.75;
        }

        if (Math.abs(forward) > c_SwerveRampDeadzone || Math.abs(strafe) > c_SwerveRampDeadzone || Math.abs(rotate) > c_SwerveRampDeadzone) {
            d_SwerveRamp = Math.min(d_SwerveRamp+1/c_AccelTime,1);
        }
        else {
            d_SwerveRamp = Math.max(d_SwerveRamp-1/c_AccelTime,0);
        }

        if (s_Swerve.getFieldOrient()) {
            s_Swerve.setControl(drive_field.
                withVelocityX(forward * d_SwerveRamp * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafe * d_SwerveRamp * c_MaxSwerveSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotate * d_SwerveRamp * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
            );
        }
        else {
            s_Swerve.setControl(drive_robot.
                withVelocityX(forward * d_SwerveRamp * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafe * d_SwerveRamp * c_MaxSwerveSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotate * d_SwerveRamp * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
            );
        }
    }

    public void setRotate(double rotate) {
        d_AutoRotate = rotate;
    }

    public void setAutomatic(boolean auto) {
        b_Automatic = auto;
    }

    public boolean getAutomatic() {
        return b_Automatic;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}