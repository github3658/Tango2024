package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LED.Color;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.subsystems.Swerve;

public class GoToPose extends Command {
    private final Swerve s_Swerve;

    /* PATHPLANNER CONSTANTS (prefix: c) */
	//private static final Pose2d c_SpeakerRightOffset = new Pose2d(new Translation2d(-1.277, -3.043), Rotation2d.fromDegrees(-60.249));
	//private static final Pose2d c_SpeakerLeftOffset = new Pose2d(new Translation2d(2.008, 2.461), Rotation2d.fromDegrees(64.938));
	public static final Pose2d Origin = new Pose2d(new Translation2d(0.0,0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d Note1Pickup = new Pose2d(new Translation2d(1.5,3.65), Rotation2d.fromDegrees(0.0));
	public static final Pose2d Note2Pickup = new Pose2d(new Translation2d(1.5,0.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d Note3Pickup = new Pose2d(new Translation2d(1.5,-3.5), Rotation2d.fromDegrees(0.0));
    public static final Pose2d TaxiLeftWall = new Pose2d(new Translation2d(7.0,4.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d TaxiLeftWallAvoidLeft = new Pose2d(new Translation2d(7.0,0.0), Rotation2d.fromDegrees(0.0));

    private final double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final double c_AccelTime = 25.0;
    private final double c_SwerveRampDeadzone = 0.1;
    private double d_SwerveRamp;
    private int i_frames;
    private Pose2d p_TargetPose;

    double forward;
    double strafe;
    double rotate;

    private final SwerveRequest.FieldCentric drive_field = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public GoToPose(Swerve subsystem, Pose2d target) {
        s_Swerve = subsystem;
        p_TargetPose = target;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        d_SwerveRamp = 0.0;
        i_frames = 0;
        System.out.println("Called GoToPose");
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            p_TargetPose = new Pose2d(new Translation2d(p_TargetPose.getX(),-p_TargetPose.getY()),p_TargetPose.getRotation().times(-1));
        }
    }

    @Override
    public void execute() {

        Pose2d currentPose = s_Swerve.getState().Pose;

        forward = toReasonableValue(p_TargetPose.getX() - currentPose.getX());
        strafe = toReasonableValue(p_TargetPose.getY() - currentPose.getY());
        rotate = 0;

        SmartDashboard.putNumber("AUTON RAW DIST",p_TargetPose.getX() - currentPose.getX());
        SmartDashboard.putNumber("AUTON FORWARD",forward);
        SmartDashboard.putNumber("AUTON STRAFE",strafe);

        if (Math.abs(forward) > c_SwerveRampDeadzone || Math.abs(strafe) > c_SwerveRampDeadzone || Math.abs(rotate) > c_SwerveRampDeadzone) {
            d_SwerveRamp = Math.min(d_SwerveRamp+1/c_AccelTime,1);
        }
        else {
            d_SwerveRamp = Math.max(d_SwerveRamp-1/c_AccelTime,0);
        }

        s_Swerve.setControl(drive_field.
            withVelocityX(forward * d_SwerveRamp * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(strafe * d_SwerveRamp * c_MaxSwerveSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotate * d_SwerveRamp * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        i_frames++;
        return (Math.abs(forward) < c_SwerveRampDeadzone && Math.abs(strafe) < c_SwerveRampDeadzone && i_frames > 5);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("REACHED DESTINATION");
    }

    // This is a bad function name.
    // It converts a distance to a reasonable forward/strafe speed.
    private double toReasonableValue(double dist) {
        return Math.min(Math.max(dist/4,-0.5),0.5);
    }
}