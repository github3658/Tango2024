// This is the swerve subsystem. I didn't write this code, and you shouldn't have to look through this file either.
// Most swerve functionality is handled through other things.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SwerveRequest.ApplyChassisSpeeds m_AutoDriveRequest = new SwerveRequest.ApplyChassisSpeeds();
    private boolean b_FieldOriented = true;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {    
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        for (int i = 0; i < 4; i++) {
            SwerveModule s = this.getModule(i);
            s.getDriveMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
            s.getSteerMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
        }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        for (int i = 0; i < 4; i++) {
            SwerveModule s = this.getModule(i);
            s.getDriveMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
            s.getSteerMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
        }
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(0.3, 0, 0),
                new PIDConstants(0.3, 0, 0.02),
                1.0,
                .350,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // Return current robot pose as Pose2d
    private Pose2d getPose() {
        //return m_odometry.getEstimatedPosition();

        Translation2d t_Translation = getState().Pose.getTranslation();
        return new Pose2d(new Translation2d(t_Translation.getX(),-t_Translation.getY()),Rotation2d.fromDegrees(getPigeon2().getAngle()+90));//Rotation2d.fromDegrees(getPigeon2().getYaw().getValue()+90));
        
        //return new Pose2d(getState().Pose.getTranslation(),Rotation2d.fromDegrees(getPigeon2().getAngle()));
       
        //return getState().Pose;
    }

    // Reset robot odometry to Pose2d
    private void resetPose(Pose2d pose) {
        m_odometry.resetPosition(Rotation2d.fromDegrees(getPigeon2().getYaw().getValue()), getModulePositions(), pose);
        //seedFieldRelative(pose);
    }

    // Returns current robot-relative ChassisSpeeds.
    private ChassisSpeeds getCurrentSpeeds() {
        return getState().speeds;
    }

    // Outputs module states using ChassisSpeeds.
    private void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(m_AutoDriveRequest.withSpeeds(speeds));
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = getModule(i).getPosition(false);
        }
        return positions;
    }

    public void zeroHeading() {
        m_odometry.resetPosition(Rotation2d.fromDegrees(getPigeon2().getAngle()), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        //m_odometry.resetPosition(Rotation2d.fromDegrees(getPigeon2().getAngle()), getModulePositions(), new Pose2d(new Translation2d(0.0,0.0), new Rotation2d()));
    }

    public void zeroPose() {
        m_odometry.resetPosition(Rotation2d.fromDegrees(getPigeon2().getAngle()), getModulePositions(), new Pose2d(new Translation2d(0.0,0.0), new Rotation2d()));
    }

    public void toggleFieldOrient() {
        b_FieldOriented = !b_FieldOriented;
    }

    public boolean getFieldOrient() {
        return b_FieldOriented;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {
            this.getModule(0).getDriveMotor(),this.getModule(0).getSteerMotor(),this.getModule(0).getCANcoder(),
            this.getModule(1).getDriveMotor(),this.getModule(1).getSteerMotor(),this.getModule(1).getCANcoder(),
            this.getModule(2).getDriveMotor(),this.getModule(2).getSteerMotor(),this.getModule(2).getCANcoder(),
            this.getModule(3).getDriveMotor(),this.getModule(3).getSteerMotor(),this.getModule(3).getCANcoder(),
            this.getPigeon2()
        };
        return pd;
    }

    public double pollOrchOutput() {
        return Math.abs(this.getModule(0).getDriveMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(0).getSteerMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(1).getDriveMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(1).getSteerMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(2).getDriveMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(2).getSteerMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(3).getDriveMotor().getRotorVelocity().getValueAsDouble())
        + Math.abs(this.getModule(3).getSteerMotor().getRotorVelocity().getValueAsDouble());
    }
}
