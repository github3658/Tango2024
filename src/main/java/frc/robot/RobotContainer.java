// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the primary file. We reference our subsystems here, give them commands, and that's it.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.LED.Color;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Telemetry;
//import frc.robot.LimelightHelpers;

//TODO: For the limelight, set the RIO static ip configuration: (IP 10.36.58.2, MASK 255.255.255.0, GATEWAY 10.36.58.1)
//TODO: For the limelight, set the Driver Station static ip configuration: (IP 10.36.58.5, MASK 255.0.0.0, GATEWAY 10.36.58.1)

public class RobotContainer {

	/* SUBSYSTEM DEFINITIONS (prefix: s) */
	public final Swerve   s_Swerve   = TunerConstants.DriveTrain;
	public final LED 	  s_LED      = new LED();
	public final Shooter  s_Shooter  = new Shooter(s_LED);
	public final Intake   s_Intake   = new Intake(s_LED);
	public final Climber  s_Climber  = new Climber(s_LED);

	/* INPUT DEVICES (prefix: xb) */
	private final GenericHID xb_Driver   = new GenericHID(0);
	private final GenericHID xb_Operator = new GenericHID(1);

	/* CONTROLS (prefix: ctrl) */
	private final int ctrl_ShooterMain = XboxController.Axis.kRightTrigger.value;
	private final int ctrl_ShooterAmp  = XboxController.Axis.kLeftTrigger.value;
	private final int ctrl_SongSelect  = XboxController.Button.kStart.value;
	private final int ctrl_VisionAlign = XboxController.Button.kB.value;
	private final int ctrl_StageShot   = XboxController.Button.kY.value;

	/* OTHER VARIABLES */
	private final Orchestra o_Orchestra = new Orchestra();
	private boolean b_Shot     = false;
	private boolean b_PlaySong = true;
	//private SendableChooser<Command> m_AutoChooser;
  	private final Telemetry logger = new Telemetry();

	/* COMMAND DEFINITIONS (prefix: com) */
	private final SwerveTeleop com_SwerveTeleop = new SwerveTeleop(s_Swerve, xb_Driver);

	/**
	 * This function sets the default commands for each subsystem.
	 * When a subsystem is not scheduled to do anything else, it will default to the command it is given here.
	 */
  	private void setDefaultSubsystemCommands() {
		// These commands contain isolated subsystem behavior
		s_Swerve.setDefaultCommand(com_SwerveTeleop);
		s_Intake.setDefaultCommand( new IntakeTeleop( s_Intake,	 xb_Operator));
		s_Shooter.setDefaultCommand(new ShooterTeleop(s_Shooter, xb_Operator));
		s_Climber.setDefaultCommand(new ClimberTeleop(s_Climber, xb_Driver));
		// More complex behaviors are handled in TeleopPeriodic.
  	}

	/**
     * This function provides PathPlannerLib with "named commands".
     * These named commands can be called from PathPlanner's autonomous programs.
     */
	private void createNamedCommands() {
		//NamedCommands.registerCommand("MetalCrusher",new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, null, "metalcrusher.chrp", xb_Operator, s_LED));
		NamedCommands.registerCommand("ShootSpeaker",new ShootGeneric(s_Shooter, s_Intake, 1.0, s_LED));
	}

	/**
     * This is the constructor for the robot container.
	 * It provides the orchestra with all compatible devices,
	 * sets each subsystem's default commands,
	 * creates named commands for PathPlanner,
	 * and defaults the LED array.
     */
  	public RobotContainer() {
		// Init orchestra
		for (ParentDevice pd : s_Climber.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Intake.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Shooter.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Swerve.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
    	setDefaultSubsystemCommands();

		createNamedCommands();

		s_LED.SetColor(Color.White);
		s_LED.SetPattern(Pattern.Line);
  	}

	/**
	 * This function is not properly implemented currently.
	 * @return Currently, an autonomous command that does nothing.
	 */
  	public Command getAutonomousCommand() {
    	return Commands.print("No autonomous command configured");
  	}

	/**
	 * This function runs when autonomous mode begins.
	 */
	public void autonomousInit() {
		String msg = SmartDashboard.getString("DB/String 0","3");
		// 3 NOTE AUTON
		if (msg.contains("3")) {
			new SequentialCommandGroup(
				new ZeroBotPose(s_Swerve),
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new IntakeGroundInstant(s_Intake),
				new GoToPose(s_Swerve, GoToPose.Note2Pickup),
				new SweepingIntake(s_Swerve, s_Intake),
				new GoToPose(s_Swerve, GoToPose.Origin),
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new ZeroBotPose(s_Swerve),
				new IntakeGroundInstant(s_Intake),
				new GoToPose(s_Swerve, GoToPose.Note1Pickup),
				new SweepingIntake(s_Swerve, s_Intake),
				new GoToPose(s_Swerve, GoToPose.Origin),
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new ZeroBotPose(s_Swerve),
				new GoToPose(s_Swerve, GoToPose.TaxiLeftWall)
			).schedule();
		}

		// 2 NOTE
		else if (msg.contains("2")) {
			new SequentialCommandGroup(
				new ZeroBotPose(s_Swerve),
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new IntakeGroundInstant(s_Intake),
				new GoToPose(s_Swerve, GoToPose.Note2Pickup),
				new SweepingIntake(s_Swerve, s_Intake),
				new GoToPose(s_Swerve, GoToPose.Origin),
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new ZeroBotPose(s_Swerve),
				new GoToPose(s_Swerve, GoToPose.TaxiLeftWallAvoidLeft)
				//new GoToPose(s_Swerve, GoToPose.TaxiLeftWall)
			).schedule();
		}

		// RIGHT SIDE 1 NOTE
		else if (msg.contains("1")) {
			new SequentialCommandGroup(
				new ShootGeneric(s_Shooter, s_Intake, 0.6, s_LED),
				new DriveForwardWorkaround(s_Swerve)
			).schedule();
		}
	}

	/**
	 * This function runs repeatedly while the drivers have control over the robot.
	 * More complex behaviors that involve commands or multiple subsystems are scheduled here.
	 * This includes, but is not limited to: shooting for the speaker/amp and playing music.
	 */
	public void teleopPeriodic() {
		logger.telemeterize(s_Swerve.getState());

		// Song Selection
		if (!b_PlaySong && xb_Driver.getRawButton(ctrl_SongSelect)) {
			switch (xb_Driver.getPOV()) {
				case 0: // UP
					ScheduleSong("bohemianrhapsody.chrp");
					break;
				case 45: // UP RIGHT
					ScheduleSong("lacrimosa.chrp");
					break;
				case 90: // RIGHT
					ScheduleSong("rickroll.chrp");
					break;
				case 135: // DOWN RIGHT
					ScheduleSong("beethoven.chrp");
					break;
				case 180: // DOWN
					ScheduleSong("starwars.chrp");
					break;
				case 225: // DOWN LEFT
					ScheduleSong("megalovania.chrp");
					break;
				case 270: // LEFT
					ScheduleSong("metalcrusher.chrp");
					break;
				case 315: // UP LEFT
					ScheduleSong("king.chrp");
					break;
				default:
					break;
			}
		}
		else if (xb_Driver.getRawButton(ctrl_SongSelect) == false) {
			b_PlaySong = false;
		}

		// Shoot for the Speaker
		if (xb_Operator.getRawButtonPressed(ctrl_StageShot) && !b_Shot) { // Special shot from the protected zone on the Stage.
			b_Shot = true;
			new ShootGeneric(s_Shooter, s_Intake, 0.80, s_LED).schedule();
		}
		else if (xb_Operator.getRawAxis(ctrl_ShooterMain) > 0.9 && !b_Shot) { // Shot from the base of the Speaker.
			b_Shot = true;
			new ShootGeneric(s_Shooter, s_Intake, 0.60, s_LED).schedule();
		}
		else if (xb_Operator.getRawAxis(ctrl_ShooterMain) < 0.1 && b_Shot) {
			b_Shot = false;
		}

		// Shoot for the Amp
		if (xb_Operator.getRawAxis(ctrl_ShooterAmp) > 0.9 && !b_Shot) {  // Shot for the Amp
			b_Shot = true;
			new ShootGeneric(s_Shooter, s_Intake, 0.13, s_LED).schedule();
		}
		else if (xb_Operator.getRawAxis(ctrl_ShooterAmp) < 0.1 && b_Shot) {
			b_Shot = false;
		}

		// Vision Alignment
		// if (xb_Driver.getRawButton(ctrl_VisionAlign)) {
		// 	if (LimelightHelpers.getFiducialID("") != -1) { // Found the AprilTag
		// 		com_SwerveTeleop.setAutomatic(true);
		// 		double calculated = Math.min(Math.max(-(LimelightHelpers.getTX("")-4.3)*0.05,-1),1);
		// 		com_SwerveTeleop.setRotate(calculated);
		// 	}
		// 	else { // Failed to find the AprilTag!
		// 		s_LED.SetColor(Color.Red);
		// 	}
		// }
		// else if (com_SwerveTeleop.getAutomatic()) {
		// 	com_SwerveTeleop.setAutomatic(false);
		// 	s_LED.SetColor(s_Intake.intakeHasNote() ? Color.Green : Color.Yellow);
		// }
	}

	/**
	 * This schedules the PlaySong command. The only thing you have to worry about is the path to the song.
	 * @param song The String path to the song
	 */
	private void ScheduleSong(String song) {
		b_PlaySong = true;
		new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, s_Climber, song, xb_Driver, s_LED).schedule();
	}
}
