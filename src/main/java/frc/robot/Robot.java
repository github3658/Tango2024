// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This file contains functions that run during the various robot states (driver-controlled, autonomous, disabled, etc.)
// This file isn't used often, only send some information to SmartDashboard.

package frc.robot;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED.Color;
import frc.robot.subsystems.LED.Pattern;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private int i_DisabledTimer = 0;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        m_robotContainer.s_LED.SetPattern(Pattern.Line);
    }

    @Override
    public void disabledPeriodic() {
        i_DisabledTimer++;
        if (i_DisabledTimer % 500 == 0) {
            int random = (int) (Math.round(Math.random()*3));
            switch (random) {
                case 0:
                    m_robotContainer.s_LED.SetColor(Color.Red);
                    m_robotContainer.s_LED.SetPattern(Pattern.Strobe);
                    break;
                case 1:
                    m_robotContainer.s_LED.SetPattern(Pattern.Alternate);
                    break;
                case 2:
                    m_robotContainer.s_LED.SetPattern(Pattern.Line);
                    break;
                case 3:
                    m_robotContainer.s_LED.SetPattern(Pattern.Dot);
                    break;
            }
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.s_LED.SetPattern(Pattern.Solid);
        m_robotContainer.s_LED.SetColor(m_robotContainer.s_Intake.intakeHasNote() ? Color.Green : Color.Yellow);
        m_robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.s_LED.SetPattern(Pattern.Solid);
        m_robotContainer.s_LED.SetColor(m_robotContainer.s_Intake.intakeHasNote() ? Color.Green : Color.Yellow);
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.teleopPeriodic();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
