package team1403.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.lib.util.CougarLogger;
import team1403.robot.swerve.SwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.turret.Limelight;
import team1403.robot.turret.Turret;

import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
  private SwerveSubsystem m_swerveSubsystem;
  SendableChooser<Command> m_autonChooser = new SendableChooser<Command>();
  Turret m_Turret = new Turret();
  Limelight m_limelight = new Limelight();
    XboxController driveController = getXboxJoystick("Driver", Constants.Driver.pilotPort);
  XboxController operatorController = getXboxJoystick("Operator", Constants.Operator.pilotPort);


  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    CameraServer.startAutomaticCapture();
    configureButtonBindings();
  }

  public Limelight getLimelight()
  {
    return m_limelight;
  }

  public Command getAutonCommand()
  {
    return m_autonChooser.getSelected();
  }

  public void initAuto()
  {
    m_swerveSubsystem.setSpeedLimiter(1.0);
  }

  public void updateAuton()
  {
    m_swerveSubsystem.setSpeedLimiter(1.0);
  }


  private void configureButtonBindings() {
    configureDriverInterface();
  }

  public void configureDriverInterface() {
    //     // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        // Setting default command of swerve subsystem
        m_swerveSubsystem.setDefaultCommand(new SwerveCommand(
            m_swerveSubsystem,
            () -> -deadband(driveController.getLeftX(), 0),
            () -> -deadband(driveController.getLeftY(), 0),
            () -> deadband(driveController.getRightX(), 0),
            () -> driveController.getYButton(),
            () -> driveController.getRightTriggerAxis()));

            new Trigger(() -> driveController.getAButton())
            .onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyroscope(), m_swerveSubsystem)); 

            new Trigger(() -> driveController.getXButton())
            .whileTrue(new InstantCommand(() -> m_swerveSubsystem.setXModeEnabled(true), m_swerveSubsystem));

            new Trigger(() -> driveController.getBButton())
            .onTrue(new InstantCommand(() -> m_Turret.centerPoint(), m_Turret));
            new Trigger(() -> driveController.getPOV() == 270)
            .onTrue(new InstantCommand(() -> m_Turret.setSpeed(0.2), m_Turret))
            .onFalse(new InstantCommand(() -> m_Turret.setSpeed(0), m_Turret));
            new Trigger(() -> driveController.getPOV() == 90)
            .onTrue(new InstantCommand(() -> m_Turret.setSpeed(-0.2), m_Turret))
            .onFalse(new InstantCommand(() -> m_Turret.setSpeed(0), m_Turret));
            

      }
      private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
      private XboxController getXboxJoystick(String role, int port) {
        if (!DriverStation.isJoystickConnected(port)) {
          DriverStation.silenceJoystickConnectionWarning(true);
          CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
              port, role);
        }
        return new XboxController(port);
      }
  }