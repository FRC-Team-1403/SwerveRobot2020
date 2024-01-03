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



// package team1403.robot;


// import com.fasterxml.jackson.databind.deser.impl.ValueInjector;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import team1403.lib.core.CougarLibInjectedParameters;
// import team1403.lib.core.CougarRobot;
// import team1403.lib.util.CougarLogger;
// import team1403.robot.swerve.SwerveCommand;
// import team1403.robot.swerve.SwerveSubsystem;
// import team1403.robot.turret.Limelight;
// import team1403.robot.turret.Turret;
// /**
//  * The heart of the robot.
//  *
//  * <p>
//  * The bulk of the robot will be implemented through various subsystems.
//  * This class creates those subsystems and configures their behaviors.
//  *
//  * <p>
//  * This class has little to do with the runtime operation. It acts more as
//  * a factory to create the subsystems and write them together and to external
//  * controls (both human and software). Once that has happened, the controls
//  * take over and issue commands that interact with the subsystem to actually
//  * do things.
//  */
// public class CougarRobotImpl extends CougarRobot {
//   /**
//    * Constructor.
//    *
//    * @param parameters Standard framework injected parameters.
//    * @param config     Our robot's custom configuration values.
//    */
//   public CougarRobotImpl(CougarLibInjectedParameters parameters) {
//     super(parameters);
//     var logger = CougarLogger.getChildLogger(
//         parameters.getRobotLogger(), "BuiltinDevices");
//     m_swerveSubsystem = new SwerveSubsystem( parameters);
//     CameraServer.startAutomaticCapture();
//     m_autonChooser = new SendableChooser<Command>();
//     m_Turret = new Turret();
//     m_limelight = new Limelight();
//   }
//   XboxController driveController = getXboxJoystick("Driver", RobotConfig.Driver.pilotPort);
//   XboxController operatorController = getXboxJoystick("Operator", RobotConfig.Operator.pilotPort);

//   @Override
//   public void robotInit() {
//     AutoManager.getInstance().init(m_swerveSubsystem);
//     m_autonChooser.setDefaultOption("Pathplanner auto", AutoManager.getInstance().getPathplannerAuto(m_swerveSubsystem));
//     SmartDashboard.putData(m_autonChooser);
//     Limelight.turnOff();
//     super.robotInit();  
//   }

//   @Override
//   public void disabledPeriodic() {
//       Limelight.turnOff();
//   }

//   @Override
//   public Command getAutonomousCommand() {
//     CommandScheduler.getInstance().removeDefaultCommand(m_swerveSubsystem);
//     return m_autonChooser.getSelected();
//   }
//   public void teleopPeriodicDriver() {
//     if (driveController.getXButton()) {
//       m_swerveSubsystem.zeroGyroscope();
//     }
//     if (driveController.getBButton()) {
//       m_swerveSubsystem.zeroGyroscope();
//     }
//     m_swerveSubsystem.setXModeEnabled(driveController.getXButton());
//   }
//   public void teleopPeriodicOperator() {
//    if (operatorController.getAButton())
//     m_Turret.centerPoint();
//   else
//     m_Turret.setSpeed(operatorController.getLeftX());

//   }
//   @Override
//   public void teleopInit() {
//     m_swerveSubsystem.setYawGyroscopeOffset(180 - m_swerveSubsystem.getGyroscopeRotation().getDegrees());
//     m_Turret.setOffset(m_Turret.getEncoder().getTurretAngle());
//     Limelight.turnOn();
//     configureDriverInterface();
//   }
//   @Override
//   public void teleopPeriodic() {
//     teleopPeriodicDriver();
//     teleopPeriodicOperator();
//     m_limelight.teleopPeriodic();
//   }
//   /**
//    * Configures the driver commands and their bindings.
//    */
//   public void configureDriverInterface() {
//     // The controls are for field-oriented driving:
//     // Left stick Y axis -> forward and backwards movement
//     // Left stick X axis -> left and right movement
//     // Right stick X axis -> rotation
//     // Setting default command of swerve subsystem
//     m_swerveSubsystem.setDefaultCommand(new SwerveCommand(
//         m_swerveSubsystem,
//         () -> -deadband(driveController.getLeftX(), 0),
//         () -> -deadband(driveController.getLeftY(), 0),
//         () -> deadband(driveController.getRightX(), 0),
//         () -> driveController.getYButton(),
//         () -> driveController.getRightTriggerAxis()));
//   }

//   /**
//    * Configures the operator commands and their bindings.
//    */

//   /**
//    * Applies a deadband to the given value.
//    *
//    * @param value    the value to apply a deadband to
//    * @param deadband the deadband to apply to the value
//    * @return 0 if the value is < deadband,
//    *         or value if value > deadband
//    */


//   /**
//    * Get controller and silence warnings if not found.
//    *
//    * @param role The role for the port for logging purposes.
//    * @param port The expected port for the controller.
//    *
//    * @return controller for port, though might not be temporarily disconnected.
//    */
//   /**
//    * Get controller and silence warnings if not found.
//    *
//    * @param role The role for the port for logging purposes.
//    * @param port The expected port for the controller.
//    *
//    * @return controller for port, though might not be temporarily disconnected.
//    */
//   private PS4Controller getPS4Controller(String role, int port) {
//     if (!DriverStation.isJoystickConnected(port)) {
//       DriverStation.silenceJoystickConnectionWarning(true);
//       CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
//           port, role);
//     }
//     return new PS4Controller(port);
//   }
//   // private final BuiltinSubsystem m_builtins;
//   // private final PhotonVisionSubsystem m_visionSubsystem;
//   private final SwerveSubsystem m_swerveSubsystem;
//   private final SendableChooser<Command> m_autonChooser;
//   public final Limelight m_limelight;
//   private Turret m_Turret;
// }
