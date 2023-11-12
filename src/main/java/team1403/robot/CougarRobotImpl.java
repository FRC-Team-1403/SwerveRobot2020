package team1403.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.util.CougarLogger;
import team1403.robot.swerve.SwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.RobotConfig.Operator;

/**
 * The heart of the robot.
 *
 * <p>
 * The bulk of the robot will be implemented through various subsystems.
 * This class creates those subsystems and configures their behaviors.
 *
 * <p>
 * This class has little to do with the runtime operation. It acts more as
 * a factory to create the subsystems and write them together and to external
 * controls (both human and software). Once that has happened, the controls
 * take over and issue commands that interact with the subsystem to actually
 * do things.
 */
public class CougarRobotImpl extends CougarRobot {
  /**
   * Constructor.
   *
   * @param parameters Standard framework injected parameters.
   * @param config     Our robot's custom configuration values.
   */
  public CougarRobotImpl(CougarLibInjectedParameters parameters) {
    super(parameters);
    var logger = CougarLogger.getChildLogger(
        parameters.getRobotLogger(), "BuiltinDevices");
    m_swerveSubsystem = new SwerveSubsystem( parameters);
    CameraServer.startAutomaticCapture();
    m_autonChooser = new SendableChooser<Command>();
  }

  @Override
  public void robotInit() {
    AutoManager.getInstance().init(m_swerveSubsystem);
    m_autonChooser.setDefaultOption("Pathplanner auto", AutoManager.getInstance().getPathplannerAuto(m_swerveSubsystem));
    SmartDashboard.putData(m_autonChooser);
    super.robotInit();  
  }

  @Override
  public Command getAutonomousCommand() {
    CommandScheduler.getInstance().removeDefaultCommand(m_swerveSubsystem);
    return m_autonChooser.getSelected();
  }

  @Override
  public void teleopInit() {
    m_swerveSubsystem.setYawGyroscopeOffset(180 - m_swerveSubsystem.getGyroscopeRotation().getDegrees());
    configureOperatorInterface();
    configureDriverInterface();
  }

  /**
   * Configures the driver commands and their bindings.
   */
  public void configureDriverInterface() {
    XboxController driveController = getXboxJoystick("Driver", RobotConfig.Driver.pilotPort);
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subsystem
    m_swerveSubsystem.setDefaultCommand(new SwerveCommand(
        m_swerveSubsystem,
        () -> -deadband(driveController.getLeftX(), 0),
        () -> -deadband(driveController.getLeftY(), 0),
        () -> -deadband(driveController.getRightX(), 0),
        () -> driveController.getYButton(),
        () -> driveController.getRightTriggerAxis()));

    new Trigger(() -> driveController.getBButton()).onFalse(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyroscope()));

    new Trigger(() -> driveController.getXButton())
        .onTrue(new InstantCommand(() -> m_swerveSubsystem.setXModeEnabled(true)));
    new Trigger(() -> driveController.getXButton())
        .onFalse(new InstantCommand(() -> m_swerveSubsystem.setXModeEnabled(false)));
  }

  /**
   * Configures the operator commands and their bindings.
   */
  public void configureOperatorInterface() {
    XboxController xboxOperator = getXboxJoystick("Operator", Operator.pilotPort);
    //left empty due to no arm
  }

  /**
   * Applies a deadband to the given value.
   *
   * @param value    the value to apply a deadband to
   * @param deadband the deadband to apply to the value
   * @return 0 if the value is < deadband,
   *         or value if value > deadband
   */
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

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private XboxController getXboxJoystick(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
          port, role);
    }
    return new XboxController(port);
  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private PS4Controller getPS4Controller(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
          port, role);
    }
    return new PS4Controller(port);
  }
  public void simulationPeriodic() {
   //heart of the teleop code

  }
  // private final BuiltinSubsystem m_builtins;
  // private final PhotonVisionSubsystem m_visionSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  private final SendableChooser<Command> m_autonChooser;
  // private final LightSubsystem m_lightSubsystem;
}
