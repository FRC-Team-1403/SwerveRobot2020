package team1403.robot.__replaceme__;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.util.CougarLogger;

/**
 * The heart of the robot.
 *
 * <p>The bulk of the robot will be implemented through various subsystems.
 * This class creates those subsystems and configures their behaviors.
 *
 * <p>This class has little to do with the runtime operation. It acts more as
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
   * @param config Our robot's custom configuration values.
   */
  public CougarRobotImpl(CougarLibInjectedParameters parameters,
                         RobotConfig config) {
    super(parameters);

    m_autoCommand = new InstantCommand(() -> {
      CougarLogger.getAlwaysOn().errorf("AutoCommand was never set");
    });
 
    // Create subsystems
    // ...

    configureOperatorInterface(config.operator);
  }

  /**
   * Configures the operator commands and their bindings.
   */
  private void configureOperatorInterface(
      RobotConfig.OperatorConfig config) {
    XboxController xboxDriver = getJoystick("Driver", config.driverPort);

    var doNothing = new InstantCommand(() -> {
      CougarLogger.getAlwaysOn().errorf("Example Command");
    });
    new JoystickButton(xboxDriver, Button.kX.value).whenPressed(doNothing);
  }

  /**
   * Provides the command used for autonomous.
   *
   * <p>This is special because it does not have a trigger.
   * This function allows the Robot to get it when it enters AUTONOMOUS mode.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private XboxController getJoystick(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
                                          port, role);
    }
    return new XboxController(port);
  }

  private final Command m_autoCommand;
}
