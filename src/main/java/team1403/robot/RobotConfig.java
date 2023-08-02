package team1403.robot;

/**
 * This class holds attributes for the robot configuration.
 *
 * <p>The RobotConfig is broken out into different areas,
 * each of which is captured in a class for that area. Each
 * subsystem has its own independent config.
 *
 * <p>The "electrical" configs are treated separate and independent
 * to make it easier to see how the robot should be wired and see
 * any conflicts since these ports specify their config together.
 */
public final class RobotConfig {
  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public static class CanBus {
    // Add CAN bus device assignments here.
    // Names should be <subsystem>Device for readability
    // e.g. drivetrainFrontLeftMotor = 1
  }

  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {
    // Add RIO port assignments here.
    // Names should be <subsystem>Device for readability
    // e.g. elevatorBottomLimitSwitch = 1
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class OperatorConfig {
    /**
     * The joystick port for the driver's controller.
     */
    public int driverPort = 1;
    public int copilotPort = 2;
  }

  public static class Swerve {
    public static final double kTrackWidth = 0.0;
    
    public static final double kWheelBase = 0.0;
  }

  // Add custom subsystem configs here.
  // e.g.. public static class drivetrainConfig {
  // }
  // 
  // Then instantiate below
  // e.g. drivetrain = new drivetrainConfig();


  // These are the actual configuration attributes.
  // Each independent aspect of config has its own type
  // so they are scoped to where they are needed and relevant.

  /**
   * The CAN bus configuration.
   */
  public CanBus canBus = new CanBus();

  /**
   * The port allocation on the RoboRIO.
   */
  public RioPorts ports = new RioPorts();

  /**
   * Configuration related to the operator interface.
   */
  public OperatorConfig operator = new OperatorConfig();
}
