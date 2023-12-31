package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import team1403.lib.device.MotorController;
import team1403.lib.util.CougarLogger;

/**
 * Device implementation for a VictorSp motor controller.
 */
public class VictorSp extends VictorSP
                       implements MotorController {
  /**
   * Constructor.
   *
   * @param name The name for the device.
   * @param channel The CAN channel the motor is on.
   * @param logger The debug logger to use for the device.
   */
  public VictorSp(String name, int channel, CougarLogger logger) {
    super(channel);
    m_name = name;
    m_logger = logger;
  }

  /**
   * Return the WPI_VictorSP API so we can do something specific.
   *
   * @return The underlying {@code WPI_VictorSPX} instance.
   */
  public final VictorSP getVictorSpxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  @Override
  public final void setSpeed(double speed) {
    m_logger.tracef("setSpeed %s %f", getName(), speed);
    super.set(speed);
  }

  private final CougarLogger m_logger;
  private final String m_name;
}
