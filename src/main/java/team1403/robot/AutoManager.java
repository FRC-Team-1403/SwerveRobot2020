package team1403.robot;

import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.Constants.Swerve;
import team1403.robot.swerve.SwerveSubsystem;

public class AutoManager {
  static private AutoManager m_instance;

  // for every path in the group
  ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));

  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      4,
      Constants.Swerve.kIAutoTurning,
      Constants.Swerve.kDAutoTurning,
      Constants.Swerve.kThetaControllerConstraints);

  private Command pathplannerAuto;

  private AutoManager() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static AutoManager getInstance() {
    if (m_instance == null) {
      m_instance = new AutoManager();
    }
    return m_instance;
  }

  /**
   * Pathplanner auto.
   * @param swerve swerve subsystem object.
   */
  public void init(SwerveSubsystem swerve) {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2, we might need to change this down the road
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    //Idk the intake function but this is a example
    // eventMap.put("intakeDown", new intakefunction());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems. but let's move it later for testing it remains here anyways we need to find out our constants
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> swerve.getPose(), // Pose2d supplier
        pose -> swerve.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Swerve.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants( Swerve.kPTranslation, Swerve.kITranslation, Swerve.kDTranslation ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants( Swerve.kPAutoTurning, Swerve.kIAutoTurning, Swerve.kDAutoTurning ), // PID constants to correct for rotation error (used to create the rotation controller)
        moduleStates -> swerve.setModuleStates(moduleStates), // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should th>e path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );    //this changes it but idk what to do next, still researching... But we need to make it into a swerve command
    pathplannerAuto = autoBuilder.fullAuto(pathGroup).andThen(() -> swerve.stop());
  }

  /**
   * Example pathplanner auto
   */
  public Command getPathplannerAuto(SwerveSubsystem swerve) {
    //swerve.setSpeedLimiter(0.5);
    return pathplannerAuto;
  }
}