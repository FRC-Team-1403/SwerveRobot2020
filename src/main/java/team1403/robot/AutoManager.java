package team1403.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.RobotConfig.Swerve;
import team1403.robot.swerve.SwerveControllerCommand;
import team1403.robot.swerve.SwerveSubsystem;

public class AutoManager {
  static private AutoManager m_instance;

  // for every path in the group
  ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("TestingAuto", new PathConstraints(4, 3));

  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  private final TrajectoryConfig m_straightTrajectoryConfig = new TrajectoryConfig(3,
  3.0).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig redRight2PieceTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig blueRight2PieceTrajectoryConfig1 = new TrajectoryConfig(14.5,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right2PieceTrajectoryConfig2 = new TrajectoryConfig(4,
      1).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right3PieceTrajectoryConfig2 = new TrajectoryConfig(10,
      3).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig m_reverseTrajectoryConfig3 = new TrajectoryConfig(10,
      2).setKinematics(RobotConfig.Swerve.kDriveKinematics);

  private final TrajectoryConfig right3PieceTrajectoryConfig1 = new TrajectoryConfig(3,
       3.25).setKinematics(RobotConfig.Swerve.kDriveKinematics).addConstraint(
        new RectangularRegionConstraint(new Translation2d(1, 1),
            new Translation2d(-1, -5),
            new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)))
    .addConstraint(
        new RectangularRegionConstraint(new Translation2d(-4.5, 1),
        new Translation2d(-6, -5),
        new SwerveDriveKinematicsConstraint(RobotConfig.Swerve.kDriveKinematics, 4)));

  private final PIDController xController = new PIDController(
      RobotConfig.Swerve.kPTranslation,
      RobotConfig.Swerve.kITranslation,
      RobotConfig.Swerve.kDTranslation);

  private final PIDController yController = new PIDController(
      RobotConfig.Swerve.kPTranslation,
      RobotConfig.Swerve.kITranslation,
      RobotConfig.Swerve.kDTranslation);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      4,
      RobotConfig.Swerve.kIAutoTurning,
      RobotConfig.Swerve.kDAutoTurning,
      RobotConfig.Swerve.kThetaControllerConstraints);

  private SwerveControllerCommand redRight2PieceTrajectory1;
  private SwerveControllerCommand redRight2PieceTrajector;

  private SwerveControllerCommand blueRight2PieceTrajectory1;
  private SwerveControllerCommand blueRight2PieceTajectory2;

  private SwerveControllerCommand redRight3PieceTrajectory1;
  private SwerveControllerCommand redRight3PieceTrajectory2;
  private SwerveControllerCommand redRight3PieceTrajectory3;  

  private Command pathplannerAuto;
  private SwerveControllerCommand blueSide3PieceTrajectory2Copy;

  private SwerveControllerCommand blueSide3PieceTrajectory1;
  private SwerveControllerCommand blueSide3PieceTrajectory2;
  private SwerveControllerCommand blueSide3PieceTrajectory3;

  private SwerveControllerCommand straightTrajectory1;

  private SwerveControllerCommand balanceTrajectory;

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
    );
    //this changes it but idk what to do next, still researching... But we need to make it into a swerve command
    pathplannerAuto = autoBuilder.fullAuto(pathGroup);

  }
  /**
   * Red side right grid autonomous command. 
   * Score 2 pieces.
   */
  public Command 
  getPathplannerAuto(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return pathplannerAuto;
  }

  /**
   * Blue side right grid autonomous command. 
   * Score 2 pieces.
   */
  public Command getBlueRightGrid2PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.45),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 3.85),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new WaitCommand(0.01),
                new SetpointArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false)),
            new SequentialCommandGroup(
                blueRight2PieceTrajectory1,
                blueRight2PieceTajectory2,
                blueSide3PieceTrajectory2Copy)));
  }

  /**
   * Red side right grid autonomous command. 
   * Score 3 pieces.
   */
  public Command getRedRightGrid3PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.65),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.2),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false),
                new RunIntake(arm, -1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.05),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))),
            new SequentialCommandGroup(
                redRight3PieceTrajectory1,
                redRight3PieceTrajectory2,
                new WaitCommand(0.5),
                redRight3PieceTrajectory3)));
  }

  /**
    * Blue side right grid autonomous command. 
    * Score 3 pieces.
   */
  public Command getBlueRightGrid3PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.65),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE)),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true),
                new RunIntake(arm, 1, 1.3),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), true),
                new SequentialMoveArmCommand(arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(),
                    false),
                new RunIntake(arm, -1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
                new WaitCommand(0.04),
                new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))),
            new SequentialCommandGroup(
                blueSide3PieceTrajectory1,
                blueSide3PieceTrajectory2,
                new WaitCommand(0.5),
                blueSide3PieceTrajectory3)));
  }

  public Command get1PieceCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> RobotConfig.ArmStates.coneHighNodeAuton, false),
        new RunIntake(arm, 1),
        new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false),
        straightTrajectory1);
  }

  public Command getMiddleGridCommand(SwerveSubsystem swerve, ArmSubsystem arm) {
    swerve.setSpeedLimiter(1);
    return new SequentialCommandGroup(
        new SequentialMoveArmCommand(arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), false),
        new RunIntake(arm, 1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new SetpointArmCommand(arm, () -> ArmStateGroup.getTuck(), false)
            ),
            balanceTrajectory
        ),
        new SwerveAutoBalanceYaw(swerve));
  }
}