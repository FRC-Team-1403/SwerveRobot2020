package team1403.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.turret.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * <p>
 * GUYS, WE FOUND THE ROBOT
 */
public class Robot extends TimedRobot {

    public static RobotContainer robotContainer;

    public static boolean isInTestMode = false;
    private Command limelightOff;
    private Command limelightOn;


    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        CommandScheduler.getInstance().enable();
        robotContainer = new RobotContainer();

        robotContainer.getLimelight();
        limelightOff = new InstantCommand(() -> Limelight.turnOff());
        limelightOn = new InstantCommand(() -> Limelight.turnOn());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        LiveWindow.enableAllTelemetry();
        limelightOff.schedule();
    }

    @Override
    public void disabledPeriodic() {
        //robotContainer.updateAuton();
    }

    @Override
    public void autonomousInit() {
        LiveWindow.enableAllTelemetry();
        limelightOn.schedule();
        robotContainer.initAuto();
        Command autonCommand = robotContainer.getAutonCommand();

        if (autonCommand != null){
            autonCommand.schedule();
            DriverStation.reportError("Auton is Scheduled", false);
        }
        DriverStation.reportError("Auton is initialized", false);
        
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.updateAuton();
    }

    @Override
    public void teleopInit() {
        LiveWindow.enableAllTelemetry();
        limelightOn.schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        isInTestMode = true;
    }

    @Override
    public void testPeriodic() {
        super.testInit();
    }
}