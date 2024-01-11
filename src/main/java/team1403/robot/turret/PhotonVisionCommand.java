package team1403.robot.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PhotonVisionCommand extends CommandBase{
    private Limelight m_limelight;

    public PhotonVisionCommand(Limelight limelight){
        m_limelight = limelight;
    }

    @Override
    public void execute() {
        double distance = m_limelight.getDistanceFromTarget();
        SmartDashboard.putNumber("Distance to target", distance);
        double xDistance = m_limelight.getXDistance();
        SmartDashboard.putNumber("Distance to target", xDistance);
        double zDistance = m_limelight.getZDistance();
        SmartDashboard.putNumber("Distance to target", zDistance);
        double zAngle = m_limelight.getZAngle();
        SmartDashboard.putNumber("Angle", zAngle);
    }
}
