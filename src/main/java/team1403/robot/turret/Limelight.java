package team1403.robot.turret;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    
    private final double TARGET_HEIGHT = 90;//inches //90 is acutal height
    private final double CAMERA_HEIGHT = 34;//inches
    private final double CAMERA_ANGLE = 20;//degrees
    private final double zOffset = 29;
    private final double errorLeniency = 1.5;
    private double prevDistance = 0;
    private boolean shouldAdjust = false;

    public double getEntry(String entryName){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entryName).getDouble(0.0);
    }

    public static void setEntry(String entryName, double value) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(entryName).setNumber(value);
    }

    public void configureUseMode(boolean adjust) {
        this.shouldAdjust = adjust;
    }

    public boolean getShouldAdjust() {
        return shouldAdjust;
    }

    public static void turnOn() {
        setEntry("ledMode", 3);
    }

    public static void turnOff() {
        setEntry("ledMode", 1);
    }

    public double getX() {
        return getEntry("tx");
    }

    public double getAdjustedX(double fieldRelativeAngle) {
        double phi = 90 - getX() - fieldRelativeAngle;
        double dist = getDistanceToTarget();
        double comp = phi - Math.atan2((dist*Math.sin(Math.toRadians(phi))), dist * Math.cos(Math.toRadians(phi)) + zOffset);
        comp = Math.toDegrees(comp);
        //SmartDashboard.putNumber("Comp", comp);
        return (comp + getX());
    }

    public double getY() {
        return getEntry("ty");
    }

    public boolean isCentered() {
        return Math.abs(getX()) < errorLeniency;
    }

    public boolean hasTarget() {
        if(getEntry("tv") == 0) {
            return false;
        }
        return true;
    }
    
    public double getDistanceToTarget() {
        return (TARGET_HEIGHT - CAMERA_HEIGHT) 
            / Math.tan((CAMERA_ANGLE + getEntry("ty")) / 180 * Math.PI);
    }
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Get Entry Limelight", getEntry("null"));
        SmartDashboard.putNumber("Limelight Distance", getDistanceToTarget());
        SmartDashboard.putNumber("Limelight Y", getY());
        SmartDashboard.putNumber("Limelight X", getX());
    }
}