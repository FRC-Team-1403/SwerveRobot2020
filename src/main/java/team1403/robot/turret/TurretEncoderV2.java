package team1403.robot.turret;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.util.PID;
import team1403.robot.Constants;

/* the turret encoder allow an absolute encoder to be used so that it
can interface with the turret. */

//In progress1

public class TurretEncoderV2 {

    private DutyCycleEncoder encoder;
    private PID positionPID;
    private final double deadzoneAngle = 15;
    private final double deadzoneSize = 115;
    private boolean shouldReset = true;
    // VVV the reading of the encoder when the magent is sensed
    private double offset = 0;
    private double magnetPosition = 180;
    private double robotStartAngle = 0;
    //private double previousValue;
    
    public TurretEncoderV2(int port) {
        encoder = new DutyCycleEncoder(port);
        //previousValue = encoder.get();
        this.positionPID = new PID("Turret encoder", this::getTurretAngle, 0.03, 0, 0);
    }
    //returns from 0 to 360 - deadzoneSize
    public double getTurretAngle() {
/*
        if (encoder.get() - previousValue < 0.5) {
            offset += 1;
        } else if (encoder.get() - previousValue > 0.5) {
            offset -= 1;
        }
        previousValue = encoder.get();
*/
        return (encoder.get() + offset) * 360 * Constants.Turret.absEncoderGearRatio;
    }

    //precondition: between 0 and 360 - deadzoneSize
    public void setSetpoint(double setpoint) {
        setpoint %= 360.0;
        boolean inDeadzone = false;
        if (setpoint < 0 || setpoint > 360.0-deadzoneSize/2.0) {
            setpoint = 0;
            inDeadzone = true;
        } else if (setpoint > 360 - deadzoneSize) {
            setpoint = 360 - deadzoneSize;
            inDeadzone = true;
        }
        SmartDashboard.putBoolean("Turret in deadzone", inDeadzone);
        positionPID.setSetpoint(setpoint);
    }

    //returns from -(180 + deadzoneAngle) to 180-deadzoneSize
    public double getRobotRelativeAngle() {
        return getTurretAngle() - (180) - deadzoneAngle;
    }

    public double getRobotRelativeSetpoint() {
        return getSetpoint() - 180.0;
    }

    //precondition: setpoint between (-180 + deadzoneAngle) to 180-deadzoneSize
    public void setRobotRelativeSetpoint(double setpoint) {
        setSetpoint(setpoint + 180);
    }

    public void setRobotStartAngle(double angle) {
        robotStartAngle = angle;
    }

    public double getRaw() {
        return encoder.get();
    }

    public double getDeadzoneWidth() {
        return this.deadzoneSize;
    }

    public double getDeadzoneAngle() {
        return this.deadzoneAngle;
    }

    public double getError() {
        return positionPID.getError();
    }

    public void handleReset() {
        if(shouldReset) {
            System.out.println("Resetting");
            //offset is in raw encoder reading
            offset = (magnetPosition / 360.0 / Constants.Turret.absEncoderGearRatio) 
                + deadzoneAngle/360.0 / Constants.Turret.absEncoderGearRatio
                - encoder.get();
                
            //positionPID is in turret angle
            positionPID.setOffset(offset * Constants.Turret.absEncoderGearRatio);
            shouldReset = false;
        }
    }

    public double getPIDOutput() {
        return positionPID.getValue();
    }


    public void setOffset(double offset) {
        positionPID.setOffset(offset);
    }

    public double getSetpoint() {
        return positionPID.getSetpoint();
    }

    public void toggleShouldReset() {
        shouldReset = !shouldReset;
    }

    public void requestReset() {
        shouldReset = true;
    }

    public boolean getShouldReset() {
        return shouldReset;
    }

}