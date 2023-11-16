package team1403.robot.turret;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.RobotConfig;

public class Turret extends SubsystemBase {
    private boolean commandCalled = false;
    private TalonSRX yawMotor;

    private TurretEncoderV2 absEncoder;
    
    public Turret() {
        yawMotor = new TalonSRX(RobotConfig.Turret.turretMotor);
        yawMotor.setInverted(true);
        absEncoder = new TurretEncoderV2(RobotConfig.Turret.absEncoderPort);
    }
    
    public void setSpeedManual(double in) {
        commandCalled = true;
        if(absEncoder.getTurretAngle() < 360.0-absEncoder.getDeadzoneWidth() && absEncoder.getTurretAngle() > 0) {
            yawMotor.set(ControlMode.PercentOutput, in);
        } else if(absEncoder.getTurretAngle() > 360 - absEncoder.getDeadzoneWidth() && in < 0) {
            yawMotor.set(ControlMode.PercentOutput, in);
        } else if(absEncoder.getTurretAngle() < 0 && in > 0) {
            yawMotor.set(ControlMode.PercentOutput, in);
        }
        else {
            stop();
        }
    }

    public void setSpeed(double in) {
        if(in > .5) {
           in = .5; 
        }
        if(in < -.5) {
            in = -.5;
        }
        
        yawMotor.set(ControlMode.PercentOutput, in);
    }
    
    public void forceSpeed(double in) {
        yawMotor.set(ControlMode.PercentOutput, in);
    }
    public void runPID() {
        setSpeed(absEncoder.getPIDOutput());
    }

    //to account for whether robot is on the left or right side of the target so the shots can go to inner port
    public void runPIDWithOffset(double angleOffset) {
        double currSetpoint = absEncoder.getSetpoint();
        absEncoder.setSetpoint(currSetpoint + angleOffset);
        setSpeed(absEncoder.getPIDOutput());
        absEncoder.setSetpoint(currSetpoint);
    }

    public void stop() {
        yawMotor.set(ControlMode.PercentOutput, 0);
    }

    public TalonSRX getMotor() {
        return this.yawMotor;
    }


    public TurretEncoderV2 getEncoder() {
        return this.absEncoder;
    }

    public void setOffset(int reading) {
        this.absEncoder.setOffset(reading);
    }
    
    public void setSetpoint(double turretAngle) {
        this.absEncoder.setSetpoint(turretAngle);
    }

    public double getSetpoint() {
        return absEncoder.getSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret Command Call", commandCalled );
        SmartDashboard.putNumber("Turret Angle", absEncoder.getTurretAngle());
    }
}