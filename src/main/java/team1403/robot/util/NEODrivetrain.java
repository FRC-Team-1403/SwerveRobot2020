package team1403.robot.util;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.RobotConfig;
import team1403.robot.util.Gains;
import team1403.robot.RobotContainer;
import team1403.robot.util.Conversions;

public class NEODrivetrain extends SubsystemBase {
    public CANSparkMax frontLeft, frontRight, backLeft, backRight;
    RobotContainer container;

   public ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    public NEODrivetrain(RobotContainer container){
        frontLeft = new CANSparkMax(Constants.neoFrontLeft, MotorType.kBrushless);
        backLeft = new CANSparkMax(Constants.neoBackLeft, MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.neoFrontRight, MotorType.kBrushless);
        backRight = new CANSparkMax(Constants.neoBackRight, MotorType.kBrushless);
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setInverted(false);
        backRight.setInverted(false);

        configSparkMaxPID(frontLeft, Constants.kGainsLeft_Velocit);
        configSparkMaxPID(backLeft, Constants.kGainsLeft_Velocit);
        configSparkMaxPID(backRight, Constants.kGainsRight_Velocit);
        configSparkMaxPID(frontRight, Constants.kGainsRight_Velocit);
        
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
        this.container = container;
    }

    public void configSparkMaxPID(CANSparkMax motor, Gains gains) {
        CANPIDController controller = motor.getPIDController();
        controller.setP(gains.kP);
        controller.setI(gains.kI);
        controller.setD(gains.kD);
        controller.setFF(gains.kF);
        controller.setOutputRange(-1, 1);
    }

    public int getNormalizedGyroReading() {
        int angle = (int)m_gyro.getAngle();
        if(angle < 0) {
            return 360 -(-angle % 360);
        }
        return angle % 360;
    }

    public double getGyroReading() {
        return m_gyro.getAngle();
    }
    public void turnGyro(double PID) {
        frontLeft.set(PID);
        frontRight.set(-PID);
    }
    
    public void driveTank(double left, double right){
        frontLeft.set(-left);
        frontRight.set(right);
    }

    public void setWheelFPS(double left, double right) {
        left = Conversions.wheelFPSToWheelRPM(left);
        right = Conversions.wheelFPSToWheelRPM(right);
        setVelocity(left, right);
    }

    public void arcadeDrive(double straight, double angle) {
        setVelocity(straight - angle, straight + angle);
    }

    public void setVelocity(double left, double right) {
        frontLeft.getPIDController().setReference(left * Constants.dtGearRatio, ControlType.kVelocity);
        frontRight.getPIDController().setReference(right * Constants.dtGearRatio, ControlType.kVelocity);
    }

    public void set(double left, double right) {
        frontLeft.set(left);
        frontRight.set(right);
    }

    public void stop(){
        setVelocity(0,0);
    }

    public double getVelocity() {
        double avg = (frontLeft.getEncoder().getVelocity() + frontRight.getEncoder().getVelocity())/2;
        return avg;
    }

}