package team1403.robot.util;


public class Gains  {
	public double kP;
	public double kI;
	public double kD;
	public double kF;
	public int kIzone;
	public double kPeakOutput;
	public double maxAccumError;
	
	public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kIzone = kIzone;
		this.kPeakOutput = kPeakOutput;
	}

	public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput, double maxAccumError) {
		this(kP, kI, kD, kF, kIzone, kPeakOutput);
		this.maxAccumError = maxAccumError;
	}
	
}