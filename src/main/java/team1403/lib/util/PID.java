package team1403.lib.util;

import java.util.function.DoubleSupplier;



public class PID  {
    
    private DoubleSupplier reading;
    private double setpoint;
    private double P, I, D;
    private double previousError = 0;
    private double accumulatedError = 0;
    private final double dt = .020;
    private DoubleSupplier setpointSupplier = null;
    private double offset = 0;
    private double maxOutput = Double.MAX_VALUE;
    private boolean isDisabled = false;

    public PID(String name, DoubleSupplier supplier, double P, double I, double D){
        reading = supplier;
        setpoint = reading.getAsDouble();
        this.P = P;
        this.I = I;
        this.D = D;
        //Shuffleboard.getTab("PID").add(name, this);
    }
    
    public PID(String name, DoubleSupplier supplier, Gains gains) {
        this(name, supplier, gains.kP, gains.kI, gains.kD);
    }
    public void setSetpointSupplier(DoubleSupplier supplier) {
      this.setpointSupplier = supplier;
    }

    public void setSetpoint(double setpoint) {
      this.setpoint = setpoint;
    }

    public void setOffset(double offset) {
      this.offset = offset;
    }

    public double getOffset() {
      return this.offset;
    }

    public double getSetpoint() {
      if(setpointSupplier == null) {
        return this.setpoint + offset;
      }
      return this.setpointSupplier.getAsDouble() + offset;
    }
  
    public void disablePID() {
      this.isDisabled = true;
    }
    
    public void enablePID() {
      this.isDisabled = false;
    }

  public double getError(){
    return this.getSetpoint() - getReading();
  }

  public double getReading() {
    return reading.getAsDouble();
  }

  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }

  public double getValue() {
    
    if(this.isDisabled) {
      return 0;
    }

    double error = getError();
    double _P = P * error;
    accumulatedError += error * dt;
    double _I = accumulatedError * I;
    double _D = D * (error - previousError) / dt;
    previousError = error;
    if(_P + _I + _D > maxOutput) {
      return maxOutput;
    } else if(_P + _I + _D < -maxOutput) {
      return -maxOutput;
    }
    return _P + _I + _D;
  }

}