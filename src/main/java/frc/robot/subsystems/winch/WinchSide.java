package frc.robot.subsystems.winch;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

// This class controls one of the two winches
public class WinchSide {
    
    private final static double COUNTS_PER_MOTOR_REV = 2048.0;
  
    // the drum ID is 2.02"
    // rope thickness is 3 mm, or 0.118"
    // diameter is 2.138", gives a 6.716" circumference. Gear ratio is 63:1, so 63 revs/6.716in is 9.38rev/in.
    private final static double MOTOR_REV_PER_INCH = 9.38;
  
    private final static double AT_TOP_INCH = 33.5; // set to 0 speed when we get here (max 33)
    //private final static double AT_TOP_INCH = 6; // test Winch

    private final static double NEAR_TOP_INCH = AT_TOP_INCH - 1.0; // go slow when we get here
  
    private final static double MAX_WINCH_INCH = AT_TOP_INCH + 0.25; // set soft limit to here, motor will disable torque
    private final static double MAX_WINCH_REV = MOTOR_REV_PER_INCH * MAX_WINCH_INCH; 
    private final static double MAX_WINCH_COUNTS = MAX_WINCH_REV * COUNTS_PER_MOTOR_REV;

    private final static double WINCH_FULL_SPEED = 1.0; // only in the range of 0 to 1
    private final static double WINCH_SLOW_SPEED = 0.5; // only in the range of 0 to 1, speed when near end of pull
    private final static double WINCH_CURRENT_LIMIT_A = 40.0;

    private final TalonFX winchMotor;
    private double winchPosition_in = 0.0; // current position
    private double maxWinchPosition_in = 0.0; // max position we've ever seen since reboot
    private boolean winchAtTop = false;
    private boolean winchNearTop = false;
    private boolean positionError = false; // if position is ever significantly less than max, motor was power cycled?

    public WinchSide(int canbusAddress, Boolean invert) {
        this.winchMotor = new TalonFX(canbusAddress, "Default Name");
        this.winchMotor.setInverted(invert);
        this.winchMotor.setSelectedSensorPosition(0.0);
        this.winchMotor.hasResetOccurred();

        // neutral behavior
        this.winchMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

        this.winchMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, WINCH_CURRENT_LIMIT_A, WINCH_CURRENT_LIMIT_A, 0.0));
        this.winchMotor.configOpenloopRamp(0.0); // will ramp manually only on the accel, not on the decel
    
        this.winchMotor.configForwardSoftLimitThreshold(MAX_WINCH_COUNTS);
        this.winchMotor.configForwardSoftLimitEnable(true);
    }

    public void RunRobotPeriodic() {
        var position_counts = this.winchMotor.getSelectedSensorPosition();
        var motorRevs = position_counts / COUNTS_PER_MOTOR_REV;
        this.winchPosition_in = motorRevs / MOTOR_REV_PER_INCH;

        this.checkForPositionError();

        if(this.winchPosition_in >= NEAR_TOP_INCH) {
            this.winchNearTop = true;
        }
        if(this.winchPosition_in >= AT_TOP_INCH) {
            this.winchAtTop = true;
        }
    }

    private void checkForPositionError() {
        // Position should only be going up.
        // If it goes down, we probably rebooted the motor, or didn't power cycle since reset winches.
        if(this.winchPosition_in < (this.maxWinchPosition_in - 0.5)) {
            this.positionError = true;
        }

        // detect motor restart after we've winched a bit
        if(this.maxWinchPosition_in > 0.25 && this.winchMotor.hasResetOccurred()) {
            this.positionError = true;
        }
        
        if(this.winchPosition_in > this.maxWinchPosition_in) {
            this.maxWinchPosition_in = this.winchPosition_in;
        }
    }

    // winch = true to winch if not at top yet
    // ramp = 0 to 1 to multiply speed so we can slowly throttle it up
    public void execute(boolean winch, double ramp) {

        var winchSpeed = 0.0;
        if(winch && !this.positionError && !this.winchAtTop) {
            winchSpeed = this.winchNearTop ? WINCH_SLOW_SPEED : WINCH_FULL_SPEED;
        }
        
        var limitedRamp = limit0to1(ramp);
        var rampedSpeed = winchSpeed * limitedRamp;
        this.winchMotor.set(TalonFXControlMode.PercentOutput, rampedSpeed);
    }

    private double limit0to1(double value) {
        if(value > 1.0) return 1.0;
        if(value < 0.0) return 0.0;
        return value;
    }

    // Returns how far the winch motor has pulled, in inches
    public double getWinchPosition_in() {
        return this.winchPosition_in;
    }

    public boolean getWinchAtTop() {
        return this.winchAtTop;
    }

    public boolean getPositionError() {
        return this.positionError;
    }
}
