package frc.robot.vision;

public class TargetData {

    private final boolean validTargetData;
    private final double x_deg;
    private final double y_deg;

    public TargetData(
            boolean validTargetData,
            double x_deg,
            double y_deg) {
        this.validTargetData = validTargetData;
        this.x_deg = x_deg;
        this.y_deg = y_deg;
    }
    
    public boolean ValidTargetData() {
        return this.validTargetData;
    }
    
    public double X_deg() {
        return this.x_deg;
    }
    
    public double Y_deg() {
        return this.y_deg;
    }
}
