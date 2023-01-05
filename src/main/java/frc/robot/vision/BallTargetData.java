package frc.robot.vision;

public class BallTargetData {

    private final boolean validTargetData;
    private final double x_pixels;
    private final double y_pixels;

    public BallTargetData(
            boolean validTargetData,
            double x_pixels,
            double y_pixels) {
        this.validTargetData = validTargetData;
        this.x_pixels = x_pixels;
        this.y_pixels = y_pixels;
    }
    
    public boolean ValidTargetData() {
        return this.validTargetData;
    }
    
    public double X_pixels() {
        return this.x_pixels;
    }
    
    public double Y_pixels() {
        return this.y_pixels;
    }
}
