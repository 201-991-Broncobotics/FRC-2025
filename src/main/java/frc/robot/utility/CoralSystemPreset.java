package frc.robot.utility;

public class CoralSystemPreset {
    public double eleHeight = 0;
    public double pivotAngle = 0;

    /**
     * Creates an object that contains all the preset variables for a coral system position
     * @param elevatorHeight inches
     * @param PivotAngle degrees (0° is straight)
     */
    public CoralSystemPreset(double elevatorHeight, double PivotAngle) {
        eleHeight = elevatorHeight;
        pivotAngle = Math.toRadians(PivotAngle);
    }
}