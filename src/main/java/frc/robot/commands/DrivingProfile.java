package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import frc.robot.utility.Functions;

public class DrivingProfile {

    // this is to normalize driving and make it easier for Mael to not drive into people


    private DoubleSupplier fowardInput, strafeInput, rotationInput, throttleInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private int driveCurveMag, turnCurveMag;

    private int ControlSmoothingTimes = 3;
    private ArrayList<Double> forwardList, strafeList, turnList;


    public DrivingProfile(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, int driveCurveMag, int turnCurveMag, int ControlSmoothingTimes) {
        this.fowardInput = fowardInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        this.throttleInput = throttleInput;
        this.driveCurveMag = driveCurveMag;
        this.turnCurveMag = turnCurveMag;
        this.ControlSmoothingTimes = ControlSmoothingTimes;

        update();
        
    }

    public void update() {
        double forward = fowardInput.getAsDouble();
        double strafe = strafeInput.getAsDouble();
        double turn = rotationInput.getAsDouble();

        forwardList.add(forward);
        strafeList.add(strafe);
        turnList.add(turn);

        if (forwardList.size() > ControlSmoothingTimes) forwardList.remove(0);
        if (strafeList.size() > ControlSmoothingTimes) strafeList.remove(0);
        if (turnList.size() > ControlSmoothingTimes) turnList.remove(0);

        forward = forwardList.stream().mapToDouble(a -> a).average().orElse(0.0);
        strafe = strafeList.stream().mapToDouble(b -> b).average().orElse(0.0);
        turn = turnList.stream().mapToDouble(c -> c).average().orElse(0.0);

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Math.hypot(forward, strafe);
        double drivePower = Functions.throttleCurve(joystickPower, driveCurveMag) * throttleInput.getAsDouble();

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, turnCurveMag);
    }

    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }

    

}
