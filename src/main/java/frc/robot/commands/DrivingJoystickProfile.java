package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.utility.Functions;

public class DrivingJoystickProfile {
    // this is to normalize driving and make it easier for Mael to not drive into people


    private DoubleSupplier fowardInput, strafeInput, rotationInput, throttleInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private int driveCurveMag, turnCurveMag;


    public DrivingJoystickProfile(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, int driveCurveMag, int turnCurveMag) {
        this.fowardInput = fowardInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        this.throttleInput = throttleInput;
        this.driveCurveMag = driveCurveMag;
        this.turnCurveMag = turnCurveMag;

        update();
    }

    public void update() {
        double forward = fowardInput.getAsDouble();
        double strafe = strafeInput.getAsDouble();
        double turn = rotationInput.getAsDouble();
        double Direction = Math.atan2(forward, strafe);
        double drivePower = Functions.throttleCurve(throttleInput.getAsDouble(), driveCurveMag);
        if (Math.abs(forward) < 0.03 && Math.abs(strafe) < 0.03 && Math.abs(turn) < 0.03) {
            drivePower = 0;
        }

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, turnCurveMag) * drivePower;
    }

    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }



}
