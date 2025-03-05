package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.utility.Functions;

public class DrivingProfiles {

    // this is to normalize driving and make it easier for Mael to not drive into people


    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;
    private DoubleSupplier fowardJoystickInput, strafeJoystickInput, rotationJoystickInput, throttleJoystickInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private int controllerDriveCurveMag, controllerTurnCurveMag;
    private int joystickDriveCurveMag, joystickTurnCurveMag;

    private boolean preferController = true;

    private final double ControllerDeadband = 0.05, JoystickDeadband = 0.05;


    public DrivingProfiles(boolean PreferController) {
        this.preferController = PreferController;
    }

    public DrivingProfiles() {
        new DrivingProfiles(preferController);
    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, int driveCurveMag, int turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }

    public void setUpJoystickInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, int driveCurveMag, int turnCurveMag) {
        this.fowardJoystickInput = fowardInput;
        this.strafeJoystickInput = strafeInput;
        this.rotationJoystickInput = rotationInput;
        this.throttleJoystickInput = throttleInput;
        this.joystickDriveCurveMag = driveCurveMag;
        this.joystickTurnCurveMag = turnCurveMag;
    }

    public void update() {
        if (preferController) {
            if (updateController()) return;
            else if (updateJoystick()) return;
        } else {
            if (updateJoystick()) return;
            else if (updateController()) return;
        }
        forwardOutput = 0;
        strafeOutput = 0;
        rotationOutput = 0;
    }


    private boolean updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationControllerInput.getAsDouble(), ControllerDeadband);

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), ControllerDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttleControllerInput.getAsDouble();

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag);

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }

    private boolean updateJoystick() {
        double forward = fowardJoystickInput.getAsDouble();
        double strafe = strafeJoystickInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationJoystickInput.getAsDouble(), JoystickDeadband);

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), JoystickDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, joystickDriveCurveMag) * throttleJoystickInput.getAsDouble();

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, joystickTurnCurveMag) * drivePower;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }

    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }

}
