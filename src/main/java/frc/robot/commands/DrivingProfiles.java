package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.Functions;

public class DrivingProfiles {

    // this is to normalize driving and make it easier for Mael to not drive into people


    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;
    private DoubleSupplier fowardJoystickInput, strafeJoystickInput, rotationJoystickInput, throttleJoystickInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private int controllerDriveCurveMag, controllerTurnCurveMag;

    private int joystickDriveCurveMag, joystickTurnCurveMag;

    private boolean controllerInUse, JoystickInUse;
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
        
    }


    private void updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = rotationControllerInput.getAsDouble();

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Math.hypot(forward, strafe);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttleControllerInput.getAsDouble();

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag);
    }

    private void updateJoystick() {
        double forward = fowardJoystickInput.getAsDouble();
        double strafe = strafeJoystickInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationJoystickInput.getAsDouble(), JoystickDeadband);
        double Direction = Math.atan2(forward, strafe);
        double drivePower = Functions.throttleCurve(throttleJoystickInput.getAsDouble(), joystickDriveCurveMag);
        if (Functions.deadbandValue(Math.hypot(forward, strafe), JoystickDeadband) == 0.0) {
            drivePower = 0;
        }

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, joystickTurnCurveMag) * drivePower;
    }




    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }

    

}
