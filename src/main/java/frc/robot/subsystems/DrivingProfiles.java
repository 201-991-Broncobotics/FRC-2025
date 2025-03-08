package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.utility.Functions;

/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */
public class DrivingProfiles extends SubsystemBase {

    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;
    private DoubleSupplier fowardJoystickInput, strafeJoystickInput, rotationJoystickInput, throttleJoystickInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private double controllerDriveCurveMag, controllerTurnCurveMag;
    private double joystickDriveCurveMag, joystickTurnCurveMag;

    private boolean preferController = true;

    private final double ControllerDeadband = 0.05, JoystickDeadband = 0.08;

    private double presetThrottleControl = 0.25;
    private boolean useThrottlePreset = false;

    private Joystick joystick;


    public DrivingProfiles(boolean PreferController) {
        this.preferController = PreferController;

    }

    public DrivingProfiles() {
        new DrivingProfiles(preferController);
    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }

    public void setUpJoystickInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
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
        double throttle = throttleControllerInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), ControllerDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }

    private boolean updateJoystick() {
        double forward = fowardJoystickInput.getAsDouble();
        double strafe = strafeJoystickInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationJoystickInput.getAsDouble(), JoystickDeadband);
        double throttle = throttleJoystickInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), JoystickDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, joystickDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, joystickTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }

    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }


    public void enableSlowDown() { useThrottlePreset = true; }
    public void disableSlowDown() { useThrottlePreset = false; }


    public void giveJoystickForTelemetry(Joystick joystick) {
        this.joystick = joystick;
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Pigeon accel X", gyroData.accelX);
        SmartDashboard.putNumber("Pigeon accel Y", gyroData.accelY);
        SmartDashboard.putNumber("Pigeon accel Z", gyroData.accelZ);
        SmartDashboard.putNumber("Pigeon pitch", gyroData.pitch);
        SmartDashboard.putNumber("Pigeon roll", gyroData.roll);
        SmartDashboard.putNumber("Pigeon yaw", gyroData.yaw);
        SmartDashboard.putNumber("Pigeon angVel X", gyroData.angVelX);
        SmartDashboard.putNumber("Pigeon angVel Y", gyroData.angVelY);
        SmartDashboard.putNumber("Pigeon angVel Z", gyroData.angVelZ);
        
    }

}
