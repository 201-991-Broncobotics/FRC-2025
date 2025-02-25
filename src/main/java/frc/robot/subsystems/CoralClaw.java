package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClaw extends SubsystemBase {

    private double TargetRightAngle, TargetLeftAngle;

    private double CurrentRightAngle, CurrentLeftAngle;

    class diffyCoords{
        diffyCoords(double Roll, double Pitch){
            roll = Roll;
            pitch = Pitch;
        }

        diffyCoords vals(){
            //calculate for left servo
            servo1 = roll;
            //calculate for right servo
            servo2 = pitch;

            return new diffyCoords(servo1, servo2);
        }

        public double roll = 0,pitch = 0;
        public double servo1 = 0, servo2 = 0;
    }


    public CoralClaw() {

    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
