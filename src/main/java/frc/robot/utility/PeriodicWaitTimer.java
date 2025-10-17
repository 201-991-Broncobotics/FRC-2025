package frc.robot.utility;

public class PeriodicWaitTimer {
    private ElapsedTime timer;
    public double EndTime = 0;

    public PeriodicWaitTimer() {
    }

    public void start(double seconds) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        EndTime = seconds;
    }

    public boolean isPastTime() {
        return timer.time() > EndTime;
    }
}
