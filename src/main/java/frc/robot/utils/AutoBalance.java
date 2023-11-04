package frc.robot.utils;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class AutoBalance {
    private final BuiltInAccelerometer mRioAccel;
    private int state;
    private int debounceCount;
    private final double robotSpeedSlow;
    private final double robotSpeedFast;
    private final double onChargeStationDegree;
    private final double levelDegree;
    private final double debounceTime;
    private final double singleTapTime;
    private final double scoringBackUpTime;
    private final double doubleTapTime;

    public AutoBalance() {
        mRioAccel = new BuiltInAccelerometer();
        state = 0;
        debounceCount = 0;

        // CONFIG
        robotSpeedFast = 0.4;
        robotSpeedSlow = 0.2;
        onChargeStationDegree = 13.0;
        levelDegree = 6.0;
        debounceTime = 0.2;
        singleTapTime = 0.4;
        scoringBackUpTime = 0.2;
        doubleTapTime = 0.3;
    }

    public double getPitch() {
        return Math.atan2(-mRioAccel.getX(), Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    }

    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if (pitch + roll >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    public double autoBalanceRoutine() {
        switch (state) {
            case 0:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;

            case 1:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0.0;
                }
                return robotSpeedSlow;

            case 2:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4;
                    debounceCount = 0;
                    return 0.0;
                }
                if (getTilt() >= levelDegree) {
                    return 0.1;
                } else if (getTilt() <= -levelDegree) {
                    return -0.1;
                }
                return 0.0;

            case 3:
                return 0.0;
        }
        return 0.0;
    }

    public double scoreAndBalance() {
        switch (state) {
            case 0:
                debounceCount++;
                if (debounceCount < secondsToTicks(singleTapTime)) {
                    return -robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime)) {
                    return robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime + doubleTapTime)) {
                    return -robotSpeedFast;
                } else {
                    debounceCount = 0;
                    state = 1;
                    return 0.0;
                }

            case 1:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;

            case 2:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    return 0.0;
                }
                return robotSpeedSlow;

            case 3:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4;
                    debounceCount = 0;
                    return 0.0;
                }
                if (getTilt() >= levelDegree) {
                    return robotSpeedSlow / 2;
                } else if (getTilt() <= -levelDegree) {
                    return -robotSpeedSlow / 2;
                }
                return 0.0;

            case 4:
                return 0.0;
        }
        return 0.0;
    }
}