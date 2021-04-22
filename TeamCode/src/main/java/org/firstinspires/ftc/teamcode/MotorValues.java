package org.firstinspires.ftc.teamcode;

public class MotorValues {
    public double fl, fr, bl, br;

    public double slowModeMultiplier = 0.5;

    public MotorValues(double cfl, double cfr, double cbl, double cbr, double slowMultiplier) {
        fl = cfl;
        fr = cfr;
        bl = cbl;
        br = cbr;
        slowModeMultiplier = slowMultiplier;
    }

    public MotorValues(double powerAll) {
        fl = powerAll;
        fr = powerAll;
        bl = powerAll;
        br = powerAll;
    }

    public void SlowMode() {
        fl *= slowModeMultiplier;
        fr *= slowModeMultiplier;
        bl *= slowModeMultiplier;
        br *= slowModeMultiplier;
    }

    public void NormaliseValues() {
        double maxSpeed = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));

        if (maxSpeed < 1) return;

        fl /= maxSpeed;
        fr /= maxSpeed;
        bl /= maxSpeed;
        br /= maxSpeed;
    }
}
