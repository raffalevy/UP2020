package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static final double CM_PER_TICK_A = -100.0 / 1673.0;
    public static final double CM_PER_TICK_B = -100.0 / 1693.0;
    public static final double CM_PER_TICK_C = -100.0 / 1434.0;

    public static final double L = 17.4;
    public static final double TWO_L = 2 * L;

    private double x = 0;
    private double y = 0;
    private double theta = Math.PI / 4;

    public double oldA = 0;
    public double oldB = 0;
    public double oldC = 0;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void update(double a, double b, double c) {
        double dA = (a - oldA) * CM_PER_TICK_A;
        double dB = (b - oldB) * CM_PER_TICK_B;
        double dC = (c - oldC) * CM_PER_TICK_C;

//        OdometryTest.telemetryStatic.addData("dA", dA);
//        OdometryTest.telemetryStatic.addData("dB", dB);
//        OdometryTest.telemetryStatic.addData("dC", dC);

        oldA = a;
        oldB = b;
        oldC = c;

        double dx = ((dA - 2 * dB + dC) * Math.cos(theta) - (dA - dC) * Math.sin(theta)) / 2;
        double dy = ((dA - 2 * dB + dC) * Math.sin(theta) + (dA - dC) * Math.cos(theta)) / 2;
        double dTheta = (dA + dC) / TWO_L;

        this.x += dx;
        this.y += dy;
        theta += dTheta;
    }

}
