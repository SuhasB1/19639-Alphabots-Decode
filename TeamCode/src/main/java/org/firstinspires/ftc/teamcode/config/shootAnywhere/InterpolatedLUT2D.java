package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class InterpolatedLUT2D {

    private static class Point {
        double x, y;
        ShooterParams params;
        Point(double x, double y, ShooterParams params) {
            this.x = x;
            this.y = y;
            this.params = params;
        }
    }

    private final List<Point> points = new ArrayList<>();

    public void addPoint(double x, double y, ShooterParams params) {
        points.add(new Point(x, y, params));
    }

    public ShooterParams get(double x, double y) {
        if (points.isEmpty()) return new ShooterParams(0, 0);

        List<Double> ys = new ArrayList<>();
        for (Point p : points) {
            if (!ys.contains(p.y)) ys.add(p.y);
        }
        ys.sort(Double::compare);

        double[] hoodValues = new double[ys.size()];
        double[] velValues = new double[ys.size()];

        for (int i = 0; i < ys.size(); i++) {
            double yi = ys.get(i);

            List<Point> row = new ArrayList<>();
            for (Point p : points) {
                if (p.y == yi) row.add(p);
            }
            if (row.isEmpty()) continue;

            row.sort(Comparator.comparingDouble(p -> p.x));

            double[] xArr = new double[row.size()];
            double[] hoodArr = new double[row.size()];
            double[] velArr = new double[row.size()];

            for (int j = 0; j < row.size(); j++) {
                xArr[j] = row.get(j).x;
                hoodArr[j] = row.get(j).params.hoodPos;
                velArr[j] = row.get(j).params.shooterVel;
            }

            hoodValues[i] = MonotoneCubic.interpolate(xArr, hoodArr, x);
            velValues[i] = MonotoneCubic.interpolate(xArr, velArr, x);
        }

        // Step 3: interpolate along y
        double[] yArr = ys.stream().mapToDouble(d -> d).toArray();
        double hood = MonotoneCubic.interpolate(yArr, hoodValues, y);
        double vel = MonotoneCubic.interpolate(yArr, velValues, y);

        return new ShooterParams(hood, vel);
    }
}
