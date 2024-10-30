package avdta.network.link.movingbottleneck;

import avdta.network.Network;

public class Point {
    public double x;
    public double y;

    public Point(double time, double position) {
        this.x = time;
        this.y = position;
    }
}
