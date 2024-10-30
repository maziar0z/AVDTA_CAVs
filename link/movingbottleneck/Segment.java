package avdta.network.link.movingbottleneck;

import avdta.network.Network;

public class Segment {
    public double x_start;
    public double y_start;
    public double x_end;
    public double y_end;
    public double slope;

    public double cc_start;
    public double cc_end;

    public Segment(double time, double position, double speed, double cc) {
        this.x_start = time;
        this.y_start = position;
        this.slope = speed * 5280 / 3600;
        this.cc_start = cc;
        this.cc_end = -1;
        this.x_end = time + Network.dt;
        this.y_end = position + (speed * 5280 / 3600) * Network.dt;
    }

    public boolean areEqual(Segment compare) {
        return (this.x_start == compare.x_start &&
                this.x_end == compare.x_end &&
                this.y_start == compare.y_start &&
                this.y_end == compare.y_end &&
                this.slope == compare.slope);
    }
}
