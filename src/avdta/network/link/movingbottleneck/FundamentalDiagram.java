package avdta.network.link.movingbottleneck;

import avdta.network.link.MovingBottleneckLTMLink;

public class FundamentalDiagram
{
    private MovingBottleneckLTMLink link;
    public double uf; // free flow speed
    public double w; // wave speed
    public double kj; // jam density
    public double q_max; // max capacity
    public double kc; // critical density

    public FundamentalDiagram(MovingBottleneckLTMLink link)
    {
        this.link = link;
        this.uf = link.getFFSpeed();
        this.w = link.getWaveSpeed();
        this.kj = link.getJamDensity();
        this.q_max = link.getCapacity();
        this.kc = (this.uf / (this.uf * this.w)) * this.kj;
    }

    // flow is Q(k)
    public double calculateQ(double k) {
        double a = Math.min(uf * k, w * (kj - k));
        if (k <= 0 || k >= this.kj) {
            return 0;
        }
        return a;
    }

    public double getTrafficSpeed(double k) {
        if (k < this.kc) {
            return this.uf;
        }
        else {
            return this.w;
        }
    }
}

