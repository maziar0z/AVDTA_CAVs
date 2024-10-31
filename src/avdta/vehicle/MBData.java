package avdta.vehicle;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class MBData implements Serializable
{
    public int veh_id;

    public List<Double> ff_time;
    public List<Double> ff_pos;
    public List<Double> w_time;
    public List<Double> w_pos;

    public MBData(int id)
    {
        this.veh_id = id;

        this.ff_time = new ArrayList<>();
        this.ff_pos = new ArrayList<>();
        this.w_time = new ArrayList<>();
        this.w_pos = new ArrayList<>();
    }

    public String toString()
    {
        return "\tMBData | "+this.veh_id+
                "\n\tt1 = "+this.ff_time+
                "\n\txj(t1) = "+this.ff_pos+
                "\n\tt2 = "+this.w_time+
                "\n\txj(t2) = "+this.w_pos+
                "\n\t---";
    }

    public double getFFTime(int idx)
    {
        return this.ff_time.get(idx);
    }

    public double getWTime(int idx)
    {
        return this.w_time.get(idx);
    }

    public double getFFPos(int idx)
    {
        return this.ff_pos.get(idx);
    }

    public double getWPos(int idx)
    {
        return this.w_pos.get(idx);
    }

    public void clear()
    {
        this.ff_time.clear();
        this.ff_pos.clear();
        this.w_time.clear();
        this.w_pos.clear();
    }
}
