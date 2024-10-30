package avdta.network.link.movingbottleneck;

import avdta.network.link.MovingBottleneckLTMLink;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class MovingBottleneckCumulativeCountStorage {
    private TreeMap<Double, TreeMap<Double, Integer>> txccMap;

    public MovingBottleneckCumulativeCountStorage() {
        this.txccMap = new TreeMap<>();
    }

    public double getCC(Double t, Double x) {
        TreeMap<Double, Integer> posMap = this.txccMap.get(t);

        if(posMap != null) {
            Integer cc = (Integer)posMap.get(x);
            if(cc != null) {
                return cc;
            }
            else {
                return this.linearInterpolate(posMap, x);
            }
        }
        return 0;
    }

    public double getPreviousCC(Double t, Double x) {
        return this.txccMap.lowerEntry(t).getValue().get(x);
    }

    public void addCC(double t, double x, int cc) {
        this.txccMap.get(t).put(x, cc);
    }

    private double linearInterpolate(TreeMap<Double, Integer> positionMap, double x) {
        // Get the known positions around the given position
        Double lowerKey = positionMap.floorKey(x);
        Double upperKey = positionMap.ceilingKey(x);

        if (lowerKey == null || upperKey == null || lowerKey.equals(upperKey)) {
            return positionMap.get(lowerKey); // Interpolation is not possible
        }

        int lowerValue = positionMap.get(lowerKey);
        int upperValue = positionMap.get(upperKey);

        // Perform linear interpolation
        return lowerValue + ((x - lowerKey) / (upperKey - lowerKey) * (upperValue - lowerValue));
    }
}
