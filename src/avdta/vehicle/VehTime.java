/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.vehicle;

import java.io.Serializable;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import avdta.network.Network;
import avdta.network.Simulator;
import avdta.network.link.LTMLink;
import avdta.network.link.MovingBottleneckLTMLink;

/**
 * This is a tuple of a {@link Vehicle} and an arrival time. 
 * This is used by the {@link LTMLink} class to check when {@link Vehicle}s have spent at least the free flow travel time on the link.
 * @author Michael
 */
public class VehTime implements Serializable
{
    public Vehicle vehicle;
    public MovingBottleneckLTMLink link;
    public int time; //arrival time @ link upstream
    public int count;

    /**
     * Constructs the {@link VehTime} with the given parameters
     * @param v the {@link Vehicle}
     * @param t the arrival time
     */
    public VehTime(Vehicle v, int t) {
        this.vehicle = v;
        this.time = t;
    }

    public VehTime(Vehicle v, int t, int c) {
        this.vehicle = v;
        this.time = t;
        this.count = c;
    }

    public int getCount() {
        return count;
    }

    public void setCount(int c) {
        this.count = c;
    }

    public void prepare() {

    }

    public void step() {

    }

    public void update() {

    }
}
