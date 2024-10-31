/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.network.node.policy;

import avdta.network.ReadNetwork;
import avdta.network.node.policy.IntersectionPolicy;
import avdta.network.Simulator;
import avdta.network.link.Link;
import avdta.vehicle.Vehicle;
import avdta.network.node.TBR;
import avdta.network.node.TBR;
import avdta.network.type.Type;
import avdta.vehicle.EmergencyVehicle;
import avdta.vehicle.route.FixedPath;
import java.util.Set;

/**
 * Prioritizes vehicles based on their reservation time (set on initialization, or when a vehicle reaches the front of its lane).
 * @author Michael
 */
public class FCFSPolicy extends IntersectionPolicy
{
    /**
     * Compares two {@link Vehicle}s according to their reservation time. {@link Vehicle}s are listed in order of decreasing priority.
     * @param v1 the first vehicle being compared
     * @param v2 the second vehicle being compared
     * @return sorting index for vehicles
     */
    public int compare(Vehicle v1, Vehicle v2)
    {
        if(v1.reservation_time != v2.reservation_time)
        {
            return v1.reservation_time - v2.reservation_time;
        }
        else if(v1.getNetEnterTime()!= v2.getNetEnterTime())
        {
            return v1.getNetEnterTime() - v2.getNetEnterTime();
        }
        else
        {
            return v1.getId() - v2.getId();
        }
    }

    /**
    * Initializes {@link Vehicle} reservation time the first time a {@link Vehicle} is initialized
    * @param node the node to be initialized at
    * @param v the vehicle to initialize
    */
    public void initialize(TBR node, Vehicle v)
    {
        if(v.reservation_time < 0)
        {
            v.reservation_time = Simulator.time;
        }
    }
    
    /**
     * Clears the reservation time when the {@link Vehicle}'s reservation is accepted.
     * @param v the {@link Vehicle} with an accepted reservation
     */
    public void onAccept(Vehicle v)
    {
        v.reservation_time = -1;
    }
    
    /**
     * Returns the type code associated with this policy
     * @return {@link ReadNetwork#FCFS}
     */
    public Type getType()
    {
        return ReadNetwork.FCFS;
    }
}