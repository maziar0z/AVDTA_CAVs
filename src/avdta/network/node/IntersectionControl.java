/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.network.node;

import avdta.vehicle.DriverType;
import avdta.network.link.Link;
import avdta.network.type.Type;
import avdta.vehicle.Vehicle;

/**
 * This is an abstract class to implement intersection controls (<b>Stops</b>, 
 * <b>Conflict Regions</b>, <b>Signals</b>, etc.). Intersection Control is 
 * implemented separately from a node so that each node can have a different 
 * intersection control.<br>
 * {@code node} Indicates on which {@link Node} this intersection control is present.
 * @see Merge
 * @see Diverge
 * @see TrafficSignal
 * @see TBR
 * @author ut
 */
public abstract class IntersectionControl
{
    private Intersection node;
    /**
     * Instantiates the {@link IntersectionControl} with null {@link Node}.
     */
    public IntersectionControl()
    {
        node = null;
    }
    /**
     * Instantiates the intersection control with the input node value in n 
     * and sets the control of {@link Intersection} of the node in the input
     * to this intersection control.
     * @see Intersection
     * @param n An {@link Intersection} which contains the node on which the 
     * intersection control is to be implemented.
     */
    public IntersectionControl(Intersection n)
    {
        this.node = n;
        
        if(node != null)
        {
            node.setControl(this);
        }
    }

    public void prepare(){}
    
    /**
     * Returns the name of the instance of the {@link IntersectionControl} class.
     * @return A String which has the name of the instance of the intersection 
     * control class.
     */
    public String toString()
    {
        return getClass().getName();
    }
    /**
     * Returns the node at which the intersection control has been implemented.
     * @return A {@link Node} where the intersection control has been 
     * implemented.
     */
    public Intersection getNode()
    {
        return node;
    }
    /**
     * Sets the node at which this {@link IntersectionControl} is present.
     * @param n An {@link Intersection} which has the node where the control
     * is present.
     */
    public void setNode(Intersection n)
    {
        this.node = n;
    }
    /**
     * An abstract function to determine if there is a conflict region (for 
     * <b>TBR</b> based policy; TBR stands for Tile Based Reservation).
     * @return A boolean value indicating if this intersection control has
     * conflict regions.
     */
    public abstract boolean hasConflictRegions();
    /**
     * An abstract function to determine if a {@link Vehicle} can move. 
     * {@link DriverType} is required because certain policies may allow only
     * certain type of vehicles to move.
     * @param i An incoming {@link Link} to the intersection.
     * @param j An outgoing {@link Link} from the intersection.
     * @param driver For knowing the {@link DriverType}.
     * @return A boolean indicating if the vehicle can move from link i to link 
     * j.
     */
    public abstract boolean canMove(Link i, Link j, DriverType driver);
    /**
     * An abstract method to execute a time step at that control during 
     * simulation.
     * @return (Ask)
     */
    public abstract int step();
    
    /**
     * Resets this intersection to restart the simulation
     */
    public abstract void reset();
    
    /**
     * Perform any initialization work for this intersection. This is called after all data is read.
     */
    public abstract void initialize();
    
    /**
     * Returns the type code for this {@link IntersectionControl}
     * 
     * @return an int specifying the type code for this {@link IntersectionControl}
     */
    public abstract Type getType();
    
    
}