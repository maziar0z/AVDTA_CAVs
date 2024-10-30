package avdta.network.link.movingbottleneck;
import avdta.network.Network;
import avdta.network.Simulator;
import avdta.vehicle.VehTime;
import avdta.vehicle.Vehicle;

import java.util.LinkedList;


/**
 * This class handles the storage and calculation of positions for a MovingBottleneck vehicle.
 * It uses VehTime to uniquely identify each trajectory for each vehicle.
 */
public class Trajectory implements Comparable<Trajectory> {
    private VehTime vehTime;
    private double count;
    private double speed;
    private double position;
    private double timeInLink;
    private double arr_time;
    public boolean need_prepare;
    private MBToFile data_record;



    private LinkedList<Segment> piecewise;
    private Segment current_segment;
    private LinkedList<Double> cc;
    private double effective_time;


    private LinkedList<Integer> cumulativeCounts; // List to store CC at each step

    // for sending flow
    private LinkedList<Double> N_boundary_freeflow;
    private LinkedList<Double> t_intersect_freeflow;
    // for receiving flow
    private LinkedList<Double> N_boundary_wave;
    private LinkedList<Double> t_intersect_wave;


    /**
     * Constructs a Trajectory for the provided VehTime
     * Starts at timeInLink = 0 and position = 0
     * count is the current upstream CC when entering obtained from the VehTime class
     * arr_time is the arrrival time at link upstream
     * @param vt VehTime for identification
     */
    public Trajectory(VehTime vt) {
        this.vehTime = vt;
        this.count = vt.count;
        this.timeInLink = 0;
        this.position = 0;
        this.arr_time = vt.time;
        this.piecewise = new LinkedList<>();

        this.effective_time = 0;

        this.cumulativeCounts = new LinkedList<>();

        this.N_boundary_freeflow = new LinkedList<>();
        this.N_boundary_wave = new LinkedList<>();
        this.t_intersect_freeflow = new LinkedList<>();
        this.t_intersect_wave = new LinkedList<>();
        this.cc = new LinkedList<>();
        this.data_record = new MBToFile(vehTime.vehicle.getCurrLink().getId());
    }


    /**
     * @return the arrival time at link upstream
     */
    public double getArrTime() {
        return this.arr_time;
    }

    /**
     * @return current position in link
     */
    public double getCurrentPosition() {
        return this.position;
    }

    /**
     * @return the current time spent in the current link
     */
    public double getCurrentTimeInLink() {
        return this.timeInLink;
    }

    /**
     *
     * @return the current speed
     */
    public double getCurrentSpeed() {
        return this.speed;
    }

    /**
     *
     * @return the current segment
     */
    public Segment getCurrentSegment() {
        return this.current_segment;
    }

    /**
     *
     * @return list of all segments which make the piecewise (x, t) curve
     */
    public LinkedList<Segment> getPiecewise() {
        return this.piecewise;
    }

    /**
     * Sets the effective time for this trajectory
     * @param effectiveTime the time of arrival at link downstream
     */
    public void setEffectiveTime(double effectiveTime) {
        effective_time = effectiveTime;
    }

    /**
     * This method calculates the trajectory in the next time step based on the values in this class and the control speed through the next step.
     * @param control_speed speed through the next time step
     * @param cc_start cc at the start of the step, used when movement is assumed to update to correct CC if passing occured
     */
    public void stepTrajectory(double control_speed, double cc_start) {
        this.current_segment = new Segment(this.timeInLink, this.position, control_speed, cc_start);
        this.speed = control_speed;
        this.piecewise.add(current_segment);
        this.data_record.writeToBottleneckFile(this.getVehicle().getId(), Simulator.time + Network.dt, this.current_segment.y_end, this.speed, this.count, this.effective_time);

    }

    /**
     * This method is called in update, and assigns the values calculated in stepTrajectory() to the trajectory.
     * @param control_speed
     * @param passing_rate
     * @return false, as vehicle is still in link and will have boundary conditions calculated in next step.
     * @return an exception if the vehicle has left the link, instead the trajectory is prepared for the next step of simulation.
     */
    public boolean updateTrajectory(double control_speed, double passing_rate) {
        try {
            this.position += speed * Network.dt; //
            System.out.println(" speed  is " + speed + " traj updated ");

            this.timeInLink = current_segment.x_end;
            this.count = current_segment.cc_end + Math.floor(passing_rate * Network.dt);
            this.speed = control_speed;
            return false;
        }
        catch (Exception e) {
            return this.prepareNextStep(control_speed, passing_rate);
        }
    }

    // updates cumulative count of the traj
    public void trajectory_cc_updated(int cumulativecount1){
        this.cumulativeCounts.add(cumulativecount1); // Store the current CC in LinkedList
    }

    // updates position of traj
    public void trajectory_pos_updated(double control_speed){
        this.position += control_speed * 5280/3600 * Network.dt;
    }




    /**
     * This method is called if the vehicle is not able to bo updated in the current link (if the vehicle left there is no segments associated with it)
     * @param control_speed
     * @param passing_rate
     * @return true if successful, which calculates the first step of boundary conditions to be used in the next step of calculations.
     */
    public boolean prepareNextStep(double control_speed, double passing_rate) {
        this.current_segment = new Segment(0, 0, control_speed, this.count);
        this.piecewise.add(current_segment);
        this.count = current_segment.cc_end + Math.floor(passing_rate * Network.dt);
        this.position = current_segment.y_end;
        this.timeInLink = current_segment.x_end;
        this.speed = control_speed;
        return true;
    }

    /**
     * Updates the speed
     * @param control_speed
     */
    public void updateSpeed(double control_speed) {
        this.speed = control_speed;
    }

    /**
     * returns the segment which is ongoing during the given time
     * @param t time for which we want to know the segment
     * @return the segment
     */
    public Segment getSegment(double t) {
        if (t == 0) {
            return this.piecewise.get(0);
        }

        for (Segment segment : this.piecewise) {
            if (segment.x_start <= t && segment.x_end >= t) {
                return segment;
            }
        }
        return null;
    }

    /**
     *
     * @return position from previous time step based on previous segment
     */
    public double getPreviousPosition() {
        try {
            return this.piecewise.get(this.piecewise.size() - 2).y_start;
        }
        catch (Exception e) {
            return 0;
        }

    }

    public void updateCount(double cc) {
        this.count = cc;
    }

    /**
     *
     * @return the count of this vehicle
     */
    public double getCount() {
        return this.count;
    }

    public int getCount2() {
            // Get the latest CC for this vehicle
            return (int) this.cumulativeCounts.getLast();
    }

    /**
     *
     * @return current upstream boundary condition
     */
    public double getCurrentFFBoundaryCondition() {
        return this.N_boundary_freeflow.getLast();
    }

    /**
     *
     * @return current downstream boundary condition
     */
    public double getCurrentWBoundaryCondition() {
        return this.N_boundary_wave.getLast();
    }

    /**
     * Downstream boundary condition
     * @param cc boundary condition to be stored
     */
    public void addN_boundary_ff(double cc) {
        this.N_boundary_freeflow.add(cc);
    }

    /**
     * Upstream boundary condition
     * @param cc boundary condition to be stored
     */
    public void addN_boundary_wave(double cc) {
        this.N_boundary_wave.add(cc);
    }

    public double getCurrentFFIntersect() {
        return this.t_intersect_freeflow.getLast();
    }

    /**
     *
     * @return current upstream boundary condition, for receiving flow
     */
    public double getCurrentWaveIntersect() {
        return this.t_intersect_wave.getLast();
    }

    /**
     *
     * @param t time of intersection with Free Flow line with the vehicle trajectory line
     */
    public void addFFIntersect(double t) {
        this.t_intersect_freeflow.add(t);
    }

    /**
     *
     * @param t time of intersection of Wave line with the vehicle trajectory line
     */
    public void addWaveIntersect(double t) {
        this.t_intersect_wave.add(t);
    }

    /**
     * Used to determine boundary condition based on count of vehicle at intersection
     * @param t time where vehicle position is wanted
     * @return the position of this vehicle at time t
     */
    public double findPosition(double t) {
        double pos = 0;
        for (Segment segment : this.piecewise) {
            if (segment.x_start <= t && segment.x_end >= t) {
                pos += segment.y_start;
                pos += segment.slope * (t - segment.x_start);
            }
        }
        return pos;
    }

    public VehTime getVehTime() {
        return this.vehTime;
    }

    /**
     *
     * @return the vehicle associated with the VehTime stored in trajectory
     */
    public Vehicle getVehicle() {
        return this.vehTime.vehicle;
    }

    /**
     * clears all the lists of values for when this trajectory enters a new link
     */
    public void clear() {
        this.piecewise.clear();
        this.N_boundary_wave.clear();
        this.N_boundary_freeflow.clear();
        this.t_intersect_wave.clear();
        this.t_intersect_freeflow.clear();
        this.cc.clear();
    }

    /**
     *
     * @return printout of important values
     */
    public String displayData() {
        return "Vehicle "+this.getVehicle()+
                "\n\tPosition = "+this.position+
                "\n\tTime = "+this.timeInLink+
                "\n\tSpeed = "+this.speed+
                "\n\tN("+this.timeInLink+", "+this.position+") = "+this.count+
                "\n\tPrevious Conditions "+
                "\n\tFREE FLOW INTERSECTIONS = "+this.t_intersect_freeflow+
                "\n\tWAVE SPEED INTERSECTIONS = "+this.t_intersect_wave+
                "\n\t\tFreeFlow "+this.N_boundary_freeflow+
                "\n\t\tWave "+this.N_boundary_wave;
    }

    /**
     * Used when sorting through trajectories
     * @param o the object to be compared.
     * @return the larger
     */
    @Override
    public int compareTo(Trajectory o) {
        return Double.compare(this.count, o.count);
    }
}
