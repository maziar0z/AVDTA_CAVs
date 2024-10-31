/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package avdta.network.link;

import avdta.network.Network;
import avdta.network.ReadNetwork;
import avdta.network.Simulator;
import avdta.network.link.movingbottleneck.FundamentalDiagram;
import avdta.network.node.Node;
import avdta.network.node.Phase;
import avdta.network.node.TrafficSignal;
import avdta.network.type.Type;
import avdta.vehicle.VehTime;
import avdta.vehicle.Vehicle;
import avdta.network.link.movingbottleneck.*;


import java.util.*;

import static avdta.network.ReadNetwork.MOVING_BOTTLENECK;
import static avdta.network.ReadNetwork.STOPSIGN;
import static avdta.network.Simulator.*;

/**
 * This class implements the link transmission model. 
 * It uses {@link ChainedArray}s to store cumulative counts at the upstream and downstream ends, and stores vehicles in a {@link LinkedList}.
 * @author Charlie
 */
public class MovingBottleneckLTMLink extends Link
{
    private boolean init;

    private int Ndown_storage = 0;


    private int exited;

    // used for CAVs acting as moving bottlenecks

    private double search_length;

    private CumulativeCountStorage cc_saver;   // saves the cc of every vehicle entered this link.
    private CumulativeCountStorage ccMB_saver; // saves cc of MBs only

    private FundamentalDiagram Q;
    private LinkedList<Trajectory> bottlenecks;
    private MovingBottleneckCumulativeCountStorage N_mb;
    MBToFile data_record;

    private int count;
    
    private LinkedList<VehTime> queue;

    private CumulativeCountStorage N_up, N_down;
    private LinkedList<Double> N_boundary_up, N_boundary_down;
    private double Maz_N_boundary_up, Maz_N_boundary_down, MBCC_lowest; // CC_lowest is the lowest cc of in each link

    private double capacityUp, capacityDown;

    /**
     * Constructs the link with the given parameters 
     * @param id the link id
     * @param source the source node
     * @param dest the destination node
     * @param capacity the capacity per lane (veh/hour)
     * @param ffspd the free flow speed (mi/h)
     * @param wavespd the congested wave speed (mi/hr)
     * @param jamd the jam density (veh/mile)
     * @param length the length (feet)
     * @param numLanes the number of lanes
     */
    public MovingBottleneckLTMLink(int id, Node source, Node dest, double capacity, double ffspd, double wavespd, double jamd, double length, int numLanes)
    {
        super(id, source, dest, capacity, ffspd, wavespd, jamd, length * 5280, numLanes);

//        System.out.println(this.dataToString()); //debug

        init = false;
    }
    
    public Iterable<Vehicle> getVehicles()
    {
        return new MovingBottleNeckLTMIterable();
    }
    
    /**
     * Initializes this {@link MovingBottleneckLTMLink} after all data is read.
     * This creates the {@link ChainedArray}s used to store cumulative counts.
     */
    public void initialize()
    {
        data_record = new MBToFile(this.getId());
        count = 0;    // set the starting cumulative count (start from N=0)

        this.queue = new LinkedList<VehTime>();

        this.bottlenecks = new LinkedList<>();
        this.cc_saver = new ChainedArray(20);
        this.ccMB_saver = new ChainedArray(20);  // 20 is assumed to be max number of cars
        this.N_up = new ChainedArray(this.getUSLookBehind() + 2);
        this.N_down = new ChainedArray(this.getDSLookBehind() + 2);
        this.N_boundary_up = new LinkedList<>();
        this.N_boundary_down = new LinkedList<>();

        this.capacityUp = (getCapacity() / 3600) * Network.dt;
        this.capacityDown = (getCapacity() / 3600) * Network.dt;


        this.Q = new FundamentalDiagram(this);
        this.N_mb = new MovingBottleneckCumulativeCountStorage();

        search_length = 2 * getVehicleLength() + getMinimumGap() + 2;

        exited = 0;

        init = true;
    }

    /**
     * Resets this {@link LTMLink} to restart the simulation. This clears the {@link ChainedArray}s
     */
    public void reset()
    {
        queue.clear();
        N_up.clear();
        N_down.clear();


        super.reset();
    }

    // put control speed calc here
    public void prepare() {
    }

    //with initial, find next conditions
    // find cc at new veh pos here
    /**
     * Executes one time step of simulation.
     * This updates the current upstream and downstream capacities.
     * Adding and removing vehicles occurs through {@link Node}s.
     */
    public void step()
    {

        if (!this.bottlenecks.isEmpty()) {
            double next_loc = 0;
            double cc_lastMB = 1000; // start with a large number

            for (Trajectory trajectory : this.bottlenecks) {
                trajectory.stepTrajectory(this.calcControlSpeed(trajectory), trajectory.getCount2());

//               trajectory.trajectory_pos_updated(calcControlSpeed(trajectory));  // this updates vehicle position

                this.calculateBoundaryConditions(trajectory);
                cc_lastMB = Math.min(trajectory.getCount2() , cc_lastMB);  // get the min of the ccs in that link
                next_loc = Math.max(next_loc, trajectory.getCurrentPosition() + this.calcControlSpeed(trajectory) * Network.dt); // find the future location
                System.out.println("vehicle id is " + trajectory.getCount() + " current position " + trajectory.getCurrentPosition() +  " curr speed " + trajectory.getCurrentSpeed());

            }
//            System.out.println("N_bound_down " + this.getDSBottleneck().getCurrentFFBoundaryCondition() + "N_bound_up " + getUSBottleneck().getCurrentWBoundaryCondition());
            this.N_boundary_down.add(this.getDSBottleneck().getCurrentFFBoundaryCondition());
            this.N_boundary_up.add(this.getUSBottleneck().getCurrentWBoundaryCondition());
            MBCC_lowest = cc_lastMB;

            // related to sending flow calculations
            if (next_loc > this.getLength() - 1) { // the vehicle will arrive to that point
                Maz_N_boundary_down = cc_lastMB;
//                System.out.println("N_boundary_down "+ Maz_N_boundary_down);
            }
            else {  // no MB vehicle is going to arrive at the
                Maz_N_boundary_down = 0;
//                System.out.println("N_boundary_down "+ Maz_N_boundary_down);
            }
        }
        else {
            this.N_boundary_down.add((double)getN_up(Simulator.time + Network.dt - this.getLength() / this.getFPSFreeFlowSpeed()));
            this.N_boundary_up.add((double)getN_down(Simulator.time + Network.dt - this.getLength() / this.getFPSWaveSpeed()) + (this.getJamDensity() / 5280) * this.getLength());
        }

        capacityUp -= (int)capacityUp;
        capacityUp += getCapacity() * Network.dt / 3600;
        capacityDown -= (int)capacityDown;
        capacityDown += getCapacity() * Network.dt / 3600;


        for (Trajectory trajectory: this.bottlenecks) {
            trajectory.getCurrentSegment().cc_start = calcN_bottleneck(Simulator.time, trajectory.getCurrentPosition(), calcControlSpeed(trajectory), trajectory.getCurrentSegment().cc_start);
//            System.out.println("after _ cc start is " + trajectory.getCurrentSegment().cc_start);
        }
    }

    // veh reached next pos and time, so achieved next cc, set them here

    // set CURRENT values in step
    public void update()
    {
        // know when veh arrive to "quantize" value
        for (Trajectory trajectory : this.bottlenecks) {
//            if (this.getPassingRate(trajectory) > 0) {
//                trajectory.updateCount(this.getPassingRate(trajectory) * Network.dt);
//            }
            // update trajectory
            // need new update method --> if just moved, calculate the control speed and boundary condition for current and next step.
            if (trajectory.updateTrajectory(this.calcControlSpeed(trajectory), this.getPassingRate(trajectory) ) ) {
                this.calculateBoundaryConditions(trajectory);
//                System.out.println(trajectory.displayData());
            }

        }

        data_record.writeToLinkFile(time, this.R, this.S, this.queue.size());
        data_record.writeToVarFile(time, 0, this.calculateN(time, 0), this.getDensity(time, 0), this.getFlow(time, 0), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 220, this.calculateN(time, 220), this.getDensity(time, 220), this.getFlow(time, 220), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 440, this.calculateN(time, 440), this.getDensity(time, 440), this.getFlow(time, 440), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 660, this.calculateN(time, 660), this.getDensity(time, 660), this.getFlow(time, 660), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 880, this.calculateN(time, 880), this.getDensity(time, 880), this.getFlow(time, 880), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 1100, this.calculateN(time, 1100), this.getDensity(time, 1100), this.getFlow(time, 1100), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 1320, this.calculateN(time, 1320), this.getDensity(time, 1320), this.getFlow(time, 1320), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 1540, this.calculateN(time, 1540), this.getDensity(time, 1540), this.getFlow(time, 1540), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 1720, this.calculateN(time, 1720), this.getDensity(time, 1720), this.getFlow(time, 1720), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 1980, this.calculateN(time, 1980), this.getDensity(time, 1980), this.getFlow(time, 1980), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 2200, this.calculateN(time, 2200), this.getDensity(time, 2200), this.getFlow(time, 2200), getNumSendingFlow(), getReceivingFlow());
        data_record.writeToVarFile(time, 2640, this.calculateN(time, 2640), this.getDensity(time, 2640), this.getFlow(time, 2640), getNumSendingFlow(), getReceivingFlow());

        N_up.nextTimeStep();
        N_down.nextTimeStep();
    }

    // returns the cc of the current trajectory in that specific section
    public double MBccfinder(Trajectory trajectory) {
        // full list of vehicle's position, speed, and cc through link
        LinkedList<Segment> segments = trajectory.getPiecewise();
        // the current segment where the vehicle is
        Segment current_segment = trajectory.getCurrentSegment();
        double arr_time = trajectory.getArrTime();
        // calculates the cumulative count for the vehicle at the position it will be next step
        current_segment.cc_end = this.calcN_bottleneck(current_segment.x_end + arr_time, trajectory.getCurrentPosition(), current_segment.slope, current_segment.cc_start);
//        System.out.println("hey " + current_segment.cc_end);
        double cc_lastMB = current_segment.cc_end;  // save the cc of the leader MB in this section
        return cc_lastMB;
    }

    // finds boundary numbers related to N*
    public void calculateBoundaryConditions(Trajectory trajectory) {
        // full list of vehicle's position, speed, and cc through link
        LinkedList<Segment> segments = trajectory.getPiecewise();
        // the current segment where the vehicle is
        Segment current_segment = trajectory.getCurrentSegment();
        double arr_time = trajectory.getArrTime();
        // calculates the cumulative count for the vehicle at the position it will be next step
        current_segment.cc_end = this.calcN_bottleneck(current_segment.x_end + arr_time, current_segment.y_end, current_segment.slope, current_segment.cc_start);
//      double cc_lastMB = current_segment.cc_end;  // save the cc of the leader MB in this section
        // t_star1 cals calcFFBoundary() based on the piecewise position function of trajectory
        double t_star1 = this.calcFFBoundary(segments);
        double N_boundary_ff;

        // if time of intersection is NOT valid, calculate the boundary value based on traditional method
        if (t_star1 < 0 || t_star1 > Integer.MAX_VALUE) {
//            System.out.println("case 1, t_star= " +  t_star1 + " N_boundary is= " + (this.calcN_bottleneck(current_segment.x_end + arr_time, current_segment.y_end, current_segment.slope, current_segment.cc_start)-1) );
            t_star1 = current_segment.x_end - this.getLength() / this.getFPSFreeFlowSpeed();
            trajectory.addFFIntersect(t_star1);
            trajectory.addN_boundary_ff(this.calcN_bottleneck(current_segment.x_end + arr_time, current_segment.y_end, current_segment.slope, current_segment.cc_start)-1);  // the cc is the vehicle in front of MB
        }
        // if it is valid, find the CC at the intersection and assign that as the boundary value
        else {
//            System.out.println("case 22s");
            trajectory.addFFIntersect(t_star1);
            Segment segment_ff = trajectory.getSegment(t_star1);
            if (segment_ff != null) {
//                System.out.println("segment " + segment_ff.x_start);
                N_boundary_ff = linearInterpolate(segment_ff.x_start, segment_ff.cc_start, segment_ff.x_end, segment_ff.cc_end, t_star1);
//                System.out.println("case 2, t_star= " +  t_star1 + " N_boundary is= " + N_boundary_ff);
                trajectory.addN_boundary_ff(N_boundary_ff);
            }
        }
        double t_star2 = this.calcWBoundary(segments);
        trajectory.addWaveIntersect(t_star2);
        Segment segment_w = trajectory.getSegment(t_star2);
        double N_boundary_w = linearInterpolate(segment_w.x_start, segment_w.cc_start, segment_w.x_end, segment_w.cc_end, t_star2);
        trajectory.addN_boundary_wave(N_boundary_w);


//        System.out.println( " t star1 " + t_star1 + " star2 " + t_star2);
    }


    /**
     * Returns how far to look backwards in time for the upstream end
     * @return {@link Link#getLength()}/{@link Link#getFFSpeed()} (s)
     */
    public int getUSLookBehind()
    {
        return (int)Math.ceil(getLength()/getFFSpeed()*3600 / Network.dt);
    }

    /**
     * Returns how far to look backwards in time for the downstream end
     * @return {@link Link#getLength()}/{@link Link#getWaveSpeed()} (s)
     */
    public int getDSLookBehind()
    {
        return (int)Math.ceil(getLength()/getWaveSpeed()*3600 / Network.dt);
    }

    // cast as signal
    public TrafficSignal getSignal() {

        if(this.getDest().getSignal() != null){
            return (TrafficSignal)this.getDest().getSignal();
        }
        else {
            return null;
        }
    }

    private double calculateN(double ti, double xi) {
        double t_intersect_w;
        double t_intersect_ff;

        // set eq1 and eq2, set eq3 and 4 equal to max initially


        double eq3 = Integer.MAX_VALUE;

        double eq4 = Integer.MAX_VALUE;

        if (xi <= 0) {
            return this.getN_up(ti);
        }
        if (xi >= this.getLength()) {
            return this.getN_down(ti);
        }

        double eq1 = this.getN_up(ti - xi / this.getFPSFreeFlowSpeed());
        double eq2 = this.getN_down(ti - (this.getLength() - xi) / this.getFPSWaveSpeed()) + (this.Q.kj / 5280) * (this.getLength() - xi);

        if (this.bottlenecks.isEmpty()) {
            return (int)Math.min(eq1, eq2);
        }
        else {
            Trajectory ahead = this.getNextBottleneck(ti, xi);
            Trajectory behind = this.getPrevBottleneck(ti, xi);

                if (ahead == behind && ahead != null && behind != null && ahead.getArrTime() + ahead.getCurrentTimeInLink() == Simulator.time && behind.getArrTime() + behind.getCurrentTimeInLink() == Simulator.time) {
                    return ahead.getCount();
                }

                if (ahead != null && ahead.getArrTime() != Simulator.time) {
                    t_intersect_w = this.traceCongestedCharacteristic(ti, xi, ahead);
                    if (t_intersect_w > 0) {
                        Segment seg_ahead = ahead.getSegment(t_intersect_w);
                        if (seg_ahead != null) {
                            eq3 = linearInterpolate(seg_ahead.x_start, seg_ahead.cc_start, seg_ahead.x_end, seg_ahead.cc_end, t_intersect_w) + (this.Q.kj / 5280) * (ahead.findPosition(t_intersect_w) - xi);
                        }
                    }
                }
                else {
                    eq3 = this.getN_up(indexTime(traceCongestedCharacteristic(ti, xi, null)) - 1);
                }


            if (behind != null && behind.getArrTime() != Simulator.time) {
                t_intersect_ff = traceUncongestedCharacteristic(ti, xi, behind);
                if (t_intersect_ff > 0) {
                    Segment seg_behind = behind.getSegment(t_intersect_ff);
                    if (seg_behind == null) {
                        eq4 = behind.getCount();
                    } else {
                        eq4 = linearInterpolate(seg_behind.x_start, seg_behind.cc_start, seg_behind.x_end, seg_behind.cc_end, t_intersect_ff) - 1;
                    }
                }
            }
            else {
                eq4 = this.getN_up((int) (this.traceUncongestedCharacteristic(ti, xi, null) / 10) * 10);
            }

            // add JAM DENSITY EQ to eq3
            double min = Math.min(Math.min(eq1, eq2), Math.min(eq3, eq4));
            if (min > 0) {
                return min;
            }
            else return 0;
        }
    }

    // returns time of intersection
    private double traceUncongestedCharacteristic(double ti, double xi, Trajectory behind) {
        double ffspd = this.getFPSFreeFlowSpeed();
        if (behind == null) {
            return (-1 * xi + ffspd * ti) / ffspd;
        }

        try {
            Segment segment = behind.getPiecewise().getLast();
            return (-1 * ffspd * ti + xi - segment.y_start) / (segment.slope - ffspd);
        }
        catch (Exception e) {
            return -1;
        }
    }

    // returns time of intersection
    private double traceCongestedCharacteristic(double ti, double xi, Trajectory ahead) {
        double t;
        double wspd = this.getFPSWaveSpeed();

        if (ahead == null) {
            return -1;
        }

        for (Segment segment : ahead.getPiecewise()) {
            t = (wspd * (ti - ahead.getArrTime()) + xi + segment.y_start) / (segment.slope + wspd);
            if (t >= segment.x_start && t <= segment.x_end) {
                return t;
            }
        }

        return -1;
    }

    //x_k is the location of MB
    private int calcN_bottleneck(double t_k, double x_k, double control_speed, double cc_prev) {
        if (x_k >= this.getLength() + 1) {
            return (int)cc_prev;
        }
        double eq1 = this.getN_up( Math.ceil( t_k - x_k / this.getFPSFreeFlowSpeed()) );
        double eq2 = cc_prev ; // passing rate should be also added
        double eq3 = this.getN_down(t_k - ((this.getLength() - x_k) / this.getFPSWaveSpeed())) + (getJamDensity() / 5280) * (this.getLength() - x_k);

        // double passing = this.getPassingRate(t_k, x_k, control_speed / 5280 * 3600);

//        System.out.println("calcN_bottleneck("+t_k+", "+x_k+", "+cc_prev+")"+
//                "\n\teq1 = "+eq1+
//                "\n\teq2 = "+eq2+
//                "\n\teq3 "+eq3);

        int cc = (int)Math.min(Math.min(eq1, eq2), eq3);

        return cc;
    }

    private double linearInterpolate(double x1, double y1, double x2, double y2, double input) {
        if (x1 == x2) {
            return -1;
        }
        if (y1 == y2) {
            return y1;
        }
        double slope = Math.abs((y2 - y1) / (x2 - x1));
        return slope * input;
    }

    private Trajectory getPrevBottleneck(double t0, double x0) {
        Trajectory prev_bottleneck = null;

        for (Trajectory prev_bottleneck_min : this.bottlenecks) {
            if (prev_bottleneck_min.getCurrentTimeInLink() + prev_bottleneck_min.getCurrentTimeInLink() <= t0 && prev_bottleneck_min.getCurrentPosition() <= x0) {
                prev_bottleneck = prev_bottleneck_min;
            }
        }

        return prev_bottleneck;
    }

    private Trajectory getNextBottleneck(double t0, double x0) {
        Trajectory next_bottleneck = null;

        for (Trajectory next_bottleneck_max : this.bottlenecks) {
            if (next_bottleneck_max.getCurrentTimeInLink() + next_bottleneck_max.getArrTime() >= t0 && next_bottleneck_max.getCurrentPosition() >= x0) {
                return next_bottleneck_max;
            }
        }
        return next_bottleneck;
    }

    public int getCount() {
        return count;
    }

    // within +-200ft of target position, in veh / mi
    public double getDensity(double time, double position) {
        double cc1 = this.calculateN(time, Math.max(position - 200, 0));
        double cc2 = this.calculateN(time, Math.min(position + 200, this.getLength()));
        // divide by length
        return (cc1 - cc2) / 400 / 5280;
    }

    public double getFlow (double time, double position) {
        return this.Q.calculateQ(this.getDensity(time, position));
    }

    public double getTrafficSpeed(double time, double position) {
        return this.Q.getTrafficSpeed(this.getDensity(time, position));
    }


    // returns control speed based on the location and speed in ft/s
    public double calcControlSpeed(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();
        double control_speed = this.getFFSpeed();
        double arr_time = this.calcArrivalTime(trajectory);
        double effective_time = 0;
        if (signal != null) {
            if (this.maxDepart(trajectory) < this.numVehInFront(trajectory)) {  // avaible veh more than max: use second effective time
                effective_time = this.secondEffectiveTime(trajectory);
                trajectory.setEffectiveTime(effective_time);
//                System.out.println("second effective time " + this.maxDepart(trajectory));
            }
            else {     // all preceding vehicles will exit before MB arrival
                effective_time = this.firstEffectiveTime(trajectory);
                trajectory.setEffectiveTime(effective_time);
//                System.out.println("first effective time");
            }
        }
        if (signal != null) {
            if (this.getLength() - trajectory.getCurrentPosition() < 1500) { // within the connectivity range
                control_speed = (this.getLength() - trajectory.getCurrentPosition()) / (effective_time - 3);  // set a buffer of 3 seconds to ensure not falling at the end of it
            }
            else{ // not in the range yet
                control_speed = this.getFFSpeed() * 5280 / 3600;  // to be edited, replace the speed at that location instead of ff speed
            }
//            System.out.println("eff time " + effective_time + " spacing " +  (this.getLength() - trajectory.getCurrentPosition()));

        }
        else { //no traffic signal
            control_speed = this.getFPSFreeFlowSpeed();
        }
        if (this.isCongested(trajectory)) { // ensure control speed is lower than traffic speed if congested
            control_speed = Math.min(control_speed, this.getFlow(trajectory.getCurrentTimeInLink() + trajectory.getArrTime(), trajectory.getCurrentPosition()) / this.getDensity(trajectory.getCurrentTimeInLink() + trajectory.getArrTime(), trajectory.getCurrentPosition()) );
        }

        if (this.getLength() - trajectory.getCurrentPosition() <= 14 * Network.dt) { // last 140 fts
            if (this.getSignal() != null) {
                if (this.getDest().getSignal() == STOPSIGN) {
                    control_speed = (this.getLength() - trajectory.getCurrentPosition()) / Network.dt;
                }
                else if (this.getSignal().isRed()) { // control speed to be zero if red and we are close to stop bar
//                    System.out.println("control speed is 0 due to red");
                    control_speed = 0;
                }
            }
        }

        if (trajectory.getCurrentPosition() >= this.getLength()) {
            control_speed = 0;
        }

        //            System.out.println("  1 control speed " + control_speed);
        //        else if (control_speed < this.getWaveSpeed()) {
        //            return this.getWaveSpeed();
        //        }
        //            System.out.println(" fianlle 2 control speed " + control_speed);
        return Math.min(control_speed, this.getFPSFreeFlowSpeed());
    }

    public double calcArrivalTime(Trajectory trajectory) {
        if (this.getLength() - trajectory.getCurrentPosition() > 500) { // far from link downstream
            return Math.min((this.getLength() - trajectory.getCurrentPosition()) / (trajectory.getCurrentSpeed() ), (this.getLength() - trajectory.getCurrentPosition()) / (getFPSFreeFlowSpeed() * 0.6) ); // set an upper bound for travel time to ensure mobility
        }
        else {
            return Math.min((this.getLength() - trajectory.getCurrentPosition()) / (trajectory.getCurrentSpeed() ), (this.getLength() - trajectory.getCurrentPosition()) / (this.getFPSFreeFlowSpeed() * 0.05)); // set an upper bound for travel time to ensure mobility}
        }
        }


    // returns the number of signal cycles the vehicle will have in horizon whithin arrival
    public int calcNumCycles(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();
        // get the total signal length
        if (signal != null) {
            return (int) Math.ceil((this.calcArrivalTime(trajectory) - signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) / signal.gettotcycle());
        }
        else{ // there is no signal, pass as soon as possible
            return 0;
        }
    }

    // returns first effective time (sum of green times)
    public double firstEffectiveTime(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();
        int m;
        if (this.calcNumCycles(trajectory) == 0) {
            m = 0;
        }
        else {
            m = -1;
        }

        if (signal == null) {
            return this.calcArrivalTime(trajectory);
        }
//        System.out.println(" num of cycles " + this.calcNumCycles(trajectory) + " estimated arrival time " + this.calcArrivalTime(trajectory) +  " signal green remained " + signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink()));
        return signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink()) + this.calcNumCycles(trajectory) * signal.gettotcycle() + signal.getCurrentPhase().getGreenTime() * m;
    }

    // estimate number of vehicles in front of AV
    public int numVehInFront(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();

        if (signal != null) {  // if we have a signal, use equation 22
            int n_depart;   // UB number of vehicles can potentially the intersection at this time step
            if (this.getSignal().isGreen(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) {
                double v = this.Q.q_max / 3600 * Network.dt;
                n_depart = (int) v;
            } else {
                n_depart = 0;
            }
            return (int) Math.max(0,  (trajectory.getCount() - this.getN_down(Simulator.time) - Math.max(this.getNumSendingFlow(), n_depart)) );
        } else { // otherwise, just the sending flow
            return (int) this.getNumSendingFlow();
        }
    }
    // max # of veh that can exit before arrival

    public int maxDepart(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();

        if(signal != null)
        {
            if (signal.isGreen(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) {
                return (int) ((this.Q.q_max / 3600) * (signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink()) + ((this.calcNumCycles(trajectory)) * signal.getgreenduration())));
            }
            else { // in red phase, we have a full green phase
                return (int) ((this.Q.q_max / 3600) * (this.calcNumCycles(trajectory) * signal.getgreenduration()) + signal.getgreenduration());
            }
        }
        // otherwise, just the capacity
        return (int) this.Q.q_max / 3600;
    }

    public double greenGrossTime(Trajectory trajectory) {
        return (double) numVehInFront(trajectory) / (this.Q.q_max / 3600);
    }

    public double secondEffectiveTime(Trajectory trajectory) {
        TrafficSignal signal = this.getSignal();

        if (signal == null) {
            return this.calcArrivalTime(trajectory);
        }
        if (signal.isGreen(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) {
            return greenGrossTime(trajectory) + Math.ceil((greenGrossTime(trajectory) - signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) / signal.getgreenduration()) * signal.getredduration();
        }
        else {
            return greenGrossTime(trajectory) + Math.ceil((greenGrossTime(trajectory) - signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink())) / signal.getgreenduration()) * signal.getredduration() + (signal.greenTimeRemaining(Simulator.time, trajectory.getVehicle().getCurrLink(), trajectory.getVehicle().getNextLink()) - signal.getgreenduration());
        }
    }



    public double vehicleSearch(double time, double position) {
        double length = 2 * 19 + 9 + 1;
        double x1 = position - length;
        double x2 = position + length;
        double av_count = 0;
        for (Trajectory bottleneck : this.bottlenecks) {
            if (bottleneck.getCurrentPosition() >= x1 && bottleneck.getCurrentPosition() <= x2) {
                av_count++;
            }
        }
        double total_count = this.getDensity(time, position) / 400 * 2 * length;

        return (av_count / total_count) * this.getUsLanes();
    }

    public boolean isCongested (Trajectory trajectory) {
        List<Segment> pos = trajectory.getPiecewise();
        double xi_prev = trajectory.getPreviousPosition();
        double xi = trajectory.getCurrentPosition();
        double ti = trajectory.getCurrentTimeInLink() + trajectory.getArrTime();

        if (Simulator.time == trajectory.getArrTime()) {
            return false;
        }

        if (trajectory.getCurrentSpeed() == 0) {
            return true;
        }

        return Math.max(this.calculateN(ti - (xi - xi_prev) / trajectory.getCurrentSpeed(), xi_prev), this.getN_down(ti - (this.getLength() - xi) / this.getFPSWaveSpeed()) + this.getJamDensity()/5280 * (this.getLength() - xi)) == this.calculateN(ti - (xi - xi_prev) / trajectory.getCurrentSpeed(), xi_prev);
    }


    // MAKE NEAREST INTEGER, fractional value good
    public double getPassingRate(Trajectory trajectory) {
        return Math.min((this.getTrafficSpeed(Simulator.time, trajectory.getCurrentPosition()) - trajectory.getCurrentSpeed()) * this.getDensity(Simulator.time, trajectory.getCurrentPosition()),
                Math.max(this.getQ_max(trajectory) * ((this.getUsLanes() - this.vehicleSearch(Simulator.time, trajectory.getCurrentPosition())) / this.getUsLanes()), 0));
    }

    public double getQ_max(Trajectory trajectory) {
        double xi = trajectory.getCurrentPosition();
        if (this.isCongested(trajectory)) {
            return this.getWaveSpeed() * (this.getJamDensity() - this.getDensity(Simulator.time, xi));
        }
        else {
            return this.Q.q_max - (this.getDensity(Simulator.time, xi) * (this.getTrafficSpeed(Simulator.time, xi)));
        }

    }

    private double calcFFBoundary(LinkedList<Segment> segments) {
        boolean found = false;

        double t1;
        double num;
        double den;

        Segment xt = segments.getLast();
        double length = this.getLength();
        double ffSpd = this.getFFSpeed() * 5280 / 3600;

        double upper_bound;
        double lower_bound;


        num = length - xt.y_start - ffSpd * xt.x_end + xt.slope * xt.x_start;
        den = xt.slope - ffSpd;

        t1 = num / den;

        lower_bound = xt.x_start;
        upper_bound = xt.x_end;

        if (xt.y_start >= length) {
            found = true;
            t1 = 0;
        }

        if (t1 >= lower_bound && t1 <= upper_bound)
        {
            found = true;
        }
        if (!found || t1 < lower_bound || t1 > upper_bound)
        {
            for(int i = 1; i <= indexTime(xt.x_start); i++)
            {
                Segment xt_prev = segments.get(segments.size() - i - 1);

                num = length - xt_prev.y_start - ffSpd * xt_prev.x_end + xt_prev.slope * xt_prev.x_start;
                den = xt_prev.slope - ffSpd;

                t1 = num / den;

                lower_bound = xt_prev.x_start;
                upper_bound = xt_prev.x_end;

                if (t1 >= lower_bound && t1 <= upper_bound)
                {
                    found = true;
                }
            }
        }
        if (!found)
        {
            t1 = -1;
        }
        return t1;
    }

    private double calcWBoundary(LinkedList<Segment> segments) {
        boolean found = false;

        double t2;
        double num;
        double den;

        Segment xt = segments.getLast();
        double wSpd = this.getFPSWaveSpeed();

        double upper_bound;
        double lower_bound;

        num = xt.y_start - wSpd * xt.x_end - xt.slope * xt.x_start;
        den = -1 * wSpd - xt.slope;

        t2 = num / den;

        lower_bound = xt.x_start;
        upper_bound = xt.x_end;

        if (t2 >= lower_bound && t2 <= upper_bound)
        {
            found = true;
        }
        if (!found || t2 < lower_bound || t2 > upper_bound)
        {
            for(int i = 1; i <= indexTime(xt.x_start); i++)
            {
                Segment xt_prev = segments.get(segments.size() - i - 1);

                num = xt_prev.y_start - wSpd * xt.x_end - xt_prev.slope * xt_prev.x_start;
                den = -1 * wSpd - xt_prev.slope;

                t2 = num / den;

                lower_bound = xt_prev.x_start;
                upper_bound = xt_prev.x_end;

                if (t2 >= lower_bound && t2 <= upper_bound)
                {
                    found = true;
                }
            }
        }
        if (!found)
        {
            t2 = 0;
        }

        return t2;
    }

    public String dataToString()
    {
        return "Link ID : "+this.getId()+"\n\tlength = "+this.getLength()+"\n\tcapacity = "+this.getCapacity()+"\n\tffspd = "+this.getFFSpeed()+"\n\twavespd = "+this.getWaveSpeed()+"\n\tjamd = "+this.getJamDensity();
    }

    public Trajectory getUSBottleneck() {
        Trajectory us_bottleneck;

        try {
            us_bottleneck = this.bottlenecks.getLast();
        }
        catch (Exception e) {
            us_bottleneck = null;
        }

        return us_bottleneck;
    }

    public Trajectory getDSBottleneck() {
        Trajectory ds_bottleneck;

        try {
            ds_bottleneck = this.bottlenecks.getFirst();
        }
        catch (Exception e) {
            ds_bottleneck = null;
        }

        return ds_bottleneck;
    }

    public Double getCurrentDSBoundaryCondition() {
        return this.N_boundary_down.getLast();
    }

    public Double Maz_getCurrentDSBoundaryCondition() {
        return this.Maz_N_boundary_down;
    }

    private static void createWindow() {
    }

    public Double getCurrentUSBoundaryCondition() {
        return this.N_boundary_up.getLast();
    }

    public double getVehicleLength()
    {
        return 19;
    }

    public double getMinimumGap()
    {
        return 8.0;
    }

    // includes fractions lost to discretization
    /**
     * Returns the current upstream capacity. This includes fractions lost to discretization.
     * @return the current upstream capacity
     */
    public double getCurrentUpstreamCapacity()
    {
        return capacityUp;
    }
    
    /**
     * Returns the current downstream capacity. This includes fractions lost to discretization.
     * @return the current downstream capacity
     */
    public double getCurrentDownstreamCapacity()
    {
        return capacityDown;
    }    
    
    /**
     * Returns the type code of this link
     * @return {@link ReadNetwork#LTM}
     */
    public Type getType()
    {
        return MOVING_BOTTLENECK;
    }
    /**
     * Returns the number of {@link Vehicle}s on this link
     * @return the number of {@link Vehicle}s on this link
     */
    public int getOccupancy()
    {
        return queue.size();
    }
    
    /**
     * Returns the queue of {@link Vehicle}s. 
     * The queue is stored as a {@link LinkedList} of {@link }s, which contain the {@link Vehicle} positions used to determine sending flows.
     * @return the queue of {@link Vehicle}s
     */
    public LinkedList<VehTime> getQueue()
    {
        return queue;
    }

    /**
     * Returns the receiving flow for this time step
     * @return the receiving flow for this time step
     */
    public double getReceivingFlow()
    {
        double receiving_boundary;
        double xjt2;

        if (!this.bottlenecks.isEmpty()) {
            Trajectory us_bottleneck = this.getUSBottleneck();
            receiving_boundary = this.getCurrentUSBoundaryCondition();
            xjt2 = Math.max(us_bottleneck.findPosition(us_bottleneck.getCurrentWaveIntersect()), us_bottleneck.getCurrentPosition()-10) ;
        }
        // make sure truly is no vehicle
        else {
            receiving_boundary = getN_up(Simulator.time + Network.dt - this.getLength() / this.getFPSWaveSpeed());
            xjt2 = 0;
        }
        double eq1 = receiving_boundary + (this.getJamDensity() / 5280) * ( this.getLength() -xjt2)  - this.getN_up(Simulator.time);
        double eq2 = this.getN_down(Simulator.time + Network.dt - this.getLength() / this.getFPSWaveSpeed()) + (this.getJamDensity() / 5280) * this.getLength() - this.getN_up(Simulator.time);
        double eq3 = this.capacityUp;

//        System.out.println( "Receiving for " + " Link "+this.getId()+" Receiving = "+
//                "\n\teq1 = "+eq1+
//                "\n\teq2 = "+eq2+
//                "\n\teq3 "+eq3);

        return Math.min(Math.min(eq1, eq2), eq3);
    }


    // phases do not restart
    // moves during correct step, but now have to adjust things in update and step to accomodate when the vehicle leaves
    /**
     * Returns the number of {@link Vehicle}s that could exit this link.
     * @return the size of the sending flow
     */
    public int getNumSendingFlow()
    {
        double epsilon = 0.01;
        double sending_boundary;
        TrafficSignal signal = this.getSignal();

        if (signal != null) {
//            System.out.println( Simulator.time + " signal is red " + signal.isRed() + "link " + this.getId());
            if (signal.isRed()) {
                return 0;
            }
        }

        if (!bottlenecks.isEmpty()) {  // MB is also loaded
//            System.out.println("calc N " + this.calculateN(Simulator.time, getLength() - bottlenecks.getFirst().getCurrentSpeed()*Network.dt - 10 ));
            Trajectory sending_trajectory = this.getDSBottleneck();
            if (Maz_N_boundary_down != 0) { // sending boundary is cc of the MB
                sending_boundary = Math.max( Maz_N_boundary_down, this.calculateN(Simulator.time, getLength() - bottlenecks.getFirst().getCurrentSpeed()*Network.dt - 10 ) ) ;   // Math.max( Maz_N_boundary_down, this.calculateN(Simulator.time, getLength() - calcControlSpeed(bottlenecks.getLast())*Network.dt - 10 ));  // get the max of MB cc and all vehicles following it that can pass
            }
            else { // sending boundary is the vehicle in front of the MB
                sending_boundary =  MBCC_lowest - 1;
            }
                    //this.getCurrentDSBoundaryCondition();

//            if (sending_trajectory.getCurrentPosition() < this.getLength() && sending_trajectory.getCount() - this.getN_down(Simulator.time) == 0) {
//                sending_boundary = 0;
//            }
        }
        else {  // All HVs
            sending_boundary = getN_up(Simulator.time - getLength()/getFPSFreeFlowSpeed() + Network.dt);
        }
        double eq1 = Math.max(0 , sending_boundary - getN_down(Simulator.time));
        double eq2 = this.getN_up(Simulator.time - getLength()/getFPSFreeFlowSpeed() + Network.dt) - getN_down(Simulator.time);
        double eq3 = this.capacityDown;
        if (this.getId() == 1) {
//        System.out.println( "N_up " + this.getN_up(Simulator.time - getLength()/getFPSFreeFlowSpeed() + Network.dt) + " N_Down " + getN_down(Simulator.time) );
        System.out.println("Sendin flow " + Simulator.time + " MB " + !bottlenecks.isEmpty() + " Link " + this.getId() + " Sending = " +
                "\n\teq1 = " + eq1 +
                "\n\teq2 = " + eq2 +
                "\n\teq3 " + eq3);
        System.out.println("final Send" + Math.min(eq1, eq2));
        }
        return (int)Math.floor(Math.min(Math.min(eq1, eq2), eq3));
    }

    /**
     * Returns the set of {@link Vehicle}s that could exit this link
     * @return the sending flow
     */
    public List<Vehicle> getSendingFlow()
    {
        List<Vehicle> output = new ArrayList<Vehicle>();

        int max = getNumSendingFlow();

        for(VehTime vt : queue) {
            if(max > 0) {

                output.add(vt.vehicle);
                if(vt.vehicle.arr_time < 0) {

                    vt.vehicle.arr_time = Simulator.time;
                }

                max--;
            }

            else {
                break;
            }
        }

        S = output.size();
        return output;
    }

    /**
     * Adds the {@link Vehicle} to this link
     * @param veh the {@link Vehicle} to be added
     */
    public void addVehicle(Vehicle veh)
    {
//        System.out.println("~~~ ADD VEHICLE "+veh.getId() + "isMB" + (veh.isBottleneck()));
        veh.enteredLink(this);
        queue.add(new VehTime(veh, Simulator.time));
        cc_saver.addCC(count-1, count +1);
        if (veh.isBottleneck()) {
            this.bottlenecks.add(new Trajectory(queue.getLast()));
            this.bottlenecks.getLast().updateSpeed(this.calcControlSpeed(this.bottlenecks.getLast() ) );  // should be edited: put control speed value instead
            ccMB_saver.addCC(count-1, count+1);
            this.bottlenecks.getLast().trajectory_cc_updated(count+1);
        }



        else {
            ccMB_saver.addCC(count-1, 0);  // non-MB vehicles have value of 0
        }
        this.addN_up(Simulator.time, 1);

//        System.out.println("cc new saved " + ccMB_saver);
        count++;
    }

    /**
     * Removes the {@link Vehicle} from this link
     * @param veh the {@link Vehicle} to be removed
     * @return if the {@link Vehicle} was removed
     */
    public boolean removeVehicle(Vehicle veh)
    {
        Iterator<VehTime> iter = queue.iterator();

        while(iter.hasNext())
        {
            VehTime vt = iter.next();

            if(vt.vehicle == veh)
            {
                if(!this.bottlenecks.isEmpty()) {
                    if (vt.vehicle.isBottleneck()) {
                        Trajectory traj = this.getTrajectory(veh);
                        traj.clear();
                        bottlenecks.remove(traj);
                    }
                }
                iter.remove();
                exited++;
                addN_down(Simulator.time, 1);
                updateTT(vt.vehicle);
                System.out.println(" vehicle leaving " + vt.vehicle);
                return true;
            }
        }

        return false;
    }

    public Trajectory getTrajectory(Vehicle veh) {
        for (Trajectory trajectory : this.bottlenecks) {
            if (trajectory.getVehicle() == veh) {
                return trajectory;
            }
        }
        return null;
    }

    /**
     * Returns the sending flow
     * @return {@link LTMLink#getVehiclesCanMove()}
     */
    public List<Vehicle> getVehiclesCanMove()
    {
        return getSendingFlow();
    }
    
    /**
     * Returns the number of vehicles waiting to exit, i.e. the component of sending flow unbounded by capacity.
     * @return the number of vehicles waiting to exit
     */
    public int getNumWaiting()
    {
        return getN_up(Simulator.time - getLength()/getFPSFreeFlowSpeed()*3600 + Network.dt) - getN_down(Simulator.time);
    }

    /**
     * Adds to the upstream cumulative count.
     * @param t the time (s)
     * @param val the number of vehicles to add
     */
    public void addN_up(double t, int val) {
        this.N_up.addCC(indexTime(t), val);
    }
    
    /**
     * Adds to the downstream cumulative count.
     * @param t the time (s)
     * @param val the number of vehicles to add
     */
    public void addN_down(double t, int val) {
        this.N_down.addCC(indexTime(t), val);
    }

    /**
     * Returns the upstream cumulative count at the specified time
     * @param t the time (s)
     * @return the upstream cumulative count
     */

    public int getN_up(double t) {
        if(t < 0)
        {
            return 0;
        }
        else
        {   // wait one step when getting the count at upstream
            return N_up.getCC(indexTime(t - Network.dt));
        }
    }

    public int getExited() {
        return exited;
    }


    // N(x_L, t) = N(x_L, t-1) + y where y is the # vehicles exiting
    // USE NEWELLS METHOD
    public double getN_downstreamMBL() {
        return this.getExited() + this.getN_down(Simulator.time - Network.dt);
    }
    /**
     * Returns the downstream cumulative count at the specified time
     * @param t the time (s)
     * @return the downstream cumulative count
     */
    //write maziar's math for N_down (#13 & #14)
    public int getN_down(double t)
    {
        if(t < 0)
        {
            return 0;
        }
        else
        {
            return N_down.getCC(Simulator.indexTime(t));
        }
    }


    private double getFPSWaveSpeed() {
        return this.getWaveSpeed() * 5280 / 3600;
    }

    // returns free flow speed in ft/s
    private double getFPSFreeFlowSpeed() {
        return this.getFFSpeed() * 5280 / 3600;
    }


    class MovingBottleNeckLTMIterable implements Iterable<Vehicle>
    {
        public Iterator<Vehicle> iterator()
        {
            return new MovingBottleneckLTMIterator(queue);
        }
    }

}



class MovingBottleneckLTMIterator implements Iterator<Vehicle>
{
    private Iterator<VehTime> iter;

    public MovingBottleneckLTMIterator(LinkedList<VehTime> queue)
    {
        this.iter = queue.iterator();
    }
    
    public boolean hasNext()
    {
        return iter.hasNext();
    }
    
    public Vehicle next()
    {
        VehTime next = iter.next();
        return next.vehicle;
    }
    
    public void remove()
    {
        iter.remove();
    }

}



