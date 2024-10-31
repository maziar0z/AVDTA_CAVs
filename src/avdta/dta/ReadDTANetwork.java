/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.dta;

import avdta.demand.ReadDemandNetwork;
import avdta.network.PathList;
import avdta.network.ReadNetwork;
import avdta.network.Simulator;
import avdta.network.link.Link;
import avdta.network.node.Node;
import avdta.network.node.Zone;
import avdta.project.DTAProject;
import avdta.traveler.Traveler;
import avdta.vehicle.VOT;
import avdta.vehicle.DriverType;
import avdta.vehicle.EmergencyVehicle;
import avdta.vehicle.PersonalVehicle;
import avdta.vehicle.wallet.StaticWallet;
import avdta.vehicle.Vehicle;
import avdta.vehicle.wallet.Wallet;
import avdta.vehicle.fuel.VehicleClass;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * This adds methods to read the trip table for DTA. 
 * This also contains the {@link ReadDemandNetwork#prepareDemand(avdta.project.DemandProject, double)} method to generate demand.
 * @author Michael
 */
public class ReadDTANetwork extends ReadDemandNetwork
{
   
    
    /**
     * Constructs the {@link ReadDTANetwork}
     */
    public ReadDTANetwork()
    {
        super();
    }
    
    /**
     * Constructs a {@link DTASimulator} for the given {@link DTAProject}. 
     * This calls {@link ReadNetwork#readNodes(avdta.project.Project)} and {@link ReadNetwork#readLinks(avdta.project.Project)} and initializes the simulator ({@link Simulator#initialize()}).
     * This also reads transit ({@link ReadDTANetwork#readTransit(avdta.project.TransitProject)} and personal vehicles ({@link ReadDTANetwork#readVehicles(avdta.project.DTAProject)}).
     * @param project the {@link DTAProject}
     * @return the created {@link DTASimulator}
     * @throws IOException if a file cannot be accessed
     */
    public DTASimulator readNetwork(DTAProject project) throws IOException
    {
        System.out.println("ReadDTANetwork.readNetwork called");
        readOptions(project);
        Set<Node> nodes = readNodes(project);
        Set<Link> links = readLinks(project);
        
        readIntersections(project);
        readPhases(project);
        
        
        DTASimulator sim = new DTASimulator(project, nodes, links);

        readVehicles(project, sim);
        
        sim.initialize();
        
        
        
        return sim;
    }
    
    /**
     * Reads in the list of vehicles, including both transit and personal vehicles.
     * @param project the project
     * @param sim the simulator
     * @throws IOException if a file cannot be accessed
     */
    public void readVehicles(DTAProject project, DTASimulator sim) throws IOException
    {
        vehicles = new ArrayList<Vehicle>();
        readTransit(project);
        readVehicles(project);
        sim.setVehicles(vehicles);
    }
    
    
    
    
    
    /**
     * This reads the demand file into a list of {@link Vehicle}s.
     * @param project the project
     * @return the list of {@link Vehicle}s.
     * @throws IOException if a file cannot be accessed
     */
    public List<Vehicle> readVehicles(DTAProject project) throws IOException
    {
        
        
        Scanner filein = new Scanner(project.getDemandFile());
        
        filein.nextLine();
        
        while(filein.hasNext())
        {
            int id = filein.nextInt();
            int type = filein.nextInt();
            int origin_id = filein.nextInt();
            
            // take the negative id to get the destination zone
            int dest_id = -filein.nextInt();
            int dtime = filein.nextInt();
            double vot = filein.nextDouble();

  
            if(type/100 == EMERGENCY_VEHICLE/100)
            {
                Zone origin = (Zone)nodesmap.get(origin_id);
                Zone dest = (Zone)nodesmap.get(dest_id);
                
                
                vehicles.add(new EmergencyVehicle(id, origin, dest, dtime));
            }
            else if(type / 100 == DA_VEHICLE/100)
            {
                
            
                Wallet wallet = new StaticWallet(vot);
            
                Zone origin = (Zone)nodesmap.get(origin_id);
                Zone dest = (Zone)nodesmap.get(dest_id);
                
                if(origin == null)
                {
                    throw new RuntimeException("Origin is null: "+origin_id);
                }
                
                if(dest == null)
                {
                    throw new RuntimeException("Dest is null: "+dest_id);
                }
                
                //origin.addProductions(1);
                //dest.addAttractions(1);
                
                VehicleClass vehClass = null;
                DriverType driver = null;
                

                
                switch(type % 10)
                {
                    case ICV:
                        vehClass = VehicleClass.icv;
                        break;
                    case BEV:
                        vehClass = VehicleClass.bev;
                        break;
                    default:
                        throw new RuntimeException("Vehicle class not recognized - "+type);
                }
                
                switch((type / 10 % 10)*10)
                {
                    case HV:
                        driver = DriverType.HV;
                        break;
                    case AV:
                        driver = DriverType.AV;
                        break;
                    default:
                        throw new RuntimeException("Vehicle class not recognized - "+type);
                }
                vehicles.add(new PersonalVehicle(new Traveler(id, origin, dest, dtime, vot), vehClass, driver));
            }
        }
        

        filein.close();

        return vehicles;
    }
    
    
    
    
}