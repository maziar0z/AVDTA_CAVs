package avdta;

import avdta.dta.DTASimulator;
import java.io.File;
import avdta.project.DTAProject;
import java.io.IOException;

import avdta.*;

import static avdta.network.Simulator.num_asts;


/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

/**
 *
 * @author michael
 */
public class Main {
    public static void main(String[] args) throws IOException
    {
        DTAProject project = new DTAProject(new File("LTMTrial"));

        DTASimulator sim = project.getSimulator();


        // sim diagnostics
        System.out.println();

        sim.msa(1);
    }
}
