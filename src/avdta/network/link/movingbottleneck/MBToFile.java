package avdta.network.link.movingbottleneck;

import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;

public class MBToFile {
    File link_file;
    File var_file;
    File bottleneck_file;

    public MBToFile (int link_id) {
        link_file = new File("Z:/research projects/LTM_Newell/AVDTA-master/output/"+link_id+"_output.csv");
        try {
            CSVWriter link_writer = new CSVWriter(new FileWriter(link_file));
            String[] link_header = {"Time", "Available R", "S", "# Vehicles Present"};
            link_writer.writeNext(link_header);
            link_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        var_file = new File("Z:/research projects/LTM_Newell/AVDTA-master/output/"+link_id+"traffic.csv");
        try {
            CSVWriter var_writer = new CSVWriter(new FileWriter(var_file));
            String[] var_header = {"time", "position", "CC", "Density", "Flow", "sending flow", "receiving flow"};
            var_writer.writeNext(var_header);
            var_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        bottleneck_file = new File("Z:/research projects/LTM_Newell/AVDTA-master/output/"+link_id+".csv");
        try {
            CSVWriter bottleneck_writer = new CSVWriter(new FileWriter(bottleneck_file));
            String[] bottleneck_header = {"vehicle", "time", "position", "speed", "count", "effective time"};
            bottleneck_writer.writeNext(bottleneck_header);
            bottleneck_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void writeToLinkFile(int time, double r, double s, int veh_present) {
        try {
            CSVWriter link_writer = new CSVWriter(new FileWriter(link_file, true));
            String[] link_line = {String.valueOf(time), String.valueOf(r), String.valueOf(s), String.valueOf(veh_present)};
            link_writer.writeNext(link_line);
            link_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void writeToVarFile(int time, double pos, double cc, double density, double flow,  int numSendingFlow, double receivingFlow) {
        try {
            CSVWriter var_writer = new CSVWriter(new FileWriter(var_file, true));
            String[] var_line = {String.valueOf(time), String.valueOf(pos), String.valueOf(cc), String.valueOf(density), String.valueOf(flow), String.valueOf(numSendingFlow), String.valueOf(receivingFlow) };
            var_writer.writeNext(var_line);
            var_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void writeToBottleneckFile(int veh_id, double time, double pos, double speed, double cc, double effective_time) {
        try {
            CSVWriter bottleneck_writer = new CSVWriter(new FileWriter(bottleneck_file, true));
            String[] bottleneck_line = {String.valueOf(veh_id), String.valueOf(time), String.valueOf(pos), String.valueOf(speed), String.valueOf(cc), String.valueOf(effective_time)};
            bottleneck_writer.writeNext(bottleneck_line);
            bottleneck_writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
//
//    public void writeToVarFile(int time, int i, double v, double density, double flow, int numSendingFlow, double receivingFlow) {
//    }
}
