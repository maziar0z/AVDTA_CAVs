/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.gui.panels.dta;

import avdta.dta.Assignment;
import avdta.dta.DTAResults;
import avdta.dta.DTASimulator;
import avdta.dta.MSAAssignment;
import avdta.gui.GUI;
import avdta.gui.panels.GUIPanel;
import avdta.gui.util.StatusBar;
import avdta.project.DTAProject;
import avdta.vehicle.DriverType;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import static avdta.gui.util.GraphicUtils.*;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

/**
 *
 * @author ml26893
 */
public class MSAPanel extends GUIPanel
{
    
    private DTAProject project;
    
    private StatusBar status;
    
    private JButton run;
    private JButton fftime;
    
    private JTextField max_iter, min_gap;
    private JTextField start_iter;
    private JTextArea data;
    
    
    public MSAPanel(DTAPanel parent)
    {
        super(parent);
        
        max_iter = new JTextField(5);       
        start_iter = new JTextField(5);
        min_gap = new JTextField(5);
        data = new JTextArea(5, 20);
        data.setEditable(false);
        
        status = new StatusBar();
        run = new JButton("Run MSA");
        
        fftime = new JButton("Calc. FF time");
        
        run.addActionListener(new ActionListener()
        {
            public void actionPerformed(ActionEvent e)
            {
                run();
            }
        });
        
        fftime.addActionListener(new ActionListener()
        {
            public void actionPerformed(ActionEvent e)
            {
                ffTime();
            }
        });
        
        
        setLayout(new GridBagLayout());
        
        JPanel p = new JPanel();
        p.setLayout(new GridBagLayout());
        JScrollPane scroll = new JScrollPane(data);
        scroll.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
        
        constrain(this, scroll, 0, 0, 1, 1);
        constrain(this, fftime, 0, 1, 1, 1);
        
        constrain(p, new JLabel("Start iteration:"), 0, 0, 1, 1);
        constrain(p, start_iter, 1, 0, 1, 1);
        constrain(p, new JLabel("Max. iteration:"), 0, 1, 1, 1);
        constrain(p, max_iter, 1, 1, 1, 1);
        constrain(p, new JLabel("Min. gap:"), 0, 2, 1, 1);
        constrain(p, min_gap, 1, 2, 1, 1);
        constrain(this, p, 0, 2, 1, 1);
        constrain(this, run, 0, 3, 2, 1);
        
        constrain(this, status, 0, 4, 2, 1);
        
        setEnabled(false);
    }
    
    public void setEnabled(boolean e)
    {
        run.setEnabled(e);
        min_gap.setEditable(e);
        max_iter.setEditable(e);
        start_iter.setEditable(e);
        fftime.setEnabled(e);
        super.setEnabled(e);
    }

    
    public void setProject(DTAProject project)
    {
        this.project = project;
        
        data.setText("");
        
        if(project != null)
        {
            start_iter.setText("1");
            max_iter.setText("30");
            min_gap.setText("1");
            setEnabled(true);
        }
        else
        {
            start_iter.setText("");
            max_iter.setText("");
            min_gap.setText("");
            setEnabled(false);
        }
    }
    
    
    
    public void loadAssignment(Assignment assign)
    {
        if(assign instanceof MSAAssignment)
        {
            start_iter.setText(""+((MSAAssignment)assign).getIter());
        }
        
        showAssignment(assign);
    }
    
    public void showAssignment(Assignment assign)
    {
        if(assign != null)
        {
            showResults(assign.getResults());
        }
    }
    public void showResults(DTAResults results)
    {
        data.setText("");
        data.append("Gap:\t"+String.format("%.2f", results.getGapPercent())+"%\n");
        data.append("TSTT:\t"+String.format("%.1f", results.getTSTT())+" hr\n");
        data.append("Avg. TT:\t"+String.format("%.2f", results.getAvgTT())+" min\n");
        data.append("Non-exit:\t"+results.getNonExiting());
        
    }
    
    public void ffTime()
    {
        parentSetEnabled(false);
        
        
        final JPanel panel = this;
        
        Thread t = new Thread()
        {
            public void run()
            {

                try
                {
                    DTASimulator sim = project.getSimulator();
                    sim.setStatusUpdate(status);

                    double fftime = sim.getFFTT();

                    JOptionPane.showMessageDialog(panel, "FF TSTT: "+String.format("%.1f", fftime/3600.0)+" hr\n"+
                            "FF Avg. TT: "+String.format("%.2f", fftime/60.0 / sim.getNumVehicles())+" min",
                            "Simulation complete", JOptionPane.PLAIN_MESSAGE);

                    status.update(0, 0, "");
                    status.resetTime();

                }
                catch(Exception ex)
                {
                    GUI.handleException(ex);
                }
                
                parentReset();
                parentSetEnabled(true);
            }
        };
        t.start();
    }
    
    public void run()
    {
        try
        {
            Integer.parseInt(start_iter.getText().trim());
        }
        catch(Exception ex)
        {
            start_iter.setText("");
            start_iter.requestFocus();
            return;
        }
        
        try
        {
            Integer.parseInt(max_iter.getText().trim());
        }
        catch(Exception ex)
        {
            max_iter.setText("");
            max_iter.requestFocus();
            return;
        }
        
        try
        {
            Double.parseDouble(min_gap.getText().trim());
        }
        catch(Exception ex)
        {
            min_gap.setText("");
            min_gap.requestFocus();
            return;
        }
        
        
        parentSetEnabled(false);
        
        
        final JPanel panel = this;
        
        Thread t = new Thread()
        {
            public void run()
            {
                try
                {
                    
                    DTAResults results;
                    
                    DTASimulator sim = project.getSimulator();
                    sim.setStatusUpdate(status);
                    
                    int start = Integer.parseInt(start_iter.getText().trim());
                    int max = Integer.parseInt(max_iter.getText().trim());
                    double gap = Double.parseDouble(min_gap.getText().trim());
                    
                    if(start> 1)
                    {
                        results = sim.msa_cont(start, max, gap);
                    }
                    else
                    {
                        results = sim.msa(max, gap);
                    }
                    
                    
                    
                    double tstt = sim.getTSTT() / 3600.0;
                    double avg = tstt * 60 / sim.getNumVehicles();
                    
                    JOptionPane.showMessageDialog(panel, "Iterations: "+(sim.getIteration()-1)+"\nGap: "+String.format("%.2f", results.getGapPercent())+
                            "\nNon-exiting: "+results.getNonExiting()+"\t"+
                            "\nTSTT: "+String.format("%.1f", tstt)+" hr\nAvg. time: "+String.format("%.2f", avg)+" min\n"+
                            "HV TT: "+String.format("%.2f", sim.getAvgTT(DriverType.HV)/60)+" min\n"+
                            "AV TT: "+String.format("%.2f", sim.getAvgTT(DriverType.AV)/60)+" min",
                            "DTA complete", JOptionPane.PLAIN_MESSAGE);

                    showResults(results);
                    
                    status.update(0, 0, "");
                    status.resetTime();

                    parentReset();
                    parentSetEnabled(true);
                }
                catch(Exception ex)
                {
                    GUI.handleException(ex);
                }
                
                max_iter.setEnabled(true);
                min_gap.setEnabled(true);
                run.setEnabled(true);
            }
        };
        t.start();
    }
  
    public void reset()
    {
        max_iter.setText("30");
        start_iter.setText("1");
        min_gap.setText("1");
    }

}
