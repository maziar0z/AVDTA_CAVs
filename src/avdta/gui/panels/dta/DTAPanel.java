/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.gui.panels.dta;

import avdta.gui.panels.dta.AssignmentPanel;
import avdta.gui.panels.dta.MSAPanel;
import avdta.dta.Assignment;
import avdta.gui.DTAGUI;
import avdta.gui.panels.GUIPanel;
import avdta.project.DTAProject;
import javax.swing.JPanel;
import java.awt.GridBagLayout;
import static avdta.gui.util.GraphicUtils.*;

/**
 *
 * @author micha
 */
public class DTAPanel extends GUIPanel
{
    private AssignmentPanel assignPane;
    private MSAPanel msaPane;

    public DTAPanel(DTAGUI parent)
    {
        super(parent);
        
        assignPane = new AssignmentPanel(this);
        msaPane = new MSAPanel(this);
        
        setLayout(new GridBagLayout());
        constrain(this, assignPane, 0, 0, 1, 1);
        constrain(this, msaPane, 1, 0, 1, 1);
    }
    

    public Assignment getMostRecentAssignment()
    {
        return assignPane.getMostRecentAssignment();
    }
    
    public void setProject(DTAProject project)
    {
        assignPane.setProject(project);
        msaPane.setProject(project);
    }
    

    public void setEnabled(boolean e)
    {
        assignPane.setEnabled(e);
        msaPane.setEnabled(e);
        super.setEnabled(e);
    }
    
    public void reset()
    {
        assignPane.reset();
        msaPane.reset();
    }
    
    public void showAssignment(Assignment assign)
    {
        msaPane.showAssignment(assign);
    }
    public void loadAssignment(Assignment assign)
    {
        msaPane.loadAssignment(assign);
    }
}