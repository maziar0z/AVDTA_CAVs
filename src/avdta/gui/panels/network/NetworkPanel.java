/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.gui.panels.network;

import avdta.gui.panels.network.ImportNetworkPanel;
import avdta.gui.panels.network.LinksPanel;
import avdta.gui.DTAGUI;
import avdta.gui.GUI;
import avdta.gui.panels.AbstractGUIPanel;
import avdta.gui.panels.GUIPanel;
import avdta.gui.panels.network.NodesPanel;
import javax.swing.JPanel;
import static avdta.gui.util.GraphicUtils.*;
import avdta.project.DTAProject;
import avdta.project.Project;
import java.awt.GridBagLayout;

/**
 *
 * @author micha
 */
public class NetworkPanel extends GUIPanel
{
    
    private LinksPanel linksPane;
    private NodesPanel nodesPane;
    private ImportNetworkPanel importPane;
    
    public NetworkPanel(AbstractGUIPanel parent)
    {
        super(parent);
        
        linksPane = new LinksPanel(this);
        nodesPane = new NodesPanel(this);
        importPane = new ImportNetworkPanel(this);
        
        setLayout(new GridBagLayout());
        
        constrain(this, importPane, 0, 0, 1, 1);
        constrain(this, linksPane, 1, 0, 1, 1);
        constrain(this, nodesPane, 2, 0, 1, 1);
        
        setMinimumSize(getPreferredSize());
    }
    
    public void setProject(Project project)
    {
        linksPane.setProject(project);
        nodesPane.setProject(project);
        importPane.setProject(project);
    }
    
    public void reset()
    {
        nodesPane.reset();
        linksPane.reset();
    }
    
    public void setEnabled(boolean e)
    {
        nodesPane.setEnabled(e);
        linksPane.setEnabled(e);
        importPane.setEnabled(e);
        super.setEnabled(e);
    }

    
}
