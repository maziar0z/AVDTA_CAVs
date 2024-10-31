/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package cdta.cell;

/**
 *
 * @author micha
 */
public class OrdinaryConnector extends SameCellConnector
{
    private int y;
    
    public OrdinaryConnector(Cell i, Cell j)
    {
        super(i, j);
    }
    
    public boolean validate()
    {
        return true;
    }
    
    public int sumY()
    {
        return y;
    }
    
    public void initConnectivity()
    {
        super.initConnectivity();
        setCongestionConnectivity(getI().getCapacity() > 0 && getJ().getCapacity() > 0);
    }
    
    public int sumYIn(Cell i)
    {
        return y;
    }
    
    public int sumYOut(Cell j)
    {
        return y;
    }
    
    public int getY(Cell i, Cell j)
    {
        return y;
    }
    
    public void addY(Cell i, Cell j)
    {
        y ++;
    }
}