/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.project;

import avdta.network.ReadNetwork;
import avdta.util.FileTransfer;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;

/**
 * This is a project that contains transit vehicles. It adds a transit subfolder with transit data files.
 * @author Michael
 */
public abstract class TransitProject extends Project
{
    /**
     * Constructs an empty {@link TransitProject}
     */
    public TransitProject(){}
    
    /**
     * Constructs a {@link TransitProject} at the given directory
     * @param directory the project directory
     * @throws IOException if a file is not found
     */
    public TransitProject(File directory) throws IOException
    {
        super(directory);
    }
    
    /**
     * Writes empty data files for transit
     * @throws IOException if a file is not found
     */
    public void writeEmptyFiles() throws IOException
    {
        super.writeEmptyFiles();
        
        PrintStream fileout = new PrintStream(new FileOutputStream(getBusFile()), true);
        fileout.println(ReadNetwork.getBusFileHeader());
        fileout.close();
        
        fileout = new PrintStream(new FileOutputStream(getBusRouteLinkFile()), true);
        fileout.println(ReadNetwork.getBusRouteLinkFileHeader());
        fileout.close();
        
        fileout = new PrintStream(new FileOutputStream(getBusFrequencyFile()), true);
        fileout.println(ReadNetwork.getBusFrequencyFileHeader());
        fileout.close();
        
        fileout = new PrintStream(new FileOutputStream(getBusPeriodFile()), true);
        fileout.println(ReadNetwork.getBusPeriodFileHeader());
        fileout.close();
    }
    
    /**
     * Clones files from the specified project
     * @param rhs the project to be cloned
     * @throws IOException if a file is not found
     */
    public void cloneFromProject(Project rhs) throws IOException
    {
        super.cloneFromProject(rhs);
        
        if(rhs instanceof TransitProject)
        {
            importTransitFromProject((TransitProject)rhs);
        }
    }
    
    /**
     * Creates the project folders. 
     * This method creates the folder {@link Project#getProjectDirectory()}/transit/.
     * This method also calls {@link Project#createProjectFolders(java.io.File)} 
     * @param dir the root directory
     * @throws IOException if a file is not found
     */
    public void createProjectFolders(File dir) throws IOException
    {
        super.createProjectFolders(dir);
        
        String dirStr = dir.getCanonicalPath();
        
        File file = new File(dirStr+"/transit");
        file.mkdirs();
    }
    
    /**
     * Returns the bus file
     * @return {@link Project#getProjectDirectory()}/transit/bus.txt
     */
    public File getBusFile()
    {
        return new File(getProjectDirectory()+"/transit/bus.txt");
    }
    
    /**
     * Returns the bus frequency file
     * @return {@link Project#getProjectDirectory()}/transit/bus_frequency.txt
     */
    public File getBusFrequencyFile()
    {
        return new File(getProjectDirectory()+"/transit/bus_frequency.txt");
    }
    
    /**
     * Returns the bus route link file
     * @return {@link Project#getProjectDirectory()}/transit/bus_route_link.txt
     */
    public File getBusRouteLinkFile()
    {
        return new File(getProjectDirectory()+"/transit/bus_route_link.txt");
    }
    
    /**
     * Returns the bus period file
     * @return {@link Project#getProjectDirectory()}/transit/bus_period.txt
     */
    public File getBusPeriodFile()
    {
        return new File(getProjectDirectory()+"/transit/bus_period.txt");
    }
    
    
    /**
     * Exports transit data to the SQL database
     * @throws SQLException if the database cannot be accessed
     * @throws IOException if a file is not found
     * @see SQLLogin
     */
    public void exportTransitToSQL() throws SQLException, IOException
    {
        SQLLogin login = new SQLLogin(getDatabaseName());
        Connection connect = DriverManager.getConnection(login.toString());
        login = null;
        
        // nodes, links, options, phases
        String dir = getProjectDirectory()+"/temp";
        File folder = new File(dir);
        folder.mkdirs();
        
        createTempFile(getBusFile(), new File(dir+"/bus.txt"));
        createTempFile(getBusPeriodFile(), new File(dir+"/bus_period.txt"));
        createTempFile(getBusFrequencyFile(), new File(dir+"/bus_frequency.txt"));
        createTempFile(getBusRouteLinkFile(), new File(dir+"/bus_route_link.txt"));
        
        Statement st = connect.createStatement();
        st.executeQuery("create table if not exists bus (id int, type int, route int, dtime int);");
        st.executeQuery("create table if not exists bus_route_link (route int, sequence int, link int, stop int, dwelltime int);");
        st.executeQuery("create table if not exists bus_frequency (route int, period int, frequency int, offset int);");
        st.executeQuery("create table if not exists bus_period (id int, starttime int, endtime int);");
        
        st.executeQuery("\\copy bus from temp/bus.txt");
        st.executeQuery("\\copy bus_route_link from temp/bus_route_link.txt");
        st.executeQuery("\\copy bus_frequency from temp/bus_frequency.txt");
        st.executeQuery("\\copy bus_period from temp/bus_period.txt");
        
        
        for(File file : folder.listFiles())
        {
            file.delete();
        }
        folder.delete();
        
    }
    
    /**
     * Imports transit data from the SQL database
     * @throws SQLException if the database cannot be accessed
     * @throws IOException if a file is not found
     * @see SQLLogin
     */
    public void importTransitFromSQL() throws SQLException, IOException
    {
        SQLLogin login = new SQLLogin(getDatabaseName());
        Connection connect = DriverManager.getConnection(login.toString());
        login = null;
        
        String dir = getProjectDirectory()+"/temp";
        File folder = new File(dir);
        folder.mkdirs();
        
        Statement st = connect.createStatement();
        st.executeQuery("\\copy bus to temp/bus.txt");
        st.executeQuery("\\copy bus_frequency to temp/bus_frequency.txt");
        st.executeQuery("\\copy bus_period to temp/bus_period.txt");
        st.executeQuery("\\copy bus_route_link to temp/bus_route_link.txt");
        
        createRealFile(new File(dir+"/bus.txt"), ReadNetwork.getBusFileHeader(), getBusFile());
        createRealFile(new File(dir+"/bus_frequency.txt"), ReadNetwork.getBusFrequencyFileHeader(), getBusFrequencyFile());
        createRealFile(new File(dir+"/bus_route_link.txt"), ReadNetwork.getBusRouteLinkFileHeader(), getBusRouteLinkFile());
        createRealFile(new File(dir+"/bus_period.txt"), ReadNetwork.getBusPeriodFileHeader(), getBusPeriodFile());
        
        for(File file : folder.listFiles())
        {
            file.delete();
        }
        folder.delete();
    }
    
    /**
     * Copies transit data from the given project
     * @param rhs the project to be copied from
     * @throws IOException if a file is not found
     */
    public void importTransitFromProject(TransitProject rhs) throws IOException
    {
        FileTransfer.copy(rhs.getBusFile(), getBusFile());
        FileTransfer.copy(rhs.getBusFrequencyFile(), getBusFrequencyFile());
        FileTransfer.copy(rhs.getBusRouteLinkFile(), getBusRouteLinkFile());
        FileTransfer.copy(rhs.getBusPeriodFile(), getBusPeriodFile());
    }
    
}