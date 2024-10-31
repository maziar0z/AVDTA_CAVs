/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package avdta.network;

import avdta.dta.Assignment;
import avdta.network.link.Link;
import avdta.network.node.Node;
import avdta.project.Project;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

/**
 * This class contains a list of unique {@link Path} objects. 
 * It is intended to avoid creating duplicate {@link Path}s, which reduces memory as well as storage when saving assignments.
 * The {@link PathList} contains methods to read and write from file.
 * 
 * {@link Path}s are hashed according to origin, destination, and size to reduce the number of {@link Path} to {@link Path} comparisons. 
 * @author Michael
 */
public class PathList implements Iterable<Path>
{
    private Map<Node, Map<Node, Map<Integer, List<Path>>>> paths;
    
    /**
     * Constructs a new empty {@link PathList}
     */
    public PathList()
    {
        paths = new HashMap<Node, Map<Node, Map<Integer, List<Path>>>>();
    }
    
    /**
     * Constructs a {@link PathList} for the specified {@link Network} from the specified {@link File}.
     * The {@link Network} is needed because {@link PathList} saves {@link Link} ids, not {@link Link} objects.
     * @param network the {@link Network}
     * @param file the {@link File} to read from
     */
    public PathList(Network network, File file)
    {
        this();
        
        try
        {
            readFromFile(network, file);
        }
        catch(IOException ex)
        {
            
        }
    }
    
    public Iterator<Path> iterator()
    {
        return new PathListIterator();
    }
    
    public List<Path> getPaths(Node origin, Node dest)
    {
        List<Path> output = new ArrayList<Path>();
        
        if(!paths.containsKey(origin) || !paths.get(origin).containsKey(dest))
        {
            return output;
        }
        
        Map<Integer, List<Path>> temp = paths.get(origin).get(dest);
        
        for(int k : temp.keySet())
        {
            for(Path p : temp.get(k))
            {
                output.add(p);
            }
        }
        
        return output;
    }
    
    /**
     * This method attempts to add the {@link Path} to the list. 
     * If the {@link Path} already exists in this {@link PathList}, this method will return the existing {@link Path}.
     * Otherwise, the new {@link Path} will be added.
     * Calls to this method should use the return value as the assigned {@link Path}.
     * @param p the {@link Path} to be added
     * @return new path, or old path if new path is a duplicate
     */
    public Path addPath(Path p)
    {
        Map<Node, Map<Integer, List<Path>>> temp;
        
        Node origin = p.getOrigin();
        if(paths.containsKey(origin))
        {
            temp = paths.get(origin);
        }
        else
        {
            paths.put(origin, temp = new HashMap<Node, Map<Integer, List<Path>>>());
        }
        
        Map<Integer, List<Path>> temp2;
        
        Node dest = p.getDest();
        
        if(temp.containsKey(dest))
        {
            temp2 = temp.get(dest);
        }
        else
        {
            temp.put(dest, temp2 = new HashMap<Integer, List<Path>>());
        }
        
        List<Path> temp3;
        
        int hash = p.hashCode();
        
        if(temp2.containsKey(hash))
        {
            temp3 = temp2.get(hash);
            
            for(Path p2 : temp3)
            {
                if(p.equals(p2))
                {
                    return p2;
                }
            }
            
            temp3.add(p);
            p.setId();
            return p;
        }
        else
        {
            temp2.put(hash, temp3 = new ArrayList<Path>());
            temp3.add(p);
            p.setId();
            return p;
        }
    }
    
    /**
     * This method saves this {@link PathList} for the specified assignment (see {@link Assignment#getPathsFile()}).
     * This calls {@link PathList#writeToFile(java.io.File)}.
     * @param assign the {@link Assignment}
     * @throws IOException if the file cannot be written 
     */
    public void writeToFile(Assignment assign) throws IOException
    {
        writeToFile(assign.getPathsFile());
    }
    
    /**
     * This method writes this {@link PathList} to a file. 
     * All saved {@link Path}s are written out as a list of {@link Link}s. 
     * {@link Path} ids are saved, and {@link Path}s should be referenced by their id.
     * @param file the {@link File} to save to
     * @throws IOException if the file cannot be written 
     */
    public void writeToFile(File file) throws IOException
    {
        PrintStream fileout = new PrintStream(new FileOutputStream(file), true);
        
        for(Node o : paths.keySet())
        {
            Map<Node, Map<Integer, List<Path>>> temp = paths.get(o);
            
            for(Node d : temp.keySet())
            {
                Map<Integer, List<Path>> temp2 = temp.get(d);
                
                for(int h : temp2.keySet())
                {
                    for(Path p : temp2.get(h))
                    {
                        fileout.print(p.getId()+"\t"+p.size()+"\t"+p.proportion);
                        
                        for(Link l : p)
                        {
                            fileout.print("\t"+l.getId());
                        }
                        
                        fileout.println();
                    }
                }
            }
        }
        
        fileout.close();
    }
    

    
    public boolean contains(Path rhs)
    {
        Map<Integer, List<Path>> temp = paths.get(rhs.getOrigin()).get(rhs.getDest());
        
        for(int i : temp.keySet())
        {
            for(Path p : temp.get(i))
            {
                if(p == rhs)
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    public void clearProportions()
    {
        for(Path p : this)
        {
            p.flow = 0;
            p.proportion = 0;
        }
    }
    
    /**
     * This method constructs this {@link PathList} from the specified {@link File}. 
     * {@link Path}s in the file are converted to a list of {@link Link}s via the specified {@link Network}.
     * The file should not be modified outside of {@link PathList}. 
     * Loading a {@link PathList} from an externally modified file may result in corrupted data.
     * The {@link Network} is needed because {@link PathList} saves {@link Link} ids, not {@link Link} objects.
     * @param network the {@link Network} this {@link PathList} applies to.
     * @param file the {@link File} to read from
     * @throws IOException if the {@link File} cannot be read
     */
    public void readFromFile(Network network, File file) throws IOException
    {
        paths.clear();
        
        Scanner filein = new Scanner(file);
        
        Map<Integer, Link> links = network.createLinkIdsMap();
        
        while(filein.hasNextInt())
        {
            int id = filein.nextInt();
            
            Path p = new Path(id);
            int size = filein.nextInt();
            
            if(!filein.hasNextInt())
            {
                p.proportion = filein.nextDouble();
            }
            
            for(int i = 0; i < size; i++)
            {
                int link_id = filein.nextInt();
                
                p.add(links.get(link_id));
            }
            

            addUnrestricted(p);
        }
    }
    
    public Path randomPath(Node origin, Node dest)
    {
        Map<Integer, List<Path>> hashtable = paths.get(origin).get(dest);
        
        if(hashtable == null)
        {
            return null;
        }
        
        double rand = Simulator.active.getProject().getRandom().nextDouble();
        
        //for every size of path from origin to destination
        for(int i : hashtable.keySet())
        {
            //for every path of size i
            for(Path p : hashtable.get(i))
            {
                if(rand < p.proportion)
                {
                    return p;
                }
                else
                {
                    rand -= p.proportion;
                }
            }
        }
        
        return null;
        
    }
    
    /**
     * This method adds the {@link Path} without checking for duplicates.
     * It should not be used outside of the {@link PathList}, therefore it is marked private.
     * @param p the {@link Path} to be added
     */
    private void addUnrestricted(Path p)
    {
        Map<Node, Map<Integer, List<Path>>> temp;
        
        Node origin = p.getOrigin();
        if(paths.containsKey(origin))
        {
            temp = paths.get(origin);
        }
        else
        {
            paths.put(origin, temp = new HashMap<Node, Map<Integer, List<Path>>>());
        }
        
        Map<Integer, List<Path>> temp2;
        
        Node dest = p.getDest();
        
        if(temp.containsKey(dest))
        {
            temp2 = temp.get(dest);
        }
        else
        {
            temp.put(dest, temp2 = new HashMap<Integer, List<Path>>());
        }
        
        List<Path> temp3;
        
        int hash = p.hashCode();
        
        if(temp2.containsKey(hash))
        {
            temp3 = temp2.get(hash);
            temp3.add(p);
        }
        else
        {
            temp2.put(hash, temp3 = new ArrayList<Path>());
            temp3.add(p);
        }
    }
    
    /**
     * This creates a map of ids to {@link Path}s to help restore assignments saved via {@link Path} ids.
     * @return a mapping of ids to {@link Path}s
     */
    public Map<Integer, Path> createPathIdsMap()
    {
        Map<Integer, Path> output = new HashMap<Integer, Path>();
        
        for(Node o : paths.keySet())
        {
            Map<Node, Map<Integer, List<Path>>> temp = paths.get(o);
            
            for(Node d : temp.keySet())
            {
                Map<Integer, List<Path>> temp2 = temp.get(d);
                
                for(int h : temp2.keySet())
                {
                    for(Path p : temp2.get(h))
                    {
                        output.put(p.getId(), p);
                    }
                }
            }
        }
        
        return output;
    }
    
    public void calculateProportions()
    {

        
        for(Node o : paths.keySet())
        {
            Map<Node, Map<Integer, List<Path>>> temp = paths.get(o);
            
            for(Node d : temp.keySet())
            {
                double total = 0.0;
                
                Map<Integer, List<Path>> temp2 = temp.get(d);
                
                for(int hash : temp2.keySet())
                {
                    for(Path p : temp2.get(hash))
                    {
                        total += p.flow;
                    }
                }
                
                for(int hash : temp2.keySet())
                {
                    for(Path p : temp2.get(hash))
                    {
                        if(total > 0)
                        {
                            p.proportion = (double) p.flow / total;
                        }
                        else
                        {
                            p.proportion = 0;
                        }

                    }
                }
            }
        }
    }
    
    class PathListIterator implements Iterator<Path>
    {
        private Iterator<Node> origins, dests;
        private Iterator<Integer> hashcodes;
        private Iterator<Path> list;
        
        private Node o, d;
        private int h;
        
        public PathListIterator()
        {
            origins = paths.keySet().iterator();
        }
        
        public boolean hasNext()
        {
            if(list != null && list.hasNext())
            {
                return true;
            }
            else if(hashcodes != null && hashcodes.hasNext())
            {
                h = hashcodes.next();
                list = paths.get(o).get(d).get(h).iterator();
                return hasNext();
            }
            else if(dests != null && dests.hasNext())
            {
                d = dests.next();
                hashcodes = paths.get(o).get(d).keySet().iterator();
         
                
                return hasNext();
            }
            else if(origins != null && origins.hasNext())
            {
                o = origins.next();
                dests = paths.get(o).keySet().iterator();
                
                return hasNext();
            }
            return false;
        }
        
        public Path next()
        {
            if(list != null && list.hasNext())
            {
                return list.next();
            }
            else if(hashcodes != null && hashcodes.hasNext())
            {
                h = hashcodes.next();
                list = paths.get(o).get(d).get(h).iterator();
                return next();
            }
            else if(dests != null && dests.hasNext())
            {
                d = dests.next();
                list = paths.get(o).get(d).get(h).iterator();

                return next();
            }
            else if(origins != null && origins.hasNext())
            {
                o = origins.next();
                dests = paths.get(o).keySet().iterator();
      
                return next();
            }
            return null;
        }
    }
}