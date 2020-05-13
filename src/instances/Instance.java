package instances;

import java.awt.Container;
import java.awt.Dimension;

import javax.swing.JFrame;
import javax.xml.transform.Transformer;

import edu.uci.ics.jung.algorithms.cluster.VoltageClusterer;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.algorithms.layout.FRLayout;
import edu.uci.ics.jung.algorithms.layout.Layout;
import edu.uci.ics.jung.algorithms.layout.TreeLayout;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.SparseGraph;
import edu.uci.ics.jung.samples.SimpleGraphDraw;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.VisualizationViewer;
import edu.uci.ics.jung.visualization.renderers.Renderer;
import reliability.Reliability;
import instances.networks.Network;
import instances.networks.edges.E;
import instances.networks.reductions.Reductions;
import instances.networks.vertices.V;

public class Instance {
	
	public Network net;
	public InstanceParameters parameters;
	public Reliability reliability;
	public Reductions reduce;
	public boolean setReductions = false;
	
	public Instance() {
		
		//setting instance parameters
		parameters = new InstanceParameters();
		
		//setting instance network
		net = new Network("instancias/"+parameters.getInstanceName());
		
		reliability = new Reliability(this);
		
	
//      JFrame frame = new JFrame();
//      Container content = frame.getContentPane();
//      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//      content.add(new MyTreeCollapseDemo(this));
//      frame.pack();
//      frame.setVisible(true);
		
//		Layout<V, E> l = new FRLayout<V, E>( net.getG() );
//		VisualizationViewer<V, E> vv = new VisualizationViewer<V, E>( l );
//		JFrame jf = new JFrame();
//		jf.getContentPane().add ( vv );
//		jf.setVisible(true);
		
//	    // The Layout<V, E> is parameterized by the vertex and edge types
//	    Layout<V, E> layout = new TreeLayout<V, E>(net.getG());
//	    layout.setSize(new Dimension(900,900)); // sets the initial size of the space
//	     // The BasicVisualizationServer<V,E> is parameterized by the edge types
//	     BasicVisualizationServer<V,E> vv = 
//	              new BasicVisualizationServer<V,E>(layout);
//	     vv.setPreferredSize(new Dimension(350,350)); //Sets the viewing area size
//	     JFrame frame = new JFrame("Simple Graph View");
//	     frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//	     frame.getContentPane().add(vv); 
//	     frame.pack();
//	     frame.setVisible(true);     
			
		if (setReductions) {
			reduce = new Reductions(net);
			System.out.println("|E| = "+net.getG().getEdgeCount());
			reduce.reduceG1();

//		      JFrame frame = new JFrame();
//		      Container content = frame.getContentPane();
//		      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		      content.add(new MyTreeCollapseDemo(this));
//		      frame.pack();
//		      frame.setVisible(true);
			
			System.out.println("|E| = "+net.getG().getEdgeCount());
			reduce.reduceG2();
			System.out.println("|E| = "+net.getG().getEdgeCount());
			reduce.reduceG4();
			System.out.println("|E| = "+net.getG().getEdgeCount());
		}
		//System.out.println("teste");
		
//	      JFrame frame = new JFrame();
//	      Container content = frame.getContentPane();
//	      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//	      content.add(new MyTreeCollapseDemo(this));
//	      frame.pack();
//	      frame.setVisible(true);
	      
	}

	public InstanceParameters getParameters() {
		return parameters;
	}
	

}
