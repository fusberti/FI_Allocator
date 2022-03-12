package reliability;

import java.util.Iterator;

import instances.Instance;
import instances.InstanceParameters;
import instances.networks.Network;
import instances.networks.edges.E;
import instances.networks.edges.E.SwitchType;
import instances.networks.vertices.V;

import edu.uci.ics.jung.graph.Graph;

public class Reliability {
	
	private InstanceParameters parameters;
	private Network net;
	private Graph<V, E> g;
	private V root;
	private double sumPFl, sumNFl, Ptheta, Ntheta, thetaTil;

	public Reliability(Instance instance) {
		parameters = instance.parameters;
		net = instance.net;
		g = net.getG();
		root = net.getRoot();
	
    	//calculando fl para todos os arcos
    	this.calcFl(root);
    	   	
    	//calculando tr para todos os nos
    	this.calcBigM(root);
    	
    	sumPFl = this.calcSumPFl();
    	sumNFl = this.calcSumNFl();
    	Ptheta = this.calcPtheta();
    	Ntheta = this.calcNtheta();
    	thetaTil = this.calcThetaTil(root);
    	
	}
	

	
	public double calcThetaTil(V node) {

		Iterator<E> iterEdges;

		node.thetaTil = node.thetaR;		
		
		iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E outEdge = iterEdges.next();
			node.thetaTil += calcThetaTil(outEdge.node2);
		
		}		
				
		//root node does not have an incoming edges
		return node.thetaTil;
	    
	}	
	
	
	private double calcFl(V node) {

		E inEdge;
		Iterator<E> iterEdges;
		double sumFl = 0.0f;

		sumFl = node.thetaL;		
		
		iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E outEdge = iterEdges.next();
			sumFl += calcFl(outEdge.node2);
		
		}		
		
		iterEdges = g.getInEdges(node).iterator();
		if (iterEdges.hasNext()) {
			
			inEdge = iterEdges.next();
			if (inEdge.status != SwitchType.PROT)
				return inEdge.fl = sumFl;
			else
				return inEdge.fl = parameters.getProtFailRate()*sumFl;				

		}
		
		//root node does not have an incoming edges
		return 0.0f;
	    
	}	
	
	private double calcBigM(V node) {
		
		node.M = node.thetaR;
		
		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			double subTr = calcBigM(edge.node2);			
			if (edge.status != SwitchType.PROT)
				node.M += subTr;
		
		}
				
		return node.M;
	    
	}	
	
	public double calcPtheta() {
		
		double Ptheta = 0;
		
		Iterator<V> iterNodes = g.getVertices().iterator();
		while (iterNodes.hasNext()) {
			
			V node = iterNodes.next();
			Ptheta += node.demandSum*(node.thetaL+node.thetaR);
				
		}		
				
		return Ptheta;
	    
	}	
	
	public double calcNtheta() {
		
		double Ntheta = 0;
		
		Iterator<V> iterNodes = g.getVertices().iterator();
		while (iterNodes.hasNext()) {
			
			V node = iterNodes.next();
			Ntheta += node.clientsSum*(node.thetaL+node.thetaR);
				
		}		
				
		return Ntheta;
	    
	}	
	
	public double calcSumPFl() {
		
		double sumPFl = 0;
		
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			sumPFl += (edge.node1.demandSum-edge.node2.demandSum)*edge.fl;
			
		}		
						
		return sumPFl;
	    
	}
	
	public double calcSumNFl() {
		
		double sumNFl = 0;
		
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			sumNFl += (edge.node1.clientsSum-edge.node2.clientsSum)*edge.fl;
			
		}		
						
		return sumNFl;
	    
	}
	
	public double getSumPFl() {
		return sumPFl;
	}

	public double getSumNFl() {
		return sumNFl;
	}

	public double getPtheta() {
		return Ptheta;
	}

	public double getNtheta() {
		return Ntheta;
	}
	
//	public double calcTl(V node) {
//		
//		node.tl = node.thetaL;
//		
//		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
//		while (iterEdges.hasNext()) {
//			
//			E edge = iterEdges.next();
//			double subTl = calcTl(edge.node2);
//
//			if (edge.status != SwitchType.PROT)
//				node.tl += subTl;
//		
//		}
//		
//		return node.tl;
//	    
//	}	
//	
//	public void calcTMax() {
//		
//		Iterator<V> iterNodes = g.getVertices().iterator();
//		while (iterNodes.hasNext()) {
//			
//			V node = iterNodes.next();		
//			net.maxT += node.thetaL+node.thetaR;	
//
//		}
//			    
//	}		
//	
//	public double calcTr(V node) {
//		
//		node.tr = node.thetaR;
//		
//		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
//		while (iterEdges.hasNext()) {
//			
//			E edge = iterEdges.next();
//			double subTr = calcTr(edge.node2);			
//			if (edge.status == SwitchType.NONE)
//				node.tr += subTr;
//			
//		
//		}
//				
//		return node.tr;
//	    
//	}		
	
//	public double calcT(V node) {
//		
//		//valido somente para o no raiz
//		node.t = node.tl+node.tr;
//		
//		Iterator<E> iterInEdges = g.getInEdges(node).iterator();
//		while (iterInEdges.hasNext()) {
//			
//			E inEdge = iterInEdges.next();
//			node.t = inEdge.node1.t;
//					
//			if (inEdge.status == SwitchType.SWITCH)
//				node.t += node.tr;
//			else if (inEdge.status == SwitchType.PROT)
//				node.t += (node.tl+node.tr);
//		
//		}
//		
//		Iterator<E> iterOutEdges = g.getOutEdges(node).iterator();
//		while (iterOutEdges.hasNext()) {
//			
//			E outEdge = iterOutEdges.next();
//			calcT(outEdge.node2);			
//		
//		}		
//				
//		return node.t;
//	    
//	}		
	
//	public double calcDEC(V node) {
//		
//		if (node.clientsSum > 0)
//			node.dec = node.t*node.clients/node.clientsSum;
//		else
//			node.dec = 0.0f;
//		
//		Iterator<E> iterOutEdges = g.getOutEdges(node).iterator();
//		while (iterOutEdges.hasNext()) {
//			
//			E outEdge = iterOutEdges.next();
//			if (node.clientsSum > 0)
//				node.dec += calcDEC(outEdge.node2)*outEdge.node2.clientsSum/node.clientsSum;
//		
//		}		
//				
//		return node.dec;
//	    
//	}	
	
//	public double decMin(V node) {
//		
//		Iterator<E> iterEdges = g.getEdges().iterator();
//		while (iterEdges.hasNext()) {
//			
//			E edge = iterEdges.next();
//			if (edge.status != SwitchType.PROT)
//				edge.status = SwitchType.SWITCH;
//			
//		}
//		
//		calcTr(node);
//		calcT(node);
//		return calcDEC(node);
//	    
//	}
//	
//	public double decLB() {
//		
//		net.decMin = 0.0f;
//		Iterator<V> iterNodes = g.getVertices().iterator();
//		while (iterNodes.hasNext()) {
//			
//			V node = iterNodes.next();
//			net.decMin += node.clientsSum*(node.thetaL+node.thetaR);
//			
//		}
//		
//		return net.decMin /= net.root.clientsSum;
//	    
//	}		
//	
//	public double decMax(V node) {
//		
//		Iterator<E> iterEdges = g.getEdges().iterator();
//		while (iterEdges.hasNext()) {
//			
//			E edge = iterEdges.next();
//			if (edge.status != SwitchType.PROT)
//				edge.status = SwitchType.NONE;
//			
//		}
//		
//		calcTr(node);
//		calcT(node);
//		return calcDEC(node);
//	    
//	}	
//	
//	public double decUB() {
//		
//		net.decMax = 0.0f;
//		Iterator<V> iterNodes = g.getVertices().iterator();
//		while (iterNodes.hasNext()) {
//			
//			V node = iterNodes.next();
//			net.decMax += (node.thetaL+node.thetaR);
//			
//		}
//		
//		return net.decMax;
//	    
//	}	

//	public double putSwitches(ArrayList<E> edges) {
//		
//		double dec = 0.0f;
//		Iterator<E> iterEdges = edges.iterator();
//		
//		while (iterEdges.hasNext())
//			dec = putSwitch(iterEdges.next());
//		
//		return dec;
//		
//	}	
//	
//	public double putSwitch(E edge) {
//		
//		//double dec = this.raiz.dec;
//		
//		if (edge.status == SwitchType.NONE) {
//			
//			edge.status = SwitchType.SWITCH;
//		
//			V root = predSectorRoot(edge.node1);
//			
//			//atualiza dec
//			if (net.raiz.clientsSum > 0)
//				net.raiz.dec -= edge.node2.tr*(root.clientsSum-edge.node2.clientsSum)/net.raiz.clientsSum;
//			
//			//atualiza o tr dos nos entre o arco modificado e a raiz da secao predecessora do arco modificado
//			V node = edge.node2;
//			do {
//				Iterator<E> iterInEdges = g.getInEdges(node).iterator();
//				if (iterInEdges.hasNext()) {
//					E inEdge = iterInEdges.next();
//					node = inEdge.node1;
//				}
//				
//				node.tr -= edge.node2.tr;
//			} while (node != root);
//					
//		}
//	    
//		return net.raiz.dec;
//		
//	}
//	
//	public void putSwitchesDeltaDEC(ArrayList<E> edges) {
//		
//		Iterator<E> iterEdges = edges.iterator();
//		
//		while (iterEdges.hasNext()) {
//			E edge = iterEdges.next();
//			edge.deltaDEC = putSwitchDeltaDEC(edge);
//		}
//		
//	}	
//	
//	public double putSwitchDeltaDEC(E edge) {
//		
//		double deltaDec = 0.0f;
//		
//		if (edge.status == SwitchType.NONE) {
//				
//			V root = predSectorRoot(edge.node1);
//			
//			//atualiza dec
//			if (net.raiz.clientsSum > 0)
//				deltaDec -= edge.node2.tr*(root.clientsSum-edge.node2.clientsSum)/net.raiz.clientsSum;
//							
//		}
//	    
//		return deltaDec;
//		
//	}	
//	
//	public double removeSwitches(ArrayList<E> edges) {
//		
//		double dec = 0.0f;
//		Iterator<E> iterEdges = edges.iterator();
//		
//		while (iterEdges.hasNext())
//			dec = removeSwitch(iterEdges.next());
//		
//		return dec;
//		
//	}
//	
//	public double removeSwitch(E edge) {
//		
//		//double dec = this.raiz.dec;
//		
//		if (edge.status == SwitchType.SWITCH) {
//			
//			edge.status = SwitchType.NONE;
//		
//			V root = predSectorRoot(edge.node1);
//			
//			//atualiza dec
//			if (net.raiz.clientsSum > 0)
//				net.raiz.dec += edge.node2.tr*(root.clientsSum-edge.node2.clientsSum)/net.raiz.clientsSum;
//			
//			//atualiza o tr dos nos entre o arco modificado e a raiz da secao predecessora do arco modificado
//			V node = edge.node2;
//			do {
//				Iterator<E> iterInEdges = g.getInEdges(node).iterator();
//				if (iterInEdges.hasNext()) {
//					E inEdge = iterInEdges.next();
//					node = inEdge.node1;
//				}
//				node.tr += edge.node2.tr;
//			} while (node != root);
//					
//		}
//	    
//		return net.raiz.dec;
//		
//	}	
//	
//	public double removeSwitchDeltaDEC(E edge) {
//		
//		double deltaDec = 0.0f;
//		
//		if (edge.status == SwitchType.SWITCH) {
//				
//			V root = predSectorRoot(edge.node1);
//			
//			//atualiza dec
//			if (net.raiz.clientsSum > 0)
//				deltaDec += edge.node2.tr*(root.clientsSum-edge.node2.clientsSum)/net.raiz.clientsSum;
//							
//		}
//	    
//		return deltaDec;
//		
//	}		
//	
//	public V predSectorRoot(V node) {
//		
//		Iterator<E> iterEdges = g.getInEdges(node).iterator();
//		
//		while (iterEdges.hasNext()) {
//			
//			E edge = iterEdges.next();
//						
//			if (edge.status == SwitchType.NONE)
//				node = predSectorRoot(edge.node1);
//		
//		}
//	    
//		return node;
//		
//	}	
	
}
