package instances.networks.reductions;

import instances.networks.Network;
import instances.networks.edges.E;
import instances.networks.edges.E.SwitchType;
import instances.networks.vertices.V;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.uci.ics.jung.graph.util.EdgeType;

public class Reductions {
	
	private Network net;

	public Reductions(Network net) {

		this.net = net;
		
	}
	
	public void renewG() {
		
		Iterator<V> iterNodes = this.net.getG().getVertices().iterator();
		int countNodes = 1;
		
		while (iterNodes.hasNext()) {
			
			V node = iterNodes.next();
			if (node.id > 0)
				node.id = countNodes++;
			else
				this.net.setRoot(node);
			
		}

		Iterator<E> iterEdges = this.net.getG().getEdges().iterator();
		int countEdges = 1;
		Map<Integer,E> newMap = new HashMap<Integer,E>();
		int numNoProt = 1;
		int numProt = 0;
		
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			if (edge.id > 0) {
				newMap.put(countEdges,edge);
				edge.id = countEdges++;
				//if (edge.prot) {
				if (edge.status == SwitchType.PROT) {
					edge.idProt = numProt++;
				} else {
					edge.idNoProt = numNoProt++;
				}
			} else {
				newMap.put(0,edge);
			}
			
		}	
		
		this.net.setMapEdgeIndex(newMap);
    	this.net.setNumNoProt(numNoProt);
    	this.net.setNumProt(numProt);
	    
	}	
	
	public void reduceG1() {

		ArrayList<E> newEdges = new ArrayList<E>();
		Iterator<E> iterEdges;
		Iterator<V> iterNodes;
		V suc;
		E sucEdge;
		boolean repeat = true;

		while (repeat) {

			repeat = false;

			iterNodes = this.net.getG().getVertices().iterator();			

			while (iterNodes.hasNext()) {

				V node = iterNodes.next();

				if ((node.clients == 0) && (this.net.getG().getPredecessorCount(node)>0)) {

					if (this.net.getG().getSuccessorCount(node) == 1) {

						iterEdges = this.net.getG().getOutEdges(node).iterator();
						sucEdge = iterEdges.next();

						if (sucEdge.status != SwitchType.PROT) {							
							
							suc = sucEdge.node2;

							repeat = true;

							node.clients += suc.clients;
							node.demand += suc.demand;
							node.thetaL += suc.thetaL;
							node.thetaR += suc.thetaR;

							iterEdges = this.net.getG().getOutEdges(suc).iterator();

							while (iterEdges.hasNext()) {
								sucEdge = iterEdges.next();

								E newEdge = new E(sucEdge.id,node,sucEdge.node2,sucEdge.status,sucEdge.idProt,sucEdge.idNoProt);

								//oldEdges.add(sucEdge);
								newEdges.add(newEdge);

							}

							iterEdges = newEdges.iterator();
							while (iterEdges.hasNext()) {
								E newEdge = iterEdges.next();
								this.net.getG().addEdge(newEdge, newEdge.node1, newEdge.node2,EdgeType.DIRECTED);
							}
							newEdges.clear();

							this.net.getG().removeVertex(suc);
							
							break;

						} //if (!sucEdge.prot)
						
					}//if (g.getSuccessorCount(node) == 1)
					
				}//if (node.clientes == 0)
				
			}//while (iterNodes.hasNext())
			
		}//while (repeat)

		renewG();

	}
	
	public void reduceG2() {

		ArrayList<E> newEdges = new ArrayList<E>();
		Iterator<E> iterEdges;
		Iterator<V> iterNodes;
		V suc = null;
		E sucEdge;
		boolean repeat = true;

		while (repeat) {

			repeat = false;

			iterNodes = this.net.getG().getVertices().iterator();			

			while (iterNodes.hasNext()) {

				V node = iterNodes.next();

				int contaNosComClientes = 0;

				if ((node.clients == 0) && (this.net.getG().getPredecessorCount(node)>0) && (this.net.getG().getSuccessorCount(node)>1)) {

					iterEdges = this.net.getG().getOutEdges(node).iterator();

					while (iterEdges.hasNext()) {
						sucEdge = iterEdges.next();						

						if (sucEdge.node2.clientsSum > 0) {
						
							contaNosComClientes++;

							//if ((contaNosComClientes > 2)||(sucEdge.prot)) {
							if ((contaNosComClientes > 2)||(sucEdge.status == SwitchType.PROT)) {
								contaNosComClientes = -1;
								break;	// while (iterEdges.hasNext())
							}
							
						}
					}	// while (iterEdges.hasNext())

					if (contaNosComClientes == 1) {
						
						repeat = true;

						iterEdges = this.net.getG().getOutEdges(node).iterator();

						while (iterEdges.hasNext()) {

							sucEdge = iterEdges.next();

							suc = sucEdge.node2;

							//if ((!sucEdge.prot)&&(suc.clientesAcum > 0))
							if ((sucEdge.status != SwitchType.PROT)&&(suc.clientsSum > 0))
								break;

						}

						node.clientsSum = node.clients + suc.clientsSum;
						node.clients += suc.clients;
						node.demand += suc.demand;
						node.thetaL += suc.thetaL;
						node.thetaR += suc.thetaR;

						iterEdges = this.net.getG().getOutEdges(suc).iterator();

						while (iterEdges.hasNext()) {
							sucEdge = iterEdges.next();

							E newEdge = new E(sucEdge.id,node,sucEdge.node2,sucEdge.status,sucEdge.idProt,sucEdge.idNoProt);

							//oldEdges.add(sucEdge);
							newEdges.add(newEdge);

						}

						iterEdges = newEdges.iterator();
						while (iterEdges.hasNext()) {
							E newEdge = iterEdges.next();
							this.net.getG().addEdge(newEdge, newEdge.node1, newEdge.node2,EdgeType.DIRECTED);
						}
						newEdges.clear();

						this.net.getG().removeVertex(suc);

						break;	

					} //if (contaNosComClientes == 1)						

				} //if (node.clientes == 0)

			}//while (iterNodes.hasNext())

		}//while (repeat)

		renewG();

	}
	
	public void reduceG4() {

		ArrayList<E> newEdges = new ArrayList<E>();
		Iterator<E> iterEdges, iterEdgesSuc;
		Iterator<V> iterNodes;//, sucessors;
		V suc = null;
		E sucEdge, sucSucEdge;
		boolean repeat = true;

		while (repeat) {

			repeat = false;

			iterNodes = this.net.getG().getVertices().iterator();			

			while (iterNodes.hasNext()&(!repeat)) {

				V node = iterNodes.next();

				int contaNosComClientes = 0, contaNosSemClientes = 0;

				if ((node.clients == 0) && (this.net.getG().getPredecessorCount(node)>0) && (this.net.getG().getSuccessorCount(node)>0)) {

					iterEdges = this.net.getG().getOutEdges(node).iterator();

					while (iterEdges.hasNext()) {
						sucEdge = iterEdges.next();						

						if (sucEdge.node2.clientsSum > 0) {
						
							contaNosComClientes++;
							break;
							
						} else {
							
							contaNosSemClientes++;
							
						}
					}	// while (iterEdges.hasNext())

					if ((contaNosSemClientes > 0)&&(contaNosComClientes == 0)) {
						
						iterEdges = this.net.getG().getOutEdges(node).iterator();

						while (iterEdges.hasNext()&&(!repeat)) {

							sucEdge = iterEdges.next();

							suc = sucEdge.node2;
						
							iterEdgesSuc = this.net.getG().getOutEdges(suc).iterator();
							while (iterEdgesSuc.hasNext()) {
								sucSucEdge = iterEdgesSuc.next();
								E newEdge = new E(sucSucEdge.id,node,sucSucEdge.node2,(sucEdge.status == SwitchType.PROT)||(sucSucEdge.status == SwitchType.PROT),sucSucEdge.idProt,sucSucEdge.idNoProt);
								newEdges.add(newEdge); 
							}

							iterEdgesSuc = newEdges.iterator();
							while (iterEdgesSuc.hasNext()) {
								E newEdge = iterEdgesSuc.next();
								this.net.getG().addEdge(newEdge, newEdge.node1, newEdge.node2,EdgeType.DIRECTED);
							}
							newEdges.clear();
							
							//if (!sucEdge.prot) {
							if (sucEdge.status != SwitchType.PROT) {
							
								node.demand += suc.demand;
								node.thetaL += suc.thetaL;
								node.thetaR += suc.thetaR;
							
							}

							this.net.getG().removeVertex(suc);
							
							repeat = true;
								
						} // while (iterEdges.hasNext()&&(!repeat))

					} //if ((contaNosSemClientes > 0)&&(contaNosComClientes == 0))						

				} //if ((node.clientes == 0) && (g.getPredecessorCount(node)>0) && (g.getSuccessorCount(node)>1))

			} //while (iterNodes.hasNext()&(!repeat))

		} //while (repeat)

		renewG();

	}			

}
