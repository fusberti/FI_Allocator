package instances.networks;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.Reader;
import java.io.StreamTokenizer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.apache.commons.collections15.Transformer;

import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.SparseMultigraph;
import edu.uci.ics.jung.graph.util.EdgeType;
import instances.Instance;
import instances.networks.edges.E;
import instances.networks.vertices.V;
import instances.networks.vertices.V.TYPES;


public class Network {

	private Graph<V, E> g;
	private Set<E> ties, transferSecs;
	private V root;
	private int numProt, numNoProt;
	private Map<Integer,E> mapEdgeIndex;
	private Map<Integer,V> mapNodeIndex;
	
	public Transformer<E, String> EdgeString = new Transformer<E,String>() {
		public String transform(E edge) {
			return "["+edge.node1+","+edge.node2+"]";
		}
	};	
	
public Network(String filename, Instance inst) {
		
		g = new SparseMultigraph<V, E>();
		//g = new Forest<V, E>();
		ties = new HashSet<E>();
		this.mapEdgeIndex = new HashMap<Integer,E>();
		this.mapNodeIndex = new HashMap<Integer,V>();
		numProt = 0;
		numNoProt = 0;
		int numV, numE, numT;		
		ArrayList<Integer> feeders = new ArrayList<>();
		double crewSpeed, failureRate;
		try {
				
			Reader r = new BufferedReader(new FileReader(filename));
	    	StreamTokenizer stok = new StreamTokenizer(r);
	    	stok.eolIsSignificant(true);
	    	stok.nextToken();	//segue coletando ate chegar na linha do problema
	    	String token = stok.sval;
	    	while (token.compareTo("p") != 0) {
	    		
		    	stok.nextToken();
		    	token = stok.sval;
		    	if (token == null) token = "";
	    		
	    	}
	    	
	    	stok.nextToken();	//coleta "fi"
	    	
	    	stok.nextToken();	//numero de nos
	    	numV = (int) stok.nval;
	    	
	    	stok.nextToken();	//numero de arestas
	    	numE = (int) stok.nval;
	    	
	    	stok.nextToken();	//failure rate per year per km
	    	failureRate = (double) stok.nval;
	    	inst.getParameters().setFailureRate(failureRate);
	    	
	    	stok.nextToken();	//crew velocity km/h
	    	crewSpeed = (double) stok.nval;
	    	inst.getParameters().setCrewVelocity(crewSpeed);
	    	
	    	stok.nextToken();	//coleta f 
	    	stok.nextToken();	
	    	
	    	while ( stok.nextToken() != StreamTokenizer.TT_EOL) {	    		
		    	int val = (int) stok.nval;
		    	//System.out.println(val);
		    	feeders.add(val);
	    	}
	    	stok.eolIsSignificant(false);
    		//No artificial -- raiz do alimentador
	    	root = new V(0,-1,0,0,0,0);
    		this.mapNodeIndex.put(-1,root);
    		g.addVertex(root);   	
	    	
	    	for (int i=0;i<numV;i++) {
	    		
	    		stok.nextToken();	//coleta "v"
	    		
	    		stok.nextToken();	//coleta indice no
	    		int label = (int) stok.nval;
	    		
	    		//System.out.println(label);
	    			    		
	    		stok.nextToken();	//coleta coordenada x
	    		double coordX = stok.nval;	
	    		
	    		stok.nextToken();	//coleta coordenada x
	    		double coordY = stok.nval;	

	    		V node = new V(g.getVertexCount(),label, coordX, coordY);

	    		this.mapNodeIndex.put(label,node);
	    		
//	    		System.out.println(node.id+" "+label+" "+thetaL+" "+thetaR+" "+demanda+" "+clientes);
	    		
	    		g.addVertex(node);	    		    		
	    	}
	    	    	
	    	
			// Aresta artificial -- da raiz do alimentador para todas as subestacoes
//			Iterator<Map.Entry<Integer, V>> iterNodes = this.mapNodeIndex.entrySet().iterator();
//			while (iterNodes.hasNext()) {
//				Map.Entry<Integer, V> entry = iterNodes.next();
//				V node = entry.getValue();
//				if (feeders.contains(node.label)) {
//					//System.out.println(node.label);
//					E edge = new E(g.getEdgeCount(), this.root, node, true, numProt++, -1, 0.0);
//					this.mapEdgeIndex.put(g.getEdgeCount(), edge);
//					g.addEdge(edge, this.root, node, EdgeType.DIRECTED);
//				}
//			}
			
			// Arestas artificiais -- da raiz para todos os alimentadores
			for(Integer label : feeders) {
				V node = this.mapNodeIndex.get(label);
				E edge = new E(g.getEdgeCount(), this.root, node, true, numProt++, -1, 0.0);
				this.mapEdgeIndex.put(g.getEdgeCount(), edge);
				g.addEdge(edge, this.root, node, EdgeType.DIRECTED);
			}
			
	    	for (int i=0;i<numE;i++) {
	    		
		    	stok.nextToken();	//coleta "e"
		    	
		    	stok.nextToken();	//no 1
		    	int nodeLabel1 = (int) stok.nval;
		    	
		    	stok.nextToken();	//no 2
		    	int nodeLabel2 = (int) stok.nval;
		    		    	
		    	   		    	
		    	stok.nextToken();
		    	double dist = stok.nval;
		    	
		    	//System.out.println(dist);
		    	
		    	V node1 = mapNodeIndex.get(nodeLabel1);
		    	V node2 = mapNodeIndex.get(nodeLabel2);
		    	
		    	if (feeders.contains(node2.label)) {
		    		V temp = node1;
		    		node1 = node2;
		    		node2 = temp;
		    	}
		    	
		    	E edge = new E(g.getEdgeCount(),node1,node2, false,numProt,-1,dist);
		    	
		    	this.mapEdgeIndex.put(g.getEdgeCount(),edge);
		    	
		    	//System.out.println(edge.id+" "+nodeLabel1+" "+mapNodeIndex.get(nodeLabel1)+" "+nodeLabel2+" "+mapNodeIndex.get(nodeLabel2));
		    	
		    	g.addEdge(edge, node1, node2,EdgeType.DIRECTED);

		    	//System.out.println("num edges = " + g.getEdgeCount());
		    		    		    	    		
	    	}	    	
	    	
//	    	ArrayList<E> listEdges = new ArrayList<>(g.getEdges());
//	    	
//	    	Collections.sort(listEdges, new EdgeComparator());
//	    	for (E edge : listEdges)
//				System.out.printf("Arco %d (%d,%d) length: %.1f\n", edge.id, edge.node1.id, edge.node2.id, edge.dist);
//	    	
	    	
	    	//calcula as distancias acumuladas por no
	    	this.calcSumDist(root);
	    	
	    	    	    	
	    } catch (Exception e) {
	    	System.err.println("Erro leitura da instancia...");
	    	e.printStackTrace();
	    	System.exit(1);
	    }	

    	
	}	

	
	
	public Network(String filename) {
		
		g = new SparseMultigraph<V, E>();
		//g = new Forest<V, E>();
		ties = new HashSet<E>();
		this.mapEdgeIndex = new HashMap<Integer,E>();
		this.mapNodeIndex = new HashMap<Integer,V>();
		numProt = 0;
		numNoProt = 0;
		int numV, numE, numT;		
		
		try {
				
			Reader r = new BufferedReader(new FileReader(filename));
	    	StreamTokenizer stok = new StreamTokenizer(r);
	    	
	    	stok.nextToken();	//segue coletando ate chegar na linha do problema
	    	String token = stok.sval;
	    	while (token.compareTo("p") != 0) {
	    		
		    	stok.nextToken();
		    	token = stok.sval;
		    	if (token == null) token = "";
	    		
	    	}
	    	
	    	stok.nextToken();	//coleta "chaves"
	    	
	    	stok.nextToken();	//numero de nos
	    	numV = (int) stok.nval;
	    	
	    	stok.nextToken();	//numero de arestas
	    	numE = (int) stok.nval;
	    	
	    	stok.nextToken();	//numero de manobras
	    	numT = (int) stok.nval;	    	
	    	
    		//No artificial -- raiz do alimentador
	    	root = new V(0,-1,0,0,0,0);
    		this.mapNodeIndex.put(0,root);
    		g.addVertex(root);   	
	    	
	    	for (int i=0;i<numV;i++) {
	    		
	    		stok.nextToken();	//coleta "v"
	    		
	    		stok.nextToken();	//coleta indice no
	    		int label = (int) stok.nval;
	    		
	    		stok.nextToken();	//coleta thetaL
	    		double thetaL = stok.nval;
	    		
	    		stok.nextToken();	//coleta thetaR
	    		double thetaR = stok.nval;
	    		
	    		stok.nextToken();	//coleta demanda
	    		double demanda = stok.nval;	
	    		
	    		stok.nextToken();	//coleta clientes
	    		int clientes = (int) stok.nval;	

	    		V node = new V(g.getVertexCount(),label,thetaL,thetaR,demanda,clientes);
//	    		vertices.add(node.id, node);
	    		this.mapNodeIndex.put(label,node);
	    		
//	    		System.out.println(node.id+" "+label+" "+thetaL+" "+thetaR+" "+demanda+" "+clientes);
	    		
	    		g.addVertex(node);
	    		    		
	    	}
	    	    	
    		//Aresta artificial -- da raiz do alimentador para todas as subestacoes    	
//	    	Iterator<V> iterNodes = g.getVertices().iterator();
//	    	while (iterNodes.hasNext()) {
//	    		
//	    		V node = iterNodes.next();
//	    		if (node.type == TYPES.SUBSTATION) {    		    	
//	    			E edge = new E(g.getEdgeCount(),this.root,node,true,numProt++,-1);
//			    	this.mapEdgeIndex.put(g.getEdgeCount(),edge);
//			    	g.addEdge(edge,this.root,node,EdgeType.DIRECTED);
//	    		}
//	    		
//	    	}
	    	
			// Aresta artificial -- da raiz do alimentador para todas as subestacoes
			Iterator<Map.Entry<Integer, V>> iterNodes = this.mapNodeIndex.entrySet().iterator();
			while (iterNodes.hasNext()) {
				Map.Entry<Integer, V> entry = iterNodes.next();
				V node = entry.getValue();
				if (node.type == TYPES.SUBSTATION) {
					E edge = new E(g.getEdgeCount(), this.root, node, true, numProt++, -1);
					this.mapEdgeIndex.put(g.getEdgeCount(), edge);
					g.addEdge(edge, this.root, node, EdgeType.DIRECTED);
				}
			}
	    	    	    	   	
	    	for (int i=0;i<numE;i++) {
	    		
		    	stok.nextToken();	//coleta "e"
		    	
		    	stok.nextToken();	//no 1
		    	int nodeLabel1 = (int) stok.nval;
		    	
		    	stok.nextToken();	//no 2
		    	int nodeLabel2 = (int) stok.nval;
		    		    	
		    	stok.nextToken();	//prot
		    	boolean prot = false;
		    	if (stok.nval > 0)
		    		prot = true;
   		    	
		    	stok.nextToken();
		    	int dist = (int) stok.nval;
		    	
		    	V node1 = mapNodeIndex.get(nodeLabel1);
		    	V node2 = mapNodeIndex.get(nodeLabel2);
		    	E edge;
		    	if (prot)
		    		//edge = new E(g.getEdgeCount(),node1,node2,prot,numProt++,-1);
		    		edge = new E(g.getEdgeCount(),node1,node2,prot,numProt++,-1,dist);
		    	else
		    		//edge = new E(g.getEdgeCount(),node1,node2,prot,-1,numNoProt++);
		    		edge = new E(g.getEdgeCount(),node1,node2,prot,numProt++,-1,dist);
		    		
		    	this.mapEdgeIndex.put(g.getEdgeCount(),edge);
		    	
//		    	System.out.println(edge.id+" "+nodeLabel1+" "+mapNodeIndex.get(nodeLabel1)+" "+nodeLabel2+" "+mapNodeIndex.get(nodeLabel2));
		    	
		    	g.addEdge(edge, mapNodeIndex.get(nodeLabel1), mapNodeIndex.get(nodeLabel2),EdgeType.DIRECTED);

		    	//System.out.println("num edges = " + g.getEdgeCount());
		    		    	    		
	    	}
	    	
	    	//CHAVES DE MANOBRA
	    	for (int i=0;i<numT;i++) {
	    		
		    	stok.nextToken();	//coleta "t"
		    	
		    	stok.nextToken();	//no 1
		    	int nodeLabel1 = (int) stok.nval;
		    	
		    	stok.nextToken();	//no 2
		    	int nodeLabel2 = (int) stok.nval;
		    		    	
		    	stok.nextToken();	//prot -- not used for tie-lines
   		    	   		   	
		    	V node1 = mapNodeIndex.get(nodeLabel1);
		    	V node2 = mapNodeIndex.get(nodeLabel2);
		    	E tie = new E(-1,node1,node2,false,-1,-1);
		    	this.ties.add(tie);
		    		    	    		
	    	}	    	
	    	    	    	
	    } catch (Exception e) {
	    	System.err.println("Erro leitura");
	    	e.printStackTrace();
	    }	

    	//calculando clientes acumulados para todos os nos
    	this.calcClientsSum(root);	
    	
    	//calculando demandas acumuladas para todos os nos
    	this.calcDemandSum(root);
    	
    	//determinando o no subestacao a montante para cada no da rede
    	this.findSubNode(this.root, this.root);
    	
    	//encontra a capacidade remanescente dos nos com uma linha aberta incidente
    	this.calcRemainingCapacity();

    	//encontra possiveis chaves seccionadoras que podem ser utilizadas em uma manobra
    	this.findGoodSecs(); 
    	
    	//encontra proximos ramos irmaos de cada no
    	this.findSuccessors(root);
    	
    	//calcula as distancias acumuladas por no
    	this.calcSumDist(root);
    	
	}	
	
	public double calcSumDist(V node) {

		Iterator<E> iterEdges;

		node.sumDist = 0;		
		
		iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E outEdge = iterEdges.next();
			node.sumDist += outEdge.dist;
			node.sumDist += calcSumDist(g.getDest(outEdge));
		
		}		
				
		//root node does not have an incoming edges
		return node.sumDist;
	    
	}	

	
	
//	public static void main(String[] args) throws IOException {
//
//		Solver newModel = null;
//
//			try {
//				Reader fileInst = new BufferedReader(new FileReader("instancias.txt"));
//				StreamTokenizer stok = new StreamTokenizer(fileInst);
//
//				stok.nextToken();
//				while (stok.sval != null) {
//
//					String instancia = stok.sval;
//
//					newModel = new Solver("instancias/" + instancia);		
//
//					stok.nextToken();
//
//				}
//				
//		    	System.out.println(newModel.inst.net.g);
//		    	
//		    	
//				Iterator<V> iterNodes;
//
//				iterNodes = newModel.inst.net.g.getVertices().iterator();			
//
//				while (iterNodes.hasNext()) {
//
//					V node = iterNodes.next();
//					System.out.println("subID["+node.label+"] = "+node.subNode.label);
//					System.out.println("trmax["+node.label+"] = "+node.subNode.M);
////					System.out.println("thetaL["+node+"] = "+node.thetaL+"\t\t tl["+node+"] = "+node.tl);
////					System.out.println("thetaR["+node+"] = "+node.thetaR+"\t\t tr["+node+"] = "+node.tr);
////					System.out.println("clientes["+node+"] = "+node.clientes);
//					System.out.println("---------------------");
//					
//				}
//		    	
////		    	Iterator<E> iterEdges = newModel.inst.net.goodSecs.iterator();
//		    	System.out.println("|goodSecs| = "+newModel.inst.net.transferSecs.size());
////		    	while (iterEdges.hasNext()) {
////					
////						E edge = iterEdges.next();
////						System.out.println(edge);
////				
////				}				
//				
//				
////		    	System.out.println("DECmin = "+newModel.inst.decMin(newModel.inst.raiz));
////		    	Iterator<E> iterEdges = newModel.inst.g.getEdges().iterator();
////				while (iterEdges.hasNext()) {
////					
////						E edge = iterEdges.next();
////						if (edge.status != SwitchType.PROT) {						
////							System.out.println("DEC = "+newModel.inst.removeSwitch(edge));
////						}
////				
////				}
////				
////				System.out.println(" ");
////				System.out.println("DECmax = "+newModel.inst.decMax(newModel.inst.raiz));
////				iterEdges = newModel.inst.g.getEdges().iterator();
////				while (iterEdges.hasNext()) {
////					
////					E edge = iterEdges.next();
////					if (edge.status != SwitchType.PROT) {
////						System.out.println("DEC = "+newModel.inst.putSwitch(edge));
////					}
////				
////				}				
//		    	
//		    	
//				
//			} catch (FileNotFoundException e) {
//				System.err.println("Arquivo nao encontrado");
//			}
//	}
	
	public void findSubNode(V node, V subID) {
		
		if (node.type == TYPES.SUBSTATION)
			subID = node;
			
		node.subNode = subID;
		
		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			findSubNode(edge.node2,subID);
		
		}
	    
	}
	
	public V pred(V node) {
		
		if (node == this.root)
			return null;
		
		Iterator<E> predEdges = g.getInEdges(node).iterator();
		E predEdge = predEdges.next();
		return predEdge.node1;
		
		
	}	
	
	
	public boolean isAncestor(V ancestor, V node) {
		
		if (ancestor == root)
			return true;
		
		while (node != root) {
			if (node == ancestor)
				return true;
			node = this.pred(node);
		}
		
		return false;
		
	}
	
	public boolean isDownstream(V nodeDown, V nodeUp) {
		
		return this.isAncestor(nodeUp, nodeDown);
		
	}	
	
	public boolean isUpstream(V nodeUp, V nodeDown) {
		
		return (!this.isDownstream(nodeUp, nodeDown));
		
	}		
	
	public double calcClientsSum(V node) {
		
		node.clientsSum = node.clients;
			
		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			node.clientsSum += calcClientsSum(edge.node2);
		
		}
		
		return node.clientsSum;
	    
	}
	
	public double calcDemandSum(V node) {
		
		node.demandSum = node.demand;
		
		Iterator<E> iterEdges = g.getOutEdges(node).iterator();
		while (iterEdges.hasNext()) {
			
			E edge = iterEdges.next();
			node.demandSum += calcDemandSum(edge.node2);
		
		}
		
		return node.demandSum;
	    
	}		
	
	private void findGoodSecs() {
		
		int idGoodSec = 0;
		this.transferSecs = new HashSet<E>();
		
		Iterator<E> iterEdges = ties.iterator();
		while (iterEdges.hasNext()) {
			
			E tie = iterEdges.next();
			V predNode1 = tie.node1;
			V predNode2 = tie.node2;
			
			//se algum dos nos for o noh raiz, entao considera-se que a capacidade de transferencia eh ilimitada
			if (predNode1.label == 0) {
				
				predNode1 = tie.node2;
				predNode2 = tie.node1;
				
			}

			//se algum dos nos for o noh raiz, entao considera-se que a capacidade de transferencia eh ilimitada			
			if (predNode2.label == 0) {
			
				while (predNode1 != this.root) {
					
					Iterator<E> predEdges1 = g.getInEdges(predNode1).iterator();
					
					E predEdge1 = predEdges1.next();	
					
					//if ((predEdge1.node1 != this.raiz)&&(predEdge1.node1.label > 1)) {
					if (predEdge1.node1 != this.root) {
						if (this.transferSecs.add(predEdge1))
							predEdge1.idGoodSec = idGoodSec++;
					} else {
						break;
					}
					
					predNode1 = predEdge1.node1;					
					
				}
				
			}			
			
			if (predNode1.subNode == predNode2.subNode) {
				
				//repita duas vezes
				for (int i=0;i<2;i++) {
					
					while (predNode1 != this.root) {
						
//						predNode2 = tie.node2;
						Iterator<E> predEdges1 = g.getInEdges(predNode1).iterator();
						
						E predEdge1 = predEdges1.next();	
						
						predNode1 = predEdge1.node1;
	
						while (predNode2 != this.root) {
							
							if (predNode1 == predNode2)
								break;
							
							Iterator<E> predEdges2 = g.getInEdges(predNode2).iterator();
							
							E predEdge2 = predEdges2.next();
							predNode2 = predEdge2.node1;
						}
						
						//if ((predNode1 != predNode2)&&(predEdge1.node1 != this.raiz)&&(predEdge1.node1.label > 1)) {
						if ((predNode1 != predNode2)&&(predEdge1.node1 != this.root)) {
							if (this.transferSecs.add(predEdge1))
								predEdge1.idGoodSec = idGoodSec++;
						} else {
							break;
						}
						
					}
					
					//faca a mesma coisa soh que com os nos invertidos
					predNode1 = tie.node2;
					predNode2 = tie.node1;
				
				}
				
			} else {
				
				//repita duas vezes
				for (int i=0;i<2;i++) {
					
					while (predNode1 != this.root) {
	
						Iterator<E> predEdges1 = g.getInEdges(predNode1).iterator();
					
						E predEdge1 = predEdges1.next();
						
						//if ((predNode1.demandaAcum <= predNode2.subNode.remainCapacity)&&(predEdge1.node1 != this.raiz)&&(predEdge1.node1.label > 1)) {
						if ((predNode1.demandSum <= predNode2.subNode.remainCapacity)&&(predEdge1.node1 != this.root)) {
							if (this.transferSecs.add(predEdge1))
								predEdge1.idGoodSec = idGoodSec++;
							predNode1 = predEdge1.node1;
						} else {
							break;
						}
						
					}
					
					//faca a mesma coisa soh que com os nos invertidos
					predNode1 = tie.node2;
					predNode2 = tie.node1;
				
				}				
				
			}
		
		}
	    
	}
	
	private void calcRemainingCapacity() {
		
		Iterator<V> iterNodes = g.getVertices().iterator();
		while (iterNodes.hasNext()) {
			
			V node = iterNodes.next();	
			if (node.type == TYPES.SUBSTATION)
				node.remainCapacity = node.capacity - node.demandSum;	

		}
	    
	}	
	
	public Graph<V, E> getG() {
		return g;
	}

	public Set<E> getTies() {
		return ties;
	}

	public Set<E> getTransferSecs() {
		return transferSecs;
	}

	public V getRoot() {
		return root;
	}

	public void setRoot(V node) {
		this.root = node;
	}	
	
	public int getNumProt() {
		return numProt;
	}

	public void setNumProt(int numProt) {
		this.numProt = numProt;
	}

	public int getNumNoProt() {
		return numNoProt;
	}
	
	public void setNumNoProt(int numNoProt) {
		this.numNoProt = numNoProt;
	}
	
	public Map<Integer, E> getMapEdgeIndex() {
		return mapEdgeIndex;
	}

	public void setMapEdgeIndex(Map<Integer, E> mapEdgeIndex) {
		this.mapEdgeIndex = mapEdgeIndex;
	}
	
	public void findSuccessors(V node) {

		node.nextBranch = null;
		node.firstSucessor = null;
		node.numSubNodes = 0;
		node.ENDmin = node.thetaR*node.demandSum;
		Iterator<V> iterNodes = g.getSuccessors(node).iterator();
		
		if (iterNodes.hasNext()) {
			
			V suc = iterNodes.next();
			node.firstSucessor = suc;
			findSuccessors(suc);
			V oldSuc = suc;
			node.numSubNodes += suc.numSubNodes+1;
			node.ENDmin += suc.ENDmin;
		
			while (iterNodes.hasNext()) {

				suc = iterNodes.next();
				findSuccessors(suc);
				oldSuc.nextBranch = suc;
				oldSuc = suc;
				node.numSubNodes += suc.numSubNodes+1;
				node.ENDmin += suc.ENDmin;
		
			}	
			
			oldSuc.nextBranch = null;
		}
	    
	}

}
