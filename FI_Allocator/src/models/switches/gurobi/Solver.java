package models.switches.gurobi;

// Parametros do modelo
// d_k : comprimento do arco k (km)
// d_i : distancia da SE ate no i (km)
// alpha : inverso da velocidade
// ro : taxa de falha por km
// P : qtd de sinalizadores +1


import instances.Instance;
import instances.networks.edges.E;
import instances.networks.edges.E.SwitchType;
import instances.networks.vertices.V;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;
import java.io.Writer;
import java.util.Iterator;

import edu.uci.ics.jung.graph.Graph;

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

public class Solver {

	public Instance inst;
	Graph<V, E> g;	

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv    env;
	public static GRBModel  model;		
	public GRBVar[] x[], y, z;

	public Solver(String filename) {	
		this.inst = new Instance();
		this.g = this.inst.net.getG();	
	}

	public static void main(String[] args) {

		Solver gurobi = null;
		String instanciaNome = null;	

		try {		
			try {
				try {				

					Reader fileInst = new BufferedReader(new FileReader("instancias.txt"));
					StreamTokenizer stok = new StreamTokenizer(fileInst);

					Writer outFileInst = new BufferedWriter(new FileWriter("lowerbounds/lowerbounds.txt"));	

					stok.nextToken();
					while (stok.sval != null) {

						instanciaNome = stok.sval;

						gurobi = new Solver("instancias/" + instanciaNome);	
						
						System.out.println(gurobi.getInst().getParameters().getInstanceName());

						env = new GRBEnv("mip1.log");
						model = new GRBModel(env);		

						//Configura os parametros do solver Gurobi
						new GurobiParameters(model);
						gurobi.populateNewModel(model);					

						// Write model to file
						model.write("FI_Allocation.lp");												      

						model.optimize();

						System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));

						System.out.println("maxSwitches "+gurobi.inst.parameters.getNumSwitches());

						model.dispose();
						env.dispose();						

						stok.nextToken();

					}

					outFileInst.close();	

					System.out.println("Otimizacao encerrada, resultados impressos em "+"resultados/" + instanciaNome + ".out");

				} catch (GRBException e) {
					System.out.println("Error code: " + e.getErrorCode() + ". " +
							e.getMessage());
				}
			} catch (FileNotFoundException e) {
				System.err.println("Arquivo nao encontrado");
			}
		} catch (IOException e) {
			System.err.println("Erro leitura da instancia");
		}		

	}

	// generate the quadractic objective function
	private void generateOF(GRBModel model, GRBLinExpr ofexpr) {
		
		for (int k=0; k <= inst.parameters.numFI; k++) {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					double objvalsX = edge.dist * edge2.dist;
					ofexpr.addTerm(objvalsX, x[edge.id][k], x[edge2.id][k]);
				}
			}
		}
		
	}
	
	
	//variaveis tipo X -- x_kp = 1 indica que o arco k pertence a parte p
	private void defineVarX(GRBModel model, int indVar, GRBLinExpr ofexpr) {

		//variaveis tipo X 
		x = new GRBVar[NUM_EDGES][inst.parameters.numFI+1];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double objvalsX = 0.0f;
				double lbX = 0.0;
				double ubX = 1.0;	
				for (int k=0;k<inst.parameters.numFI;k++) {
					x[edge.id][k] = model.addVar(lbX, ubX, objvalsX,tipoBinary,"x["+edge.id+","+k+"]");
					ofexpr.addTerm(objvalsX, x[edge.id][k]);
				}
			}      

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}	

	}
	
	//variaveis tipo Y -- y_i = 1 indica que na fronteira do no i existe um sinalizador
	private void defineVarY(GRBModel model, int indVar, GRBLinExpr ofexpr) {

		//variaveis tipo Y
		y = new GRBVar[g.getVertexCount()];

		try {

			Iterator<V> iterNodes = g.getVertices().iterator();
			while (iterNodes.hasNext()) {
				V node = iterNodes.next();
				double objvalsY = 0.0f;
				// fault indicator
				double lbX = 0.0;
				double ubX = 1.0;			
				y[node.id] = model.addVar(lbX, ubX, 0.0f,tipoBinary,"y["+node.label+"]");
				ofexpr.addTerm(objvalsY, y[node.id]);
			}      

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}	

	}

	//RESTRICAO (0): sum X = 1	
	private void addConstraint0(GRBModel model) {

		try {	

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				GRBLinExpr constraint = new GRBLinExpr();
				for (int p=0;p<this.inst.parameters.numFI;p++) {
					constraint.addTerm(1, x[edge.idNoProt][p]);
				}
					
				model.addConstr(constraint, GRB.EQUAL, inst.parameters.numFI, "c0");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}	

	}

	//RESTRICAO (1): fij >= theta_j + sum_{(j,k) \in A}(fjk) - Mxij (all (i,j) in E)	
	private void addConstraint1(GRBModel model) {

		try {

			GRBLinExpr constraint = new GRBLinExpr();		

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {

				E edge = iterEdges.next();

				if (edge.status != SwitchType.PROT) {

					constraint.clear();
					constraint.addTerm(1.0, f[edge.idNoProt]);	

					constraint.addTerm(edge.node2.M, x[edge.idNoProt]);

					//arestas saindo do no
					Iterator<E> iterEdgesOut = g.getOutEdges(edge.node2).iterator();
					while (iterEdgesOut.hasNext()) {
						E outEdge = iterEdgesOut.next();
						if (outEdge.status != SwitchType.PROT)
							constraint.addTerm(-1.0, f[outEdge.idNoProt]);
					}	
					model.addConstr(constraint, GRB.GREATER_EQUAL, edge.node2.thetaR, "c1["+edge.node1.label+","+edge.node2.label+"]");

				}

			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	//RESTRICAO (2): SAIDI <= SAIDImax	
	private void addConstraint2(GRBModel model) {

		try {

			GRBLinExpr constraint = new GRBLinExpr();

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				if (edge.status != SwitchType.PROT)
					constraint.addTerm((double)(edge.node1.clientsSum-edge.node2.clientsSum)/(double)inst.net.getRoot().clientsSum, f[edge.idNoProt]);
			}
			model.addConstr(constraint, GRB.LESS_EQUAL, inst.parameters.getLimSAIDI()-(inst.reliability.getSumNFl()+inst.reliability.calcNtheta())/(double)inst.net.getRoot().clientsSum, "c2");	

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	//RESTRICAO (3): ENS <= ENSmax	
	private void addConstraint3(GRBModel model) {

		try {

			GRBLinExpr constraint = new GRBLinExpr();

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				if (edge.status != SwitchType.PROT)
					constraint.addTerm((double)(edge.node1.demandSum-edge.node2.demandSum), f[edge.idNoProt]);
			}
			model.addConstr(constraint, GRB.LESS_EQUAL, inst.parameters.getLimENS()-(inst.reliability.calcSumPFl()+inst.reliability.calcPtheta()), "c3");	

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	//RESTRICAO (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in goodSecs)
	//OBS: testar sum{(u,v) \in path(0,j)}{dtuv} <= ti - Fj (errado)
	private void addConstraint4(GRBModel model) {

		try {

			GRBLinExpr constraint = new GRBLinExpr();

			Iterator<E> goodSecs = inst.net.getTransferSecs().iterator();
			while (goodSecs.hasNext()) {

				E goodSec = goodSecs.next();

				constraint.clear();
				constraint.addTerm(1.0f, dt[goodSec.idGoodSec]); // + dtij
				constraint.addTerm(1.0f, f[goodSec.idNoProt]);	// + fij			

				V predNode1 = goodSec.node1;

				double sumTheta = 0.0f;

				// fij - sum_{(j,k) \in A}{fjk}
				while (predNode1 != inst.net.getRoot()) {

					sumTheta += predNode1.thetaR;

					Iterator<E> sucEdges = g.getOutEdges(predNode1).iterator();

					while (sucEdges.hasNext()) {

						E sucEdge = sucEdges.next();
						if (sucEdge.status != E.SwitchType.PROT)
							constraint.addTerm(-1.0f, f[sucEdge.idNoProt]);	// - sum_{(j,k) \in A}{fjk}

					}					

					Iterator<E> predEdges = g.getInEdges(predNode1).iterator();
					E predEdge = predEdges.next();
					if ((predEdge.node1 != inst.net.getRoot())&&(predEdge.status != E.SwitchType.PROT))
						constraint.addTerm(1.0f, f[predEdge.idNoProt]);	// + fij				

					predNode1 = predEdge.node1;

				}

				V predNode2 = goodSec.node1;

				while (predNode2 != inst.net.getRoot()) {

					Iterator<E> predEdges = g.getInEdges(predNode2).iterator();
					E predEdge = predEdges.next();
					if (inst.net.getTransferSecs().contains(predEdge))
						constraint.addTerm(1.0f, dt[predEdge.idGoodSec]); // sum{(u,v) \in path(0,i)}{dtuv}		
					predNode2 = predEdge.node1;

				}

				model.addConstr(constraint, GRB.LESS_EQUAL, sumTheta, "c4["+goodSec.node1.label+","+goodSec.node2.label+"]"); // >= sumTheta			

			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	//RESTRICAO (5): dt <= M*xij (all (i,j) in goodSecs)
	private void addConstraint5(GRBModel model) {

		try {

			GRBLinExpr constraint = new GRBLinExpr();

			Iterator<E> goodSecs = inst.net.getTransferSecs().iterator();
			while (goodSecs.hasNext()) {

				E goodSec = goodSecs.next();
				constraint.clear();
				constraint.addTerm(1.0, dt[goodSec.idGoodSec]);
				constraint.addTerm(-(goodSec.node1.subNode.M-goodSec.node2.M), x[goodSec.idNoProt]);
				model.addConstr(constraint, GRB.LESS_EQUAL, 0.0f, "c5["+goodSec.node1.label+","+goodSec.node2.label+"]");

			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	//Essas restricoes "apertam" o modelo
	//RESTRICAO (6): fij <= theta_j + sum_{(j,k) \in A}(fjk) (all (i,j) in E)
	//private void addConstraint6(GRBModel model) {
	//
	//	try {
	//		
	//		GRBLinExpr constraint = new GRBLinExpr();		
	//		
	//		Iterator<E> iterEdges = g.getEdges().iterator();
	//		while (iterEdges.hasNext()) {
	//	
	//			E edge = iterEdges.next();
	//			
	//			if (edge.status != SwitchType.PROT) {
	//			
	//				constraint.clear();
	//				constraint.addTerm(1.0, f[edge.idNoProt]);	
	//							
	//				//arestas saindo do no
	//				Iterator<E> iterEdgesOut = g.getOutEdges(edge.node2).iterator();
	//				while (iterEdgesOut.hasNext()) {
	//					E outEdge = iterEdgesOut.next();
	//					if (outEdge.status != SwitchType.PROT)
	//						constraint.addTerm(-1.0, f[outEdge.idNoProt]);
	//				}	
	//				model.addConstr(constraint, GRB.LESS_EQUAL, edge.node2.thetaR, "c6["+edge.node1.label+","+edge.node2.label+"]");
	//				
	//			}
	//			
	//		}
	//		
	//	} catch (GRBException e) {
	//		System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
	//	}
	//	
	//}

	private void populateNewModel(GRBModel model) {

		try {

			GRBLinExpr ofexpr = new GRBLinExpr();

			// Create variables
			vars = new GRBVar[3][];
			this.defineVarX(model, 0, ofexpr);
			this.defineVarF(model, 1, ofexpr);
			// se considera transferencia por chaves de manobra 
			if (this.inst.parameters.isTransfer())
				this.defineVarDT(model, 2, ofexpr);		

			// Integrate new variables
			model.update();		

			// Create objective function expression
			switch (inst.parameters.getOBJ()) {
			case COST:
				ofexpr.addConstant(inst.parameters.getKwhCost()*(inst.reliability.getPtheta()+inst.reliability.getSumPFl()));
				break;
			case ENS:
				ofexpr.addConstant(inst.reliability.getPtheta()+inst.reliability.getSumPFl());
				break;
			case SAIDI:
				ofexpr.addConstant((inst.reliability.getNtheta()+inst.reliability.getSumNFl())/inst.net.getRoot().clientsSum);
				break;
			}	
			model.setObjective(ofexpr);

			//Constraint (0): sum X <= numMax
			this.addConstraint0(model);

			//Constraint (1): fij >= theta_j + sum_{(j,k) \in A}(fjk) - Mxij (all (i,j) in E)	
			this.addConstraint1(model);

			// if the SAIDI is bounded 
			if (this.inst.parameters.getLimSAIDI() < Double.MAX_VALUE) {

				//Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in goodSecs)
				this.addConstraint2(model);

			}

			// if the ENS is bounded 
			if (this.inst.parameters.getLimENS() < Double.MAX_VALUE) {

				//Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in goodSecs)
				this.addConstraint3(model);

			}		


			// if there is load transfer through tie lines
			if (this.inst.parameters.isTransfer()) {

				//Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in goodSecs)
				this.addConstraint4(model);

				//Constraint (5): dt <= M*xij (all (i,j) in goodSecs)
				this.addConstraint5(model);		

			}

			model.update();		

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " +
					e.getMessage());
		}			


	}

	public Instance getInst() {
		return inst;
	}

}

