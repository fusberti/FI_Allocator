package models.switches.gurobi;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import edu.uci.ics.jung.algorithms.shortestpath.DijkstraShortestPath;
import edu.uci.ics.jung.graph.Graph;
import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBQuadExpr;
import gurobi.GRBVar;

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

public class Solver {

	public Instance inst;
	Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
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

						// Configura os parametros do solver Gurobi
						new GurobiParameters(model);
						gurobi.populateNewModel(model);

						// Write model to file
						model.write("FI_Allocation.lp");

						model.optimize();

						System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));

						System.out.println("maxSwitches " + gurobi.inst.parameters.getNumSwitches());

						model.dispose();
						env.dispose();

						stok.nextToken();

					}

					outFileInst.close();

					System.out.println(
							"Otimizacao encerrada, resultados impressos em " + "resultados/" + instanciaNome + ".out");

				} catch (GRBException e) {
					System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
				}
			} catch (FileNotFoundException e) {
				System.err.println("Arquivo nao encontrado");
			}
		} catch (IOException e) {
			System.err.println("Erro leitura da instancia");
		}

	}

	// generate the quadractic objective function
	private void generateOF(GRBModel model, GRBQuadExpr ofexpr) {

		for (int k = 0; k <= inst.parameters.numFI; k++) {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					double objvalsX = edge.length * edge2.length;
					ofexpr.addTerm(objvalsX, x[edge.id][k], x[edge2.id][k]);
				}
			}
		}

	}

	// variaveis tipo X -- x_kp = 1 indica que o arco k pertence a parte p
	private void defineVarX(GRBModel model, int indVar, GRBLinExpr ofexpr) {

		// variaveis tipo X
		x = new GRBVar[g.getEdges().size()][inst.parameters.numFI + 1];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double objvalsX = 0.0f;
				double lbX = 0.0;
				double ubX = 1.0;
				for (int k = 0; k < inst.parameters.numFI; k++) {
					x[edge.id][k] = model.addVar(lbX, ubX, objvalsX, tipoBinary, "x[" + edge.id + "," + k + "]");
					ofexpr.addTerm(objvalsX, x[edge.id][k]);
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// variaveis tipo Y -- y_ij = 1 indica que na fronteira do no i existe um
	// sinalizador
	private void defineVarY(GRBModel model, int indVar, GRBLinExpr ofexpr) {

		// variaveis tipo Y
		y = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = 1.0;
				y[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "y[" + edge.id + "]");
				ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// variaveis tipo Z -- z_ij = 1 indica que na fronterira do no j existe um
	// sinalizador
	private void defineVarZ(GRBModel model, int indVar, GRBLinExpr ofexpr) {

		// variaveis tipo Y
		z = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = 1.0;
				z[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "z[" + edge.id + "]");
				ofexpr.addTerm(objvalsY, z[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// RESTRICAO (0): sum x_ij = 1 \in k
	private void addConstraint0(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				GRBLinExpr constraint = new GRBLinExpr();
				for (int k = 0; k <= this.inst.parameters.numFI; k++) {
					constraint.addTerm(1, x[edge.id][k]);
				}
				model.addConstr(constraint, GRB.EQUAL, 1, "c0");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	public Collection<E> getEdgesInPath(V orig, V dest) {
		DijkstraShortestPath<V, E> dsp = new DijkstraShortestPath<V, E>(g);
		List<E> path = dsp.getPath(orig, dest);
		return path;
	}

	// RESTRICAO (1): x^k_a + x^k_c <= 1 + x^k_b k \in K, {a, c \in A}, b \in P(a,c)
	// TO DO
	private void addConstraint1(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {			
				E edgeA = iterEdges.next();
				for (E edgeC : g.getEdges()) {
					if (edgeC != edgeA && edgeC.node1 != edgeA.node1 && edgeC.node1 != edgeA.node2
							&& edgeC.node2 != edgeA.node1 && edgeC.node2 != edgeA.node2) {
						V orig, dest;
						if (g.isSource(edgeA.node1, edgeA)) //se i=node1 e j=node2 nao precisa
							orig = edgeA.node1;
						else
							orig = edgeA.node2;
						if (g.isSource(edgeC.node1, edgeC))
							dest = edgeC.node1;
						else
							dest = edgeC.node2;
						Iterator<E> iterPredEdges = getEdgesInPath(orig, dest).iterator();
						while (iterPredEdges.hasNext()) {
							E edgeB = iterEdges.next();
							if (edgeB == edgeA)
								break;
							for (int k = 0; k <= this.inst.parameters.numFI; k++) {
								GRBLinExpr constraint = new GRBLinExpr();
								constraint.addTerm(1, x[edgeA.id][k]);
								constraint.addTerm(1, x[edgeC.id][k]);
								constraint.addTerm(-1, x[edgeB.id][k]);
								model.addConstr(constraint, GRB.LESS_EQUAL, 1, "c1");
							}
						}
					}
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// RESTRICAO (2): sum(y_ij + z_ij) = |K|-1 (i,j \in A)
	private void addConstraint2(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			GRBLinExpr constraint = new GRBLinExpr();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				constraint.addTerm(1, y[edge.id]);
				constraint.addTerm(1, z[edge.id]);
			}
			model.addConstr(constraint, GRB.LESS_EQUAL, inst.parameters.numFI - 1, "c2");
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// RESTRICAO (3): x^k_{ij} + x^k_a \leqslant 2 - y_{ij} & \forall (i,j) \in A,
	// \forall k \in K, \forall a \in \delta(i) / {j}
	// x^k_{ij} + x^k_a \leqslant 2 - z_{ij} & \forall (i,j) \in A, \forall k \in K,
	// \forall a \in \delta(j) / {i}
	private void addConstraint3(GRBModel model) {

		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); //(i,j) 
				// arestas incidentes do no i se i=node1 j=node2
				Iterator<E> iterEdgesInc = g.getIncidentEdges(edge.node1).iterator();
				while (iterEdgesInc.hasNext()) {
					E edgeA = iterEdgesInc.next();
					if (edge!= edgeA) {
						for (int k = 0; k <= this.inst.parameters.numFI; k++) {
							constraint.addTerm(1.0, x[edge.id][k]);
							constraint.addTerm(1.0, x[edgeA.id][k]);
							constraint.addTerm(-1.0, y[edge.id]);
							model.addConstr(constraint, GRB.LESS_EQUAL, 2, "c3y");
							constraint.clear();
						}
					}
				}
				// arestas incidentes em j
				iterEdgesInc = g.getIncidentEdges(edge.node2).iterator();
				while (iterEdgesInc.hasNext()) {
					E edgeA = iterEdgesInc.next();
					if (edge != edgeA) {
						for (int k = 0; k <= this.inst.parameters.numFI; k++) {
							constraint.addTerm(1.0, x[edge.id][k]);
							constraint.addTerm(1.0, x[edgeA.id][k]);
							constraint.addTerm(-1.0, z[edge.id]);
							model.addConstr(constraint, GRB.LESS_EQUAL, 2, "c3z");
							constraint.clear();
						}
					}					
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	
	// Essas restricoes "apertam" o modelo
	// RESTRICAO (6): fij <= theta_j + sum_{(j,k) \in A}(fjk) (all (i,j) in E)
	// private void addConstraint6(GRBModel model) {
	//
	// try {
	//
	// GRBLinExpr constraint = new GRBLinExpr();
	//
	// Iterator<E> iterEdges = g.getEdges().iterator();
	// while (iterEdges.hasNext()) {
	//
	// E edge = iterEdges.next();
	//
	// if (edge.status != SwitchType.PROT) {
	//
	// constraint.clear();
	// constraint.addTerm(1.0, f[edge.idNoProt]);
	//
	// //arestas saindo do no
	// Iterator<E> iterEdgesOut = g.getOutEdges(edge.node2).iterator();
	// while (iterEdgesOut.hasNext()) {
	// E outEdge = iterEdgesOut.next();
	// if (outEdge.status != SwitchType.PROT)
	// constraint.addTerm(-1.0, f[outEdge.idNoProt]);
	// }
	// model.addConstr(constraint, GRB.LESS_EQUAL, edge.node2.thetaR,
	// "c6["+edge.node1.label+","+edge.node2.label+"]");
	//
	// }
	//
	// }
	//
	// } catch (GRBException e) {
	// System.out.println("Error code: " + e.getErrorCode() + ". " +
	// e.getMessage());
	// }
	//
	// }

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
				ofexpr.addConstant(
						inst.parameters.getKwhCost() * (inst.reliability.getPtheta() + inst.reliability.getSumPFl()));
				break;
			case ENS:
				ofexpr.addConstant(inst.reliability.getPtheta() + inst.reliability.getSumPFl());
				break;
			case SAIDI:
				ofexpr.addConstant(
						(inst.reliability.getNtheta() + inst.reliability.getSumNFl()) / inst.net.getRoot().clientsSum);
				break;
			}
			model.setObjective(ofexpr);

			// Constraint (0): sum X <= numMax
			this.addConstraint0(model);

			// Constraint (1): fij >= theta_j + sum_{(j,k) \in A}(fjk) - Mxij (all (i,j) in
			// E)
			this.addConstraint1(model);

			// if the SAIDI is bounded
			if (this.inst.parameters.getLimSAIDI() < Double.MAX_VALUE) {

				// Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in
				// goodSecs)
				this.addConstraint2(model);

			}

			// if the ENS is bounded
			if (this.inst.parameters.getLimENS() < Double.MAX_VALUE) {

				// Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in
				// goodSecs)
				this.addConstraint3(model);

			}

			// if there is load transfer through tie lines
			if (this.inst.parameters.isTransfer()) {

				// Constraint (4): sum{(u,v) \in path(0,j)}{dtuv} <= ti - fij (all (i,j) in
				// goodSecs)
				this.addConstraint4(model);

				// Constraint (5): dt <= M*xij (all (i,j) in goodSecs)
				this.addConstraint5(model);

			}

			model.update();

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	public Instance getInst() {
		return inst;
	}

}
