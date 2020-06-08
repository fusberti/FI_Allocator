/*
 * Modelo de alocação de sinalizadores para redes de distribuição de energia
 * 
 * Ideias para futuro: 
 * - avaliar o impacto economico da alocação de sinalizadores, ou seja, a quantidade de sinalizadores
 * passa a ser uma variável do modelo. 
 * 
 */

package models.switches.gurobi;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.stream.Collectors;

import edu.uci.ics.jung.graph.Graph;
import gurobi.GRB;
import gurobi.GRBCallback;
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
import instances.networks.vertices.V;

public class Solver extends GRBCallback {

	public Instance inst;
	public static Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
	public GRBVar[] x[], y, z;
	
	private int edgeGroup[];
	private boolean visitedEdges[];

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

					// Writer outFileInst = new BufferedWriter(new
					// FileWriter("lowerbounds/lowerbounds.txt"));

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

//						GRBVar[] results = 	model.getVars();
//						for (GRBVar var : results)
//							System.out.printf("%s = %f\n", var.get(GRB.StringAttr.VarName), var.get(GRB.DoubleAttr.X));

//						for (E edge : g.getEdges())
//							System.out.printf("Arco %d (%d,%d) lengnt: %.1f\n", edge.id, edge.node1.id, edge.node2.id, edge.dist);
//
//						for (V vertex1 : g.getVertices())
//							for (V vertex2 : g.getVertices())
//								System.out.printf("Arco(%d,%d) %s\n", vertex1.id, vertex2.id,
//										g.findEdge(vertex1, vertex2));

						// System.exit(0);

						GRBVar[] fvars = model.getVars();
						double[] x = model.get(GRB.DoubleAttr.X, fvars);
						String[] vnames = model.get(GRB.StringAttr.VarName, fvars);

						for (int j = 0; j < fvars.length; j++) {
							if (x[j] != 0.0) {
								System.out.println(vnames[j] + " " + x[j]);
							}
						}
//						System.out.println("maxSwitches " + gurobi.inst.parameters.getNumSwitches());

						model.dispose();
						env.dispose();

						stok.nextToken();

					}

					// outFileInst.close();

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

		for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
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

	// variaveis tipo X -- x_kp = 1 indica que o arco k pertence a parte p
	private void defineVarX(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo X
		x = new GRBVar[g.getEdges().size()][inst.parameters.getNumFI() + 1];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsX = 0.0f;
				double lbX = 0.0;
				double ubX = 1.0;
				for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
					x[edge.id][k] = model.addVar(lbX, ubX, 0.0f, tipoBinary, "x[" + edge.id + "," + k + "]");
					// ofexpr.addTerm(objvalsX, x[edge.id][k]);
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// variaveis tipo Y -- y_ij = 1 indica que na fronteira do no i existe um
	// sinalizador
	private void defineVarY(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo Y
		y = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = 1.0;
				y[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "y[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// variaveis tipo Z -- z_ij = 1 indica que na fronterira do no j existe um
	// sinalizador
	private void defineVarZ(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo Y
		z = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = 1.0;
				z[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "z[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, z[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// RESTRICAO (0): sum x_ij = 1 \in k
	// Particao de arestas.
	private void addConstraint0(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				GRBLinExpr constraint = new GRBLinExpr();
				for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
					constraint.addTerm(1, x[edge.id][k]);
				}
				model.addConstr(constraint, GRB.EQUAL, 1, "c0_"+edge.id);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	public Collection<E> getEdgesInPath(V orig, V dest) {
		List<E> path = new ArrayList<E>();
		E edge = g.findEdge(orig, dest);
		if ((edge = g.findEdge(orig, dest)) != null || (edge = g.findEdge(dest, orig)) != null) {
			path.add(edge);
			return path;
		}
		List<V> predecessorsOrig = new ArrayList<V>();
		List<V> predecessorsDest = new ArrayList<V>();
		V pred = orig;
		predecessorsOrig.add(pred);
		while (g.getPredecessors(pred).iterator().hasNext()) {
			pred = g.getPredecessors(pred).iterator().next();
			predecessorsOrig.add(pred);
		}

		predecessorsDest.add(dest);
		int idx = predecessorsOrig.indexOf(dest);
		pred = dest;
		while (g.getPredecessors(pred).iterator().hasNext() && idx < 0) {
			pred = g.getPredecessors(pred).iterator().next();
			idx = predecessorsOrig.indexOf(pred);
			predecessorsDest.add(pred);
		}
		List<V> predecessors;
		predecessors = new ArrayList<V>(predecessorsOrig.subList(0, idx + 1));
		ListIterator<V> iterPred = predecessorsDest.listIterator(predecessorsDest.size() - 1);
		while (iterPred.hasPrevious()) {
			predecessors.add(iterPred.previous());
		}
		Iterator<V> iter = predecessors.iterator();
		V next, start = iter.next();
		while (iter.hasNext()) {
			next = iter.next();
			edge = g.findEdge(start, next);
			if (edge == null)
				path.add(g.findEdge(next, start));
			else
				path.add(edge);
			start = next;
		}
//		System.out.printf("%d->%d: %s\n", orig.id, dest.id, predecessors);
//		System.out.printf("%d->%d: %s\n", orig.id, dest.id, path);
		return path;
	}

	// RESTRICAO (1): x^k_a + x^k_c <= 1 + x^k_b k \in K, {a, c \in A}, b \in P(a,c)
	// Conectividade de cada grupo. Se existir um par de arestas "a" e "c" em um
	// mesmo grupo
	// "k", então toda aresta do caminho entre "a" e "c" deve fazer parte do grupo
	// "k".
	private void addConstraint1(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edgeA = iterEdges.next();
				for (E edgeC : g.getEdges()) {
					if (edgeC != edgeA && edgeC.node1 != edgeA.node1 && edgeC.node1 != edgeA.node2
							&& edgeC.node2 != edgeA.node1 && edgeC.node2 != edgeA.node2) {
						V orig, dest;
						if (g.isSource(edgeA.node1, edgeA)) // se i=node1 e j=node2 nao precisa
							orig = edgeA.node1;
						else
							orig = edgeA.node2;
						if (g.isSource(edgeC.node1, edgeC))
							dest = edgeC.node1;
						else
							dest = edgeC.node2;
						Iterator<E> iterPredEdges = getEdgesInPath(orig, dest).iterator();
						while (iterPredEdges.hasNext()) {
							E edgeB = iterPredEdges.next();
							if (edgeB == edgeA || edgeB == edgeC)
								continue;
							for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
								GRBLinExpr constraint = new GRBLinExpr();
								constraint.addTerm(1, x[edgeA.id][k]);
								constraint.addTerm(1, x[edgeC.id][k]);
								constraint.addTerm(-1, x[edgeB.id][k]);
								model.addConstr(constraint, GRB.LESS_EQUAL, 1, "c1_"+edgeA.id+","+edgeB.id+","+edgeC.id+","+k);
							}
						}
					}
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// RESTRICAO (2): sum(y_ij + z_ij) = |K|-1
	// Quantidade de sinalizadores
	private void addConstraint2(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			GRBLinExpr constraint = new GRBLinExpr();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				constraint.addTerm(1, y[edge.id]);
				constraint.addTerm(1, z[edge.id]);
			}
			model.addConstr(constraint, GRB.EQUAL, inst.parameters.getNumFI(), "c2");
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// RESTRICAO (3): x^k_{ij} + x^k_a \leqslant 2 - y_{ij} & \forall (i,j) \in A,
	// \forall k \in K, \forall a \in \delta(i) / {j}
	// x^k_{ij} + x^k_a \leqslant 2 - z_{ij} & \forall (i,j) \in A, \forall k \in K,
	// \forall a \in \delta(j) / {i}
	// Restrições de fronteira. Uma vez que um sinalizador é alocado, verifica-se as
	// arestas incidentes na vizinhança do sinalizador. As restrições impedem que
	// duas arestas vizinhas separadas por um sinalizador estejam no mesmo grupo.
	private void addConstraint3(GRBModel model) {

		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); // (i,j)
				V orig, dest;
				if (g.isSource(edge.node1, edge)) {
					orig = edge.node1;
					dest = edge.node2;
				} else {
					orig = edge.node2;
					dest = edge.node1;
				}
				Iterator<E> iterEdgesInc = g.getIncidentEdges(orig).iterator();
				while (iterEdgesInc.hasNext()) {
					E edgeA = iterEdgesInc.next();
					if (edge != edgeA) {
						for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
							constraint.addTerm(1.0, x[edge.id][k]);
							constraint.addTerm(1.0, x[edgeA.id][k]);
							constraint.addTerm(1.0, y[edge.id]);
							model.addConstr(constraint, GRB.LESS_EQUAL, 2, "c3y_"+edge.id+","+edgeA.id+","+k);
							constraint.clear();
						}
					}
				}
				// arestas incidentes em j
				iterEdgesInc = g.getIncidentEdges(dest).iterator();
				while (iterEdgesInc.hasNext()) {
					E edgeA = iterEdgesInc.next();
					if (edge != edgeA) {
						for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
							constraint.addTerm(1.0, x[edge.id][k]);
							constraint.addTerm(1.0, x[edgeA.id][k]);
							constraint.addTerm(1.0, z[edge.id]);
							model.addConstr(constraint, GRB.LESS_EQUAL, 2, "c3z_"+edge.id+","+edgeA.id+","+k);
							constraint.clear();
						}
					}
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// x^{k_1}_{ij} + x^{k_2}_{j,l} \leqslant 1 + y_{ij} + z_{ij} & \forall
	// (i,j),(j,l) \in A, \forall k_1,k_2 \in K, k_1 \neq k_2
	// completa a restricao de fronteira, se as arestas vizinhas sao de grupos
	// diferentes entao tem que ter um sinalizador entre elas
	private void addConstraint4(GRBModel model) {
		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); // (i,j)
				V dest = edge.node2;
				if (g.isSource(edge.node2, edge))
					dest = edge.node1;
				Iterator<E> iterEdgesOut = g.getOutEdges(dest).iterator();
				while (iterEdgesOut.hasNext()) {
					E edgeOut = iterEdgesOut.next();
					for (int k1 = 0; k1 <= this.inst.parameters.getNumFI(); k1++)
						for (int k2 = 0; k2 <= this.inst.parameters.getNumFI(); k2++)
							if (k1 != k2) {
								constraint.addTerm(1.0, x[edge.id][k1]);
								constraint.addTerm(1.0, x[edgeOut.id][k2]);
								constraint.addTerm(-1.0, z[edge.id]);
								constraint.addTerm(-1.0, y[edgeOut.id]);
								model.addConstr(constraint, GRB.LESS_EQUAL, 1, "c4_"+edge.id+","+edgeOut.id+","+k1+","+k2);
								constraint.clear();
							}
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// Forca as variaveis y[0]=0 e as z[ij]=0 para todo ij se j eh no folha
	private void setContourVars(GRBModel model) {
		try {
			GRBLinExpr constraint = new GRBLinExpr();
			// y[0]=0 for the root node
			constraint.addTerm(1.0, y[0]);
			model.addConstr(constraint, GRB.EQUAL, 0, "contoury0");
			constraint.clear();
			// z[ij]=0 when j is a leaf
//			Iterator<E> iterEdges = g.getEdges().iterator();
//			V orig, dest;
//			while (iterEdges.hasNext()) {
//				E edge = iterEdges.next(); // (i,j)
//				if (g.isSource(edge.node1, edge)) {
//					orig = edge.node1;
//					dest = edge.node2;
//				} else {
//					orig = edge.node2;
//					dest = edge.node1;
//				}
//
//				if (g.getSuccessorCount(dest) == 0) {
//					constraint.addTerm(1.0, z[edge.id]);
//					model.addConstr(constraint, GRB.EQUAL, 0, "contourz");
//					constraint.clear();
//				}
//			}

			g.getVertices().stream().filter(v -> g.getIncidentEdges(v).size() < 2 && v.id != 0).forEach(v -> {
				try {
					constraint.addTerm(1.0, z[g.getInEdges(v).iterator().next().id]);
					model.addConstr(constraint, GRB.EQUAL, 0, "contourz_"+v.id);
					constraint.clear();
				} catch (GRBException e) {
					System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
				}
			});

			// System.out.println(g.getVertices().stream().filter(v->g.getIncidentEdges(v).size()<2
			// && v.id!=0).collect(Collectors.toList()));

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// Forca as variaveis y[0]=0 e as z[ij]=0 para todo ij se j eh no folha
	private void setSymmetryBreaking(GRBModel model) {
		try {
			
			int k = 0;
			
			
			//Iterator<E> iterEdges = g.getEdges().iterator();
			List<E> shuffleEdges = new ArrayList<E>(g.getEdges());
			Collections.shuffle(shuffleEdges);
			Iterator<E> iterEdges = shuffleEdges.iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); 
				
				GRBLinExpr constraint = new GRBLinExpr();
				for (int k2 = 0; k2 <= k; k2++) {
					constraint.addTerm(1.0, x[edge.id][k2]);
				}
				model.addConstr(constraint, GRB.EQUAL, 1, "symmetry"+k);
				k++;
				if (k == inst.getParameters().getNumFI()) break;
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

			GRBQuadExpr ofexpr = new GRBQuadExpr();

			// Create variables
			this.defineVarX(model, 0, ofexpr);
			this.defineVarY(model, 1, ofexpr);
			this.defineVarZ(model, 2, ofexpr);
			this.generateOF(model, ofexpr);

			// Integrate new variables
			model.update();

			model.setObjective(ofexpr);

			// RESTRICAO (0): sum x_ij = 1 \in k
			this.addConstraint0(model);

			// RESTRICAO (1): x^k_a + x^k_c <= 1 + x^k_b k \in K, {a, c \in A}, b \in P(a,c)
			this.addConstraint1(model);

			// RESTRICAO (2): de fronteira em y
			this.addConstraint2(model);

			// RESTRICAO (3): de fronteira em z
			this.addConstraint3(model);

			// RESTRICAO (3): de fronteira com y e z
			this.addConstraint4(model);

			// fixa variaveis de contorno
			this.setContourVars(model);
			
			// eliminação de simetria
			//this.setSymmetryBreaking(model);

			model.update();

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	public Instance getInst() {
		return inst;
	}
	
	void checkConnectivity(V root, int k) {
		
		Iterator<E> iterEdges = g.getIncidentEdges(root).iterator();
		
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();	
			if (edgeGroup[edge.id] == k && !visitedEdges[edge.id]) {
				V next = (edge.node1 != root) ? edge.node1 : edge.node2; 
				visitedEdges[edge.id] = true;
				checkConnectivity(next,k);
			}
		}
 
	}
	

	@Override
	  protected void callback() {
			  
//		visitedEdges = new boolean[g.getEdgeCount()];
//		edgeGroup = new int[g.getEdgeCount()];
//		E firstInGroup[] = new E[inst.getParameters().getNumFI()+1];
//		  
//	    try {
//	    	
//	      if (where == GRB.CB_MIPSOL) {
//	        // Found an integer feasible solution - does it visit every node?
//	        double x_val[][] = getSolution(x);
//	        Iterator<E> iterEdges;
//	        
//	        
//			iterEdges = g.getEdges().iterator();
//			while (iterEdges.hasNext()) {
//				E edge = iterEdges.next();
//		        for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
//		        	if (x_val[edge.id][k] > 0.5) {
//		        		edgeGroup[edge.id] = k;
//		        		firstInGroup[k] = edge;
//		        	}
//		        }      	
//	        }
//			
//			
//	        for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
//	        	boolean visited[] = new boolean[g.getVertexCount()];
//				E edgeA = firstInGroup[k];
//				visited[edgeA.node1.id] = true;
//				V root = edgeA.node1;
//				//visited[edge.node2.id] = true;
//							
//				checkConnectivity(root, k);
//				
//				iterEdges = g.getEdges().iterator();
//				E edgeC = null;
//				while (iterEdges.hasNext()) {
//					edgeC = iterEdges.next();			
//					if (edgeGroup[edgeC.id] == k && !visitedEdges[edgeC.id]) {
//						break; // encontrei
//					}
//				}
//				
//				// inserir restricao
//				if (edgeC != null) {
//					iterEdges = getEdgesInPath(edgeA.node1,edgeC.node1).iterator();
//					while (iterEdges.hasNext()) {
//						E edgeB = iterEdges.next();	
//						if (edgeB == edgeA || edgeB == edgeC)
//							continue;
//						GRBLinExpr constraint = new GRBLinExpr();
//						constraint.addTerm(1, x[edgeA.id][k]);
//						constraint.addTerm(1, x[edgeC.id][k]);
//						constraint.addTerm(-1, x[edgeB.id][k]);
//						addLazy(constraint, GRB.LESS_EQUAL, 1); //, "c1_lazy_"+edgeA.id+","+edgeB.id+","+edgeC.id+","+k);
//						break;
//					}
//				}
//				
//	        }  
//	      }
//	    } catch (GRBException e) {
//	      System.out.println("Error code: " + e.getErrorCode() + ". " +
//	          e.getMessage());
//	      e.printStackTrace();
//	    }
	  }
	
}
