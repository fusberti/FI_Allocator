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
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
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

public class SolverPMedianHybrid extends GRBCallback {

	public Instance inst;
	public static Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
	public GRBVar[] x[], c[], w[];

	private int edgeGroup[];
	private boolean visitedEdges[];

	private static FileWriter logfile;
	private static int count;

	public SolverPMedianHybrid(String filename) {
		this.inst = new Instance();
		this.g = this.inst.net.getG();
	}

	public static void main(String[] args) {

		SolverPMedianHybrid gurobi = null;
		String instanciaNome = null;

		try {
			try {
				try {

					Reader fileInst = new BufferedReader(new FileReader("instancias.txt"));
					StreamTokenizer stok = new StreamTokenizer(fileInst);

					stok.nextToken();
					while (stok.sval != null) {

						instanciaNome = stok.sval;

						gurobi = new SolverPMedianHybrid("instancias/" + instanciaNome);

						System.out.println(gurobi.getInst().getParameters().getInstanceName());

						env = new GRBEnv("mip1.log");
						model = new GRBModel(env);

						// Open log file for callbacks
						logfile = new FileWriter("callback.log");

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
								System.out.println(vnames[j] + "= " + x[j]);
							}
						}

						if (!gurobi.checkSol(model)) {// só para ter certeza!
							System.out.println("Solução infatível");
						}
//						System.out.println("maxSwitches " + gurobi.inst.parameters.getNumSwitches());

						logfile.close();

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
	
	// variaveis tipo C -- c_kp = 1 indica que o arco k pertence a parte p
	private void defineVarC(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo C
		c = new GRBVar[g.getEdges().size()][inst.parameters.getNumFI() + 1];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsX = 0.0f;
				double lbX = 0.0;
				double ubX = 1.0;
				for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
					c[edge.id][k] = model.addVar(lbX, ubX, 0.0f, tipoBinary, "c[" + edge.id + "," + k + "]");
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

	// variaveis tipo W -- w^k_i = 1 indica que na fronteira do no i existe mais
	// que uma aresta do mesmo grupo
	private void defineVarW(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo Y
		w = new GRBVar[g.getEdges().size()][inst.parameters.getNumFI() + 1];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				V node = edge.node1;
				// TEM QUE CHECAR SE O VALOR ESTA CERTO
				int degree = g.getIncidentEdges(node).size();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub;
				//if (degree >= 4) {
					ub = 1.0;
				//} else {
				//	ub = 0.0;
				//}
				for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
					w[edge.id][k] = model.addVar(lb, ub, 0.0f, tipoBinary, "w[" + edge.id + "][" + k + "]");
				}
				
				
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
				model.addConstr(constraint, GRB.EQUAL, 1, "c0_" + edge.id);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (1): sum c_ij = 1 \in k
	// Alocacao unica de centros
	private void addConstraint1(GRBModel model) {
		try {
			for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
				GRBLinExpr constraint = new GRBLinExpr();
				Iterator<E> iterEdges = g.getEdges().iterator();
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					constraint.addTerm(1, c[edge.id][k]);
				}
				model.addConstr(constraint, GRB.EQUAL, 1, "c1_" + k);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (2): x_ek >= c_ek   forall e,k
	// Atribuicao aresta grupo-centro
	private void addConstraint2(GRBModel model) {
		try {
			Iterator<E> iterEdgesE = g.getEdges().iterator();
			while (iterEdgesE.hasNext()) {
				E edgeE = iterEdgesE.next();
				for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
					GRBLinExpr constraint = new GRBLinExpr();
					constraint.addTerm(1, x[edgeE.id][k]);
					constraint.addTerm(-1, c[edgeE.id][k]);
					model.addConstr(constraint, GRB.GREATER_EQUAL, 0, "c2_" + edgeE.id + "," + k);
				}
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

	// RESTRICAO (2): x^k_a + x^k_c <= 1 + x^k_b k \in K, {a, c \in A}, b \in P(a,c)
	// Conectividade de cada grupo. Se existir uma aresta "a" no grupo "k", então toda
	// aresta "b" no caminho de "a" ate o centro de "k" deve fazer parte do mesmo grupo.
	private void addConstraint3(GRBModel model) {
		try {
			Iterator<E> iterEdgesE = g.getEdges().iterator();
			while (iterEdgesE.hasNext()) {
				E edgeE = iterEdgesE.next();
				for (E edgeA : g.getEdges()) {
					if (edgeA != edgeE && edgeA.node1 != edgeE.node1 && edgeA.node1 != edgeE.node2
							&& edgeA.node2 != edgeE.node1 && edgeA.node2 != edgeE.node2) {
						V orig, dest;
						if (g.isSource(edgeE.node1, edgeE)) // se i=node1 e j=node2 nao precisa
							orig = edgeE.node1;
						else
							orig = edgeE.node2;
						if (g.isSource(edgeA.node1, edgeA))
							dest = edgeA.node1;
						else
							dest = edgeA.node2;
						Iterator<E> iterEdgesB = getEdgesInPath(orig, dest).iterator();
						while (iterEdgesB.hasNext()) {
							E edgeB = iterEdgesB.next();
							if (edgeB == edgeE || edgeB == edgeA)
								continue;
							for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
								GRBLinExpr constraint = new GRBLinExpr();
								constraint.addTerm(1, c[edgeE.id][k]);
								constraint.addTerm(1, x[edgeA.id][k]);
								constraint.addTerm(-1, x[edgeB.id][k]);
								model.addConstr(constraint, GRB.LESS_EQUAL, 1,
										"c3_" + edgeE.id + "," + edgeB.id + "," + edgeA.id + "," + k);
							}
						}
					}
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	
	// (|\delta(i)|-1)*w^k_i >= \sum_{j \in \delta(i)}{x^k_{ij}} - 1
	// identifica o no que possui pelo menos duas arestas com o mesmo grupo 
	private void addConstraint4(GRBModel model) {
		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); 
				for (int k = 0; k <= this.inst.parameters.getNumFI(); k++) {
					int degree = g.getIncidentEdges(edge.node1).size();
					constraint.addTerm(degree-1, w[edge.id][k]);
					Iterator<E> iterIncEdges = g.getIncidentEdges(edge.node1).iterator();
					while (iterIncEdges.hasNext()) {
						E incEdge = iterIncEdges.next(); 
						constraint.addTerm(-1, x[incEdge.id][k]);
					}
					model.addConstr(constraint, GRB.GREATER_EQUAL, -1,
							"c4_" + edge.id + "," + k);
					constraint.clear();
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// w^k1_i + w^k2_i <= 1
	// impede que dois pares de arcos de cores diferentes co-existam no mesmo vértice
	private void addConstraint5(GRBModel model) {
		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next(); 
				for (int k1 = 0; k1 <= this.inst.parameters.getNumFI(); k1++) {
					for (int k2 = k1+1; k2 <= this.inst.parameters.getNumFI(); k2++) {
						constraint.addTerm(1, w[edge.id][k1]);
						constraint.addTerm(1, w[edge.id][k2]);
						model.addConstr(constraint, GRB.LESS_EQUAL, 1,
								"c5_" + edge.id + "," + k1 + "," + k2);
						constraint.clear();
					}
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}



	
	private void setInternalSymmetryBreaking(GRBModel model) {
		try {
			Iterator<E> iterEdgesU = g.getEdges().iterator();
			while (iterEdgesU.hasNext()) {
				E edgeU = iterEdgesU.next();
				for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
					GRBLinExpr constraint = new GRBLinExpr();
					Iterator<E> iterEdgesV = g.getEdges().iterator();
					constraint.addTerm(edgeU.id, x[edgeU.id][k]);
					while (iterEdgesV.hasNext()) {
						E edgeV = iterEdgesV.next();
						constraint.addTerm(-1.0*edgeV.id, c[edgeV.id][k]);						
					}					
					model.addConstr(constraint, GRB.LESS_EQUAL, 0, "symmetry1_" + edgeU.id + "," + k);
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	private void setExternalSymmetryBreaking(GRBModel model) {
		try {
			for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
				Iterator<E> iterEdgesU = g.getEdges().iterator();
				GRBLinExpr constraint = new GRBLinExpr();
				while (iterEdgesU.hasNext()) {
					E edgeU = iterEdgesU.next();
					constraint.addTerm(edgeU.id, c[edgeU.id][k]);
				}									
				for (int k1 = k+1; k1 <= inst.getParameters().getNumFI(); k1++) {
					Iterator<E> iterEdgesV = g.getEdges().iterator();				
					while (iterEdgesV.hasNext()) {
						E edgeV = iterEdgesV.next();
						constraint.addTerm(-1.0*edgeV.id, c[edgeV.id][k1]);						
					}
					model.addConstr(constraint, GRB.LESS_EQUAL, -(k1-k), "symmetry2_" + k1 + "," + k);
				}
			}
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	private void setVIESymmetryBreaking(GRBModel model) {
		try {
			for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
				Iterator<E> iterEdgesU = g.getEdges().iterator();				
				while (iterEdgesU.hasNext()) {
					GRBLinExpr constraint = new GRBLinExpr();
					E edgeU = iterEdgesU.next();
					Iterator<E> iterEdgesV = g.getEdges().iterator();				
					while (iterEdgesV.hasNext()) {
						E edgeV = iterEdgesV.next();
						if (edgeV.id > edgeU.id)
							constraint.addTerm(1, x[edgeV.id][k]);						
					}
					int bigM = Math.min(g.getEdgeCount()-edgeU.id, g.getEdgeCount()-inst.getParameters().getNumFI()+2);
					constraint.addTerm(bigM, c[edgeU.id][k]);
					model.addConstr(constraint, GRB.LESS_EQUAL, bigM, "symmetry3_" + edgeU.id + "," + k);
				}	
			}
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	

	private void populateNewModel(GRBModel model) {

		try {

			GRBQuadExpr ofexpr = new GRBQuadExpr();

			// Create variables
			this.defineVarX(model, 0, ofexpr);
			this.defineVarC(model, 1, ofexpr);
			// this.defineVarZ(model, 2, ofexpr);
			this.defineVarW(model, 3, ofexpr);
			this.generateOF(model, ofexpr);

			model.set(GRB.IntParam.LazyConstraints, 1);
			model.setCallback(this);
			
			// Integrate new variables
			model.update();

			model.setObjective(ofexpr);

			// RESTRICAO (0)
			this.addConstraint0(model);

			// RESTRICAO (1)
			this.addConstraint1(model);

			// RESTRICAO (2)
			this.addConstraint2(model);

			// RESTRICAO (3)
			//this.addConstraint3(model);

			// RESTRICAO (4)
			//this.addConstraint4(model);
			
			// RESTRICAO (5)
			//this.addConstraint5(model);
							
			//eliminação de simetria interna
			this.setInternalSymmetryBreaking(model);
			
			//eliminação de simetria externa
			this.setExternalSymmetryBreaking(model);
			
			//eliminação de VIE simetria 
			this.setVIESymmetryBreaking(model);

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
				checkConnectivity(next, k);
			}
		}

	}

	
	/** TODO
	 * 
	 * Tem que adaptar a integerSeparation para o modelo híbrido.
	 * 
	 */
	private void integerSeparation() throws GRBException, IOException {

		// Found an integer feasible solution - does any connectivity constraint
		// violated?
		visitedEdges = new boolean[g.getEdgeCount()];
		edgeGroup = new int[g.getEdgeCount()];
		E firstInGroup[] = new E[inst.getParameters().getNumFI() + 1];

		double x_val[][] = getSolution(x);

		Iterator<E> iterEdges;

		iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
				if (x_val[edge.id][k] > 0.5) {
					edgeGroup[edge.id] = k;
					firstInGroup[k] = edge;
				}
			}
		}

		for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
			Arrays.fill(visitedEdges, false);

			E edgeA = firstInGroup[k];
			if (edgeA != null) {
				V root = edgeA.node1;

				// DFS no grupo k
				checkConnectivity(root, k);

				// alguma aresta do grupo k não foi visitada pela DFS?
				iterEdges = g.getEdges().iterator();
				E edgeC = null;
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					if (edgeGroup[edge.id] == k && !visitedEdges[edge.id]) {
						edgeC = edge;
						break; // encontrei
					}
				}

				// inserir restricao
				if (edgeC != null) {
					iterEdges = getEdgesInPath(edgeA.node1, edgeC.node1).iterator();
					while (iterEdges.hasNext()) {
						E edgeB = iterEdges.next();
						if (edgeGroup[edgeB.id] == k || edgeB == edgeA || edgeB == edgeC)
							continue;
						GRBLinExpr constraint = new GRBLinExpr();
						constraint.addTerm(1, x[edgeA.id][k]);
						constraint.addTerm(1, x[edgeC.id][k]);
						constraint.addTerm(-1, x[edgeB.id][k]);
						addLazy(constraint, GRB.LESS_EQUAL, 1);
						logfile.write(count++ + ": c1_lazy_" + edgeA.id + "," + edgeB.id + "," + edgeC.id + "," + k + "\n");
						//break;
					}
				}
			}
		}
	}

	private void integerSeparationWithFI_Violation() throws GRBException, IOException {
		// Found an integer feasible solution - does any connectivity constraint
		// violated?
		visitedEdges = new boolean[g.getEdgeCount()];
		edgeGroup = new int[g.getEdgeCount()];
		double x_val[][] = getSolution(x);
		double c_val[][] = getSolution(c);
		
		E firstInGroup[] = new E[inst.getParameters().getNumFI() + 1];
		for (int k = 0; k <= inst.parameters.getNumFI(); k++)
			for(E edge : g.getEdges())			 
				if (c_val[edge.id][k]>0.5)
					firstInGroup[k]= edge;

		Iterator<E> iterEdges;

		iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
				if (x_val[edge.id][k] > 0.5) {
					edgeGroup[edge.id] = k;
				}
			}
		}

		for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
			Arrays.fill(visitedEdges, false);

			E edgeE = firstInGroup[k];
			if (edgeE != null) {
				V root = edgeE.node1;

				// DFS no grupo k
				checkConnectivity(root, k);

				// alguma aresta do grupo k não foi visitada pela DFS?
				iterEdges = g.getEdges().iterator();
				E edgeA = null;
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					if (edgeGroup[edge.id] == k && !visitedEdges[edge.id]) {
						edgeA = edge;
						break; // encontrei
					}
				}

				// inserir restricao
				if (edgeA != null) {
					iterEdges = getEdgesInPath(edgeE.node1, edgeA.node1).iterator();
					while (iterEdges.hasNext()) {
						E edgeB = iterEdges.next();
						if (edgeGroup[edgeB.id] == k || edgeB == edgeE || edgeB == edgeA)
							continue;
						GRBLinExpr constraint = new GRBLinExpr();
						constraint.addTerm(1, c[edgeE.id][k]);
						constraint.addTerm(1, x[edgeA.id][k]);
						constraint.addTerm(-1, x[edgeB.id][k]);
						addLazy(constraint, GRB.LESS_EQUAL, 1);
						logfile.write(count++ + ": c3_lazy_" + edgeE.id + "," + edgeB.id + "," + edgeA.id + "," + k + "\n");
						//break;
					}
				}
			}
		}
		
		int countGroup[][] = new int[g.getVertexCount()][inst.getParameters().getNumFI() + 1];
		iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			V node = edge.node1;
			
			int degree = g.getIncidentEdges(node).size();
			
			
			if (degree >= 4) {
				Iterator<E> iterIncEdges = g.getIncidentEdges(node).iterator();
				while (iterIncEdges.hasNext()) {
					E incEdge = iterIncEdges.next(); 
					countGroup[node.id][edgeGroup[incEdge.id]]++;
				}
				
				int k1, k2;
				for (k1 = 0; k1 <= inst.parameters.getNumFI(); k1++) {
					if (countGroup[node.id][k1] >= 2) {
						break;
					}
				}
				for (k2 = k1+1; k2 <= inst.parameters.getNumFI(); k2++) {
					if (countGroup[node.id][k2] >= 2) {
						break;
					}
				}
				if (k2 > inst.parameters.getNumFI()) {
					continue;
				}
				
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(degree-1, w[edge.id][k1]);
				iterIncEdges = g.getIncidentEdges(node).iterator();
				while (iterIncEdges.hasNext()) {
					E incEdge = iterIncEdges.next(); 
					constraint.addTerm(-1, x[incEdge.id][k1]);
				}
				addLazy(constraint, GRB.GREATER_EQUAL, -1);
				logfile.write(count++ + "c4_lazy_" + edge.id + "," + k1 + "\n");
				
				constraint.clear();
				constraint.addTerm(degree-1, w[edge.id][k2]);
				iterIncEdges = g.getIncidentEdges(node).iterator();
				while (iterIncEdges.hasNext()) {
					E incEdge = iterIncEdges.next(); 
					constraint.addTerm(-1, x[incEdge.id][k2]);
				}
				addLazy(constraint, GRB.GREATER_EQUAL, -1);
				logfile.write(count++ + "c4_lazy_" + edge.id + "," + k2 + "\n");
				
				constraint.clear();
				constraint.addTerm(1, w[edge.id][k1]);
				constraint.addTerm(1, w[edge.id][k2]);
				addLazy(constraint, GRB.LESS_EQUAL, 1);
				logfile.write(count++ + "c5_lazy_" + edge.id + "," + k1 + "," + k2 + "\n");
			}

		}
		
	}
	
	
	void checkConnectivityFractional(V root, int k, double x_val[][], E maxEdgeInGroup[]) {

		Iterator<E> iterEdges = g.getIncidentEdges(root).iterator();

		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			if (edgeGroup[edge.id] == k && !visitedEdges[edge.id]) {
				V next = (edge.node1 != root) ? edge.node1 : edge.node2;
				visitedEdges[edge.id] = true;
				if (maxEdgeInGroup[k] != null && x_val[maxEdgeInGroup[k].id][k] < x_val[edge.id][k]) {
					maxEdgeInGroup[k] = edge;
				}
				checkConnectivityFractional(next, k, x_val, maxEdgeInGroup);
			}
		}
		
	}
	
	
	@Override
	protected void callback() {
		try {
			if (where == GRB.CB_MIPSOL) {
				 //System.out.println("**** New node integer sol****");
				//integerSeparation();
				integerSeparationWithFI_Violation();
			} else if (where == GRB.CB_MIPNODE) {				
				// MIP node callback
				// System.out.println("**** New node fractional sol****");
				if (getIntInfo(GRB.CB_MIPNODE_STATUS) == GRB.OPTIMAL) {					
					//fractionalSeparation();
					//exactFractionalSeparation();
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
			e.printStackTrace();
		} catch (IOException e) {
			System.out.println("Can't write in file!: " + e.getMessage());
			e.printStackTrace();
		}
	}

	private boolean checkSol(GRBModel model) {
		try {
			visitedEdges = new boolean[g.getEdgeCount()];
			edgeGroup = new int[g.getEdgeCount()];
			E firstInGroup[] = new E[inst.getParameters().getNumFI() + 1];

			double x_val[][] = new double[g.getEdgeCount()][inst.getParameters().getNumFI() + 1];
			GRBVar[] fvars = model.getVars();
			double[] x = model.get(GRB.DoubleAttr.X, fvars);
			String[] vnames = model.get(GRB.StringAttr.VarName, fvars);			
			String[] xnames = Arrays.asList(vnames).stream().filter(name->name.contains("x")).toArray(String[]::new);
			
			for (int i = 0; i < xnames.length; i += inst.getParameters().getNumFI() + 1) {
				int idx = Integer.valueOf(xnames[i].substring(2, xnames[i].indexOf(",")));				
				for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
					x_val[idx][k] = x[i + k];
				}
			}

			Iterator<E> iterEdges;

			iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
					if (x_val[edge.id][k] > 0.5) {
						edgeGroup[edge.id] = k;
						firstInGroup[k] = edge;
					}
				}
			}

			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
				Arrays.fill(visitedEdges, false);

				E edgeA = firstInGroup[k];

				if (edgeA != null) {
					V root = edgeA.node1;
					// DFS no grupo k
					checkConnectivity(root, k);
					// alguma aresta do grupo k não foi visitada pela DFS?
					iterEdges = g.getEdges().iterator();
					E edgeC = null;
					while (iterEdges.hasNext()) {
						edgeC = iterEdges.next();
						if (edgeGroup[edgeC.id] == k && !visitedEdges[edgeC.id]) {
							return false; // encontrei
						}
					}
				}
			}
			return true;
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
			e.printStackTrace();
		}
		return false;
	}

}
