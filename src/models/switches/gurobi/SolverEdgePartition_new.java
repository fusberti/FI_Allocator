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
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Locale;

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

class OrdenaPorId implements Comparator<E> {
	public int compare(E edge1, E edge2) {
		return edge1.id - edge2.id;
	}
}

public class SolverEdgePartition_new extends GRBCallback {

	public Instance inst;
	public static Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
	public GRBVar[] x[], Sum;

	private int edgeGroup[];
	private boolean visitedEdges[];
	private boolean fiLocations[];

	public SolverEdgePartition_new(String[] args) {
		this.inst = new Instance(args);
		this.g = this.inst.net.getG();
	}

	public static void main(String[] args) {

		SolverEdgePartition_new gurobi = null;
		String instanciaNome = null;

		try {
			try {
				try {
					
					System.out.println("Edge Partition Model");

					Reader fileInst = new BufferedReader(new FileReader("instancias.txt"));
					StreamTokenizer stok = new StreamTokenizer(fileInst);

					// Writer outFileInst = new BufferedWriter(new
					// FileWriter("lowerbounds/lowerbounds.txt"));

					stok.nextToken();
					while (stok.sval != null) {

						instanciaNome = stok.sval;

						gurobi = new SolverEdgePartition_new(args);

						System.out.println(gurobi.getInst().getParameters().getInstanceName());

						env = new GRBEnv("logs/"+gurobi.getInst().getParameters().getInstanceName()+"."+gurobi.getInst().getParameters().getNumFI()+".log");
						model = new GRBModel(env);

						// Open log file for callbacks
						//logfile = new FileWriter("callback.log");

						// Configura os parametros do solver Gurobi
						new GurobiParameters(model);
						gurobi.populateNewModel(model);

						// Write model to file
						model.write("FI_Allocation.lp");

						System.out.println("Otimizando....");
						model.optimize();

						System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
						System.out.println("Obj (hrs): " + model.get(GRB.DoubleAttr.ObjVal)*gurobi.inst.getParameters().getFailureRate()*1/gurobi.inst.getParameters().getCrewVelocity());

//						GRBVar[] fvars = model.getVars();
//						double[] x = model.get(GRB.DoubleAttr.X, fvars);
//						String[] vnames = model.get(GRB.StringAttr.VarName, fvars);
//
//						for (int j = 0; j < fvars.length; j++) {
//							if (x[j] != 0.0) {
//								System.out.println(vnames[j] + "= " + x[j]);
//							}
//						}

						if (!gurobi.checkSol(model)) {// só para ter certeza!
							System.out.println("Solução infatível");
						}
						gurobi.setFILocations(model);
//						System.out.println("maxSwitches " + gurobi.inst.parameters.getNumSwitches());

						//logfile.close();
						
						gurobi.printResults(model);

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
			ofexpr.addTerm(1, Sum[k], Sum[k]);
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
	
	// variaveis tipo Sum_k -- Establishes the sum of distances for each part k 
	private void defineVarSum(GRBModel model, int indVar, GRBQuadExpr ofexpr) {

		// variaveis tipo X
		Sum = new GRBVar[inst.parameters.getNumFI() + 1];

		try {

				double lbX = 0.0;
				double ubX = Double.MAX_VALUE;
				for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
					Sum[k] = model.addVar(lbX, ubX, 0.0f, tipoFloat, "Sum[" + k + "]");
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
	
	// RESTRICAO (1): Sum_k = sum x^k_ij k
	// Particao de arestas.
	private void addConstraint1(GRBModel model) {
		try {	
			
			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
				GRBLinExpr constraint = new GRBLinExpr();
				Iterator<E> iterEdges = g.getEdges().iterator();
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					constraint.addTerm(-edge.dist, x[edge.id][k]);
				}
				constraint.addTerm(1, Sum[k]);
				model.addConstr(constraint, GRB.EQUAL, 0, "c1_" + k);
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


	private void setSymmetryBreaking(GRBModel model) {
		try {

			int k = 0;

			// Iterator<E> iterEdges = g.getEdges().iterator();
			List<E> shuffleEdges = new ArrayList<E>(g.getEdges());
			Collections.shuffle(shuffleEdges);
			Iterator<E> iterEdges = shuffleEdges.iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();

				GRBLinExpr constraint = new GRBLinExpr();
				for (int k2 = 0; k2 <= k; k2++) {
					constraint.addTerm(1.0, x[edge.id][k2]);
				}
				model.addConstr(constraint, GRB.EQUAL, 1, "symmetry" + k);
				k++;
				if (k == inst.getParameters().getNumFI())
					break;
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	private void setSymmetryBreaking2(GRBModel model) {
		try {

			for (int k = 1; k <= inst.getParameters().getNumFI(); k++) {

				GRBLinExpr constraint = new GRBLinExpr();

				Iterator<E> iterEdges = g.getEdges().iterator();
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					constraint.addTerm(edge.dist, x[edge.id][k - 1]);
					constraint.addTerm(-edge.dist, x[edge.id][k]);
				}
				model.addConstr(constraint, GRB.LESS_EQUAL, 0, "symmetry2_" + k);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	private void printInitialSolution() {
		for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
			Iterator<E> iterEdges = g.getEdges().iterator();
			System.out.print(k + ": ");
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				if (edge.iniSol_district == k && edge.iniSol_isCenter)
					System.out.print("[" + edge.id + "] ");
				else if (edge.iniSol_district == k)
					System.out.print(edge.id + " ");
			}
			System.out.println();
		}
	}

	// must call createValidSymmetryInitialSolution previously
	private void setInitialSolution(GRBModel model, String filename) {
		boolean initSol = false;
		try {
			// LEITURA DA SOLUCAO INICIAL
			BufferedReader reader = new BufferedReader(
					new FileReader(filename + ".sol." + inst.getParameters().getNumFI()));
			String line;
			int k = 0;
			while ((line = reader.readLine()) != null) {
				String[] edgeIds = line.split(" ");
				for (String strId : edgeIds) {
					int id = Integer.parseInt(strId);
					this.inst.net.getMapEdgeIndex().get(id).iniSol_district = k;
				}
				k++;
			}
			reader.close();
			initSol = true;
		} catch (Exception e) {
			System.err.println("Não há arquivo com solução inicial");
			e.printStackTrace();
		}

		if (!initSol)
			return;

		System.out.println("Setting initial solution...");

		// System.out.println();
		printInitialSolution();

		try {

			Iterator<E> iterEdges;

			iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
					x[edge.id][k].set(GRB.DoubleAttr.Start, 0.0);
				}
			}

			iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				int k = edge.iniSol_district;
				x[edge.id][k].set(GRB.DoubleAttr.Start, 1.0);
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
			this.defineVarSum(model, 1, ofexpr);
			this.generateOF(model, ofexpr);

			model.set(GRB.IntParam.LazyConstraints, 1);
			model.setCallback(this);

			// Integrate new variables
			model.update();

			model.setObjective(ofexpr);

			// RESTRICAO (0): sum x_ij = 1 \in k
			this.addConstraint0(model);
			
			this.addConstraint1(model);

			//this.setInitialSolution(model, "inisols/" + this.getInst().getParameters().getInstanceName().substring(this.getInst().getParameters().getInstanceName().lastIndexOf("/")+1));

			model.update();

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	private void printResults(GRBModel model) throws IOException, GRBException {
		FileWriter solFile = new FileWriter("resultados.csv",true);
		double distToTime = inst.getParameters().getFailureRate() * 1 / inst.getParameters().getCrewVelocity();
		double totalDistance = 0;
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
		E edge = iterEdges.next();
		totalDistance += edge.dist;
		}
		String str= inst.getParameters().getInstanceName().substring(0, inst.getParameters().getInstanceName().lastIndexOf('.')) + "\t"
		+ inst.getParameters().getNumFI() + "\t" 
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjVal)) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjBound)) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjVal)*distToTime) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjBound)*distToTime) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjVal)*distToTime/(totalDistance*inst.getParameters().getFailureRate())) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.ObjBound)*distToTime/(totalDistance*inst.getParameters().getFailureRate())) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.MIPGap)) + "\t"
		+ String.format(Locale.US, "%.2f",model.get(GRB.DoubleAttr.Runtime));

		solFile.write(str + "\n");
		solFile.close();
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

	private void integerSeparationWithFI_Violation() throws GRBException, IOException {

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
						//logfile.write(
							//	count++ + ": c1_lazy_" + edgeA.id + "," + edgeB.id + "," + edgeC.id + "," + k + "\n");
						// break;
					}
				}
			}
		}
		
		
		// new FI Violation constraints
		
		Iterator<V> iterNodes = g.getVertices().iterator();
		
		while (iterNodes.hasNext()) {
			V node = iterNodes.next();
			if (g.getSuccessorCount(node) > 2) {
				
				int countGroup[] = new int[inst.getParameters().getNumFI() + 1];
				int numGroups = 0;
				
				//Iterator<E> outEdges = g.getOutEdges(node).iterator();
				Iterator<E> outEdges = g.getIncidentEdges(node).iterator();
				while (outEdges.hasNext()) {
					E outEdge = outEdges.next();
					countGroup[edgeGroup[outEdge.id]]++;
					if (countGroup[edgeGroup[outEdge.id]] == 2) {
						numGroups++;
					}
				}
				
				if (numGroups > 1) {

					int group1 = -1, terms1 = 0;
					int group2 = -1, terms2 = 0;
					GRBLinExpr constraint = new GRBLinExpr();

					outEdges = g.getIncidentEdges(node).iterator();
					while (outEdges.hasNext()) {
						E outEdge = outEdges.next();
						if (countGroup[edgeGroup[outEdge.id]] > 1) {
							if (edgeGroup[outEdge.id] == group1 || group1 < 0) {
								group1 = edgeGroup[outEdge.id];
								if (terms1 < 2) {
									constraint.addTerm(1, x[outEdge.id][group1]);
									terms1++;
								}
								
							} else if (edgeGroup[outEdge.id] == group2 || group2 < 0) {
								group2 = edgeGroup[outEdge.id];
								if (terms2 < 2)	{
									constraint.addTerm(1, x[outEdge.id][group2]);
									terms2++;
								}
							}
						}
					}
					if ((terms1 == 2)&&(terms2 == 2)) {
						addLazy(constraint, GRB.LESS_EQUAL, 3);
					}
				}
			}
		}

	}

	void checkConnectivityFractional(V root, int k, double x_val[][], E maxEdgeInGroup[]) {

		Iterator<E> iterEdges = g.getIncidentEdges(root).iterator();

		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			if (!visitedEdges[edge.id] && x_val[edge.id][k] > 0.9) {
				V next = (edge.node1 != root) ? edge.node1 : edge.node2;
				visitedEdges[edge.id] = true;
//				if (maxEdgeInGroup[k] != null && x_val[maxEdgeInGroup[k].id][k] < x_val[edge.id][k]) {
//					maxEdgeInGroup[k] = edge;
//				}
				checkConnectivityFractional(next, k, x_val, maxEdgeInGroup);
			}
		}

	}

	private void exactFractionalSeparation() throws GRBException, IOException {
		// Found a fractional solution - does any connectivity constraint violated?
		visitedEdges = new boolean[g.getEdgeCount()];
		edgeGroup = new int[g.getEdgeCount()];

		double[][] x_val = getNodeRel(x);

		Iterator<E> iterEdges;

//		iterEdges = g.getEdges().iterator();
//		while (iterEdges.hasNext()) {
//			E edge = iterEdges.next();
//			double maxVal = -1.0f;
//			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
//				if (x_val[edge.id][k] > maxVal) {
//					edgeGroup[edge.id] = k;
//					maxVal = x_val[edge.id][k];
//				}
//			}
//		}

		Iterator<E> iterEdgesA = g.getEdges().iterator();
		while (iterEdgesA.hasNext()) {
			E edgeA = iterEdgesA.next();
			visitedEdges[edgeA.id] = true;
			int k = edgeGroup[edgeA.id];
			Iterator<E> iterEdgesC = g.getEdges().iterator();
			while (iterEdgesC.hasNext()) {
				E edgeC = iterEdgesC.next();
				if (edgeGroup[edgeC.id] == k && !visitedEdges[edgeC.id]) {
					//if (x_val[edgeA.id][k] + x_val[edgeC.id][k] <= 1.0f) {
					if (x_val[edgeA.id][k] + x_val[edgeC.id][k] >= 1.5f) {
						Iterator<E> iterEdgesB = getEdgesInPath(edgeA.node1, edgeC.node1).iterator();
						while (iterEdgesB.hasNext()) {
							E edgeB = iterEdgesB.next();
							if (edgeB == edgeA || edgeB == edgeC
									|| (x_val[edgeA.id][k] + x_val[edgeC.id][k] <= 1 + x_val[edgeB.id][k]))
								continue;
							//System.out.println("inserted");
							GRBLinExpr constraint = new GRBLinExpr();
							constraint.addTerm(1, x[edgeA.id][k]);
							constraint.addTerm(1, x[edgeC.id][k]);
							constraint.addTerm(-1, x[edgeB.id][k]);
							addLazy(constraint, GRB.LESS_EQUAL, 1); // ,
							// "c1_lazy_"+edgeA.id+","+edgeB.id+","+edgeC.id+","+k);
							//logfile.write(count++ + ": c1f_lazy_" + edgeA.id + "," + edgeB.id + "," + edgeC.id + "," + k
								//	+ "\n");
						}
					}
				}
			}
		}

	}

	private void fractionalSeparation() throws GRBException, IOException {
		// Found a fractional solution - does any connectivity constraint violated?
		visitedEdges = new boolean[g.getEdgeCount()];
		// E firstInGroup[] = new E[inst.getParameters().getNumFI() + 1];
		E maxEdgeInGroup[] = new E[inst.getParameters().getNumFI() + 1];
		E maxEdgeInGroupOutDFS[] = new E[inst.getParameters().getNumFI() + 1];

		double[][] x_val = getNodeRel(x);

		Iterator<E> iterEdges;

//		double maxVal = -1.0;
//		iterEdges = g.getEdges().iterator();
//		while (iterEdges.hasNext()) {
//			E edge = iterEdges.next();
//			for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
//				if (x_val[edge.id][k] > maxVal) {
//					edgeGroup[edge.id] = k;
//					// firstInGroup[k] = edge;
//					maxEdgeInGroup[k] = edge;
//					maxVal = x_val[edge.id][k];
//				}
//			}
//		}
		
		for (int k = 0; k <= inst.parameters.getNumFI(); k++) {
			double maxVal = -1.0;
			iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				if (x_val[edge.id][k] > maxVal) {
					maxEdgeInGroup[k] = edge;
					maxVal = x_val[edge.id][k];
				}
			}
		}

		for (int k = 0; k <= inst.parameters.getNumFI(); k++) {

			Arrays.fill(visitedEdges, false);

			E edgeA = maxEdgeInGroup[k];
			if (edgeA != null) {
				V root = edgeA.node1;

				// DFS no grupo k
				checkConnectivityFractional(root, k, x_val, maxEdgeInGroup);

				// escolher a aresta do grupo k não foi visitada pela DFS com maior valor de
				// x_val
				iterEdges = g.getEdges().iterator();
				E edgeC = null;
				double maxVal = -1.0;
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					if (!visitedEdges[edge.id]) {
						if (x_val[edge.id][k] > maxVal) {
							maxEdgeInGroupOutDFS[k] = edge;
							maxVal = x_val[edge.id][k];
						}
					}
				}
				edgeC = maxEdgeInGroupOutDFS[k];
				// inserir restricao
				if (edgeC != null) {
					iterEdges = getEdgesInPath(edgeA.node1, edgeC.node1).iterator();
					while (iterEdges.hasNext()) {
						E edgeB = iterEdges.next();
						if (edgeB == edgeA || edgeB == edgeC
								|| (x_val[edgeA.id][k] + x_val[edgeC.id][k] <= 1 + x_val[edgeB.id][k]))
							continue;
						//System.out.println("cut inserted "+(x_val[edgeA.id][k] + x_val[edgeC.id][k] - 1 - x_val[edgeB.id][k]));
						GRBLinExpr constraint = new GRBLinExpr();
						constraint.addTerm(1, x[edgeA.id][k]);
						constraint.addTerm(1, x[edgeC.id][k]);
						constraint.addTerm(-1, x[edgeB.id][k]);
						addLazy(constraint, GRB.LESS_EQUAL, 1); // ,
																// "c1_lazy_"+edgeA.id+","+edgeB.id+","+edgeC.id+","+k);
						//logfile.write(
							//	count++ + ": c1f_lazy_" + edgeA.id + "," + edgeB.id + "," + edgeC.id + "," + k + "\n");
						break;
					}
				}
			}
		}
	}

	@Override
	protected void callback() {
		try {
			if (where == GRB.CB_MIPSOL) {
				// System.out.println("**** New node integer sol****");
				// integerSeparation();
				integerSeparationWithFI_Violation();
			} else if (where == GRB.CB_MIPNODE) {
				// MIP node callback
				//System.out.println("**** New node fractional sol****");
				if (getIntInfo(GRB.CB_MIPNODE_STATUS) == GRB.OPTIMAL) {
					fractionalSeparation();
					// exactFractionalSeparation();
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
			String[] xnames = Arrays.asList(vnames).stream().filter(name -> name.contains("x")).toArray(String[]::new);

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
					// alguma aresta do grupo k n�o foi visitada pela DFS?
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

	void setFILocations(GRBModel model) throws IOException {
		try {
			edgeGroup = new int[g.getEdgeCount()];
			fiLocations = new boolean[g.getEdgeCount() * 2];

			double x_val[][] = new double[g.getEdgeCount()][inst.getParameters().getNumFI() + 1];
			GRBVar[] fvars = model.getVars();
			double[] x = model.get(GRB.DoubleAttr.X, fvars);
			String[] vnames = model.get(GRB.StringAttr.VarName, fvars);
			String[] xnames = Arrays.asList(vnames).stream().filter(name -> name.contains("x")).toArray(String[]::new);

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
					if (x_val[edge.id][k] > 0.5)
						edgeGroup[edge.id] = k;
				}
			}

			// print solution
			for (int k = 0; k <= inst.getParameters().getNumFI(); k++) {
				iterEdges = g.getEdges().iterator();
				System.out.print(k + ": [");
				while (iterEdges.hasNext()) {
					E edge = iterEdges.next();
					if (edgeGroup[edge.id] == k)
						System.out.print(" " + edge.id);// + " " + edge);
				}
				System.out.println("]");
			}

			g.getVertices().stream().filter(v -> g.getIncidentEdges(v).size() >= 2).forEach(v -> {
				// System.out.println("v: " + v);
				if (v.label != -1) {
					E edge = g.getInEdges(v).iterator().next();
					// System.out.println("Edge: " + edge);
					Iterator<E> itIncEdges = g.getIncidentEdges(v).iterator();
					double arcsInGroup[] = new double[inst.getParameters().getNumFI() + 1];
					boolean changedColor = false;
					while (itIncEdges.hasNext()) {
						E incEdge = itIncEdges.next();
						if (edge != incEdge)
							arcsInGroup[edgeGroup[incEdge.id]]++;//
						if (edge != incEdge && edgeGroup[edge.id] != edgeGroup[incEdge.id])
							changedColor = true;
					}
					if (changedColor) {
						// System.out.println("Mudou de cor!");
						Arrays.sort(arcsInGroup);
						// System.out.println(Arrays.toString(arcsInGroup));
						// System.out.println(Arrays.binarySearch(arcsInGroup,1.5));
						if (-1 * Arrays.binarySearch(arcsInGroup, 1.5) <= arcsInGroup.length) { // rule 1
							// System.out.println("Regra 1!");
							fiLocations[2 * edge.id + 1] = true; // fi located at the end of the predecessor arc
						}
						itIncEdges = g.getIncidentEdges(v).iterator();
						while (itIncEdges.hasNext()) {
							boolean siblings = false;
							E incEdge = itIncEdges.next();
							if (incEdge == edge)
								continue;
							Iterator<E> iterEdges2 = g.getIncidentEdges(v).iterator();
							while (iterEdges2.hasNext()) {
								E incEdge2 = iterEdges2.next();
								if (incEdge2 == edge || incEdge == incEdge2)
									continue;
								if (edgeGroup[incEdge.id] == edgeGroup[incEdge2.id]) {
									siblings = true;
									break;
								}
							}
							if (edgeGroup[incEdge.id] != edgeGroup[edge.id] && !siblings) { // rule 2
								fiLocations[2 * incEdge.id] = true;
								// System.out.println("Regra 2!");
							}
						}
					}
				} 
				else {
					Iterator<E> itIncEdges = g.getIncidentEdges(v).iterator();
					while (itIncEdges.hasNext()) {
						boolean siblings = false;
						E incEdge = itIncEdges.next();						
						Iterator<E> iterEdges2 = g.getIncidentEdges(v).iterator();
						while (iterEdges2.hasNext()) {
							E incEdge2 = iterEdges2.next();
							if (incEdge == incEdge2)
								continue;
							if (edgeGroup[incEdge.id] == edgeGroup[incEdge2.id]) {
								siblings = true;
								break;
							}
						}
						if (!siblings) { // rule 2
							fiLocations[2 * incEdge.id] = true;
							// System.out.println("Regra 2!");
						}
					}
				}
			});
			writePlotFiles(edgeGroup);
			// System.out.println("FI: " + Arrays.toString(fiLocations));
			System.out.print("FI: ");
			for (int i = 0; i < fiLocations.length; i++)
				if (fiLocations[i])
					System.out.printf("%d(%s) ", i / 2, i % 2 == 0 ? "i" : "f");
			System.out.println();

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
			e.printStackTrace();
		}
	}
	
	public void writePlotFiles(int[] edges) throws IOException{
		String instance = getInst().getParameters().getInstanceName();
		try (FileWriter writer = new FileWriter(instance.substring(instance.lastIndexOf("/")+1, instance.lastIndexOf(".")) + "_edges.dat")){
			g.getEdges().stream().filter(e -> e.id != 0).forEach(e -> {
				double x1 = g.getSource(e).coordX;
				double y1 = g.getSource(e).coordY;
				double x2 = g.getDest(e).coordX;
				double y2 = g.getDest(e).coordY;
				try {					
					writer.write(x1 + "\t" + y1 + "\t" + edges[e.id] + "\n");
					writer.write(x2 + "\t" + y2 + "\t" + edges[e.id] + "\n\n");
					
				} catch (IOException e1) {
					e1.printStackTrace();
				}
			});
		}
//		try (FileWriter writer = new FileWriter(instance.substring(instance.lastIndexOf("/")+1, instance.lastIndexOf(".")) + "_vertices.dat")){
//			g.getVertices().stream().filter(v -> v.label != -1).forEach(v -> {
//				try {					
//					writer.write(v.coordX + "\t" + v.coordY + "\n");					
//				} catch (IOException e1) {
//					e1.printStackTrace();
//				}
//			});
//		}		
	}
}
