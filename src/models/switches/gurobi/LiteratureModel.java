/*
 * Modelo de alocação de sinalizadores para redes de distribuição de energia
 * 
 * Ideias para futuro: 
 * - avaliar o impacto economico da alocação de sinalizadores, ou seja, a quantidade de sinalizadores
 * passa a ser uma variável do modelo. 
 * 
 */

// ISSO EH UM TESTE PARA O GIT

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

public class LiteratureModel {

	public Instance inst;
	public static Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
	public GRBVar[] yt, yh, b[], l[], t;
	public double totalDist;
	//int count = 1000;

	public LiteratureModel(String[] args) {
		this.inst = new Instance(args);
		this.g = this.inst.net.getG();
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			totalDist += edge.dist;
		}
	}

	public static void main(String[] args) {

		LiteratureModel gurobi = null;
		String instanciaNome = null;

		try {
			try {
				try {

					System.out.println("Literature Model");
					
					Reader fileInst = new BufferedReader(new FileReader("instancias.txt"));
					StreamTokenizer stok = new StreamTokenizer(fileInst);

					// Writer outFileInst = new BufferedWriter(new
					// FileWriter("lowerbounds/lowerbounds.txt"));

					stok.nextToken();
					while (stok.sval != null) {

						instanciaNome = stok.sval;

						gurobi = new LiteratureModel(args);

						System.out.println(gurobi.inst.getParameters().getInstanceName());

						env = new GRBEnv("logs/"+gurobi.inst.getParameters().getInstanceName()+"."+gurobi.inst.getParameters().getNumFI()+".log");
						model = new GRBModel(env);

						// Open log file for callbacks
						//logfile = new FileWriter("callback.log");

						// Configura os parametros do solver Gurobi
						new GurobiParameters(model);
						gurobi.populateNewModel(model);

						// Write model to file
						model.write("FI_Allocation_Flows.lp");

						System.out.println("Otimizando....");
						model.optimize();

						System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
						System.out.println("Obj (hrs): " + model.get(GRB.DoubleAttr.ObjVal)*gurobi.inst.getParameters().getFailureRate()*1/gurobi.inst.getParameters().getCrewVelocity());

						GRBVar[] fvars = model.getVars();
						double[] y1 = model.get(GRB.DoubleAttr.X, gurobi.yt);
						double[] y2 = model.get(GRB.DoubleAttr.X, gurobi.yh);

						Iterator<E> iterEdges = g.getEdges().iterator();
						while (iterEdges.hasNext()) {
							E edge = iterEdges.next();
							if (y1[edge.id] > 0.5) {
								System.out.println("yt["+edge.id+"] = " + y1[edge.id]);
							}
							if (y2[edge.id] > 0.5) {
								System.out.println("yh["+edge.id+"] = " + y2[edge.id]);
							}
						}
						
						gurobi.printResults(model);


						//logfile.close();

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
	
	double[] yt_cut, yh_cut;
	GRBLinExpr cut;
	
	private double probeMaxCut(V root, double sumY, double budget)  {
		
		double sumDist = 0;
		
		Iterator<E> outRootEdges = g.getOutEdges(root).iterator();
		while (outRootEdges.hasNext()) {
			E edge = outRootEdges.next();
			if (sumY + yh_cut[edge.id] < budget) {
				double sumYlocal = sumY + yh_cut[edge.id];
				double downDist = 0; 
				if (sumYlocal + yt_cut[edge.id] < budget) {
					sumYlocal += yt_cut[edge.id];
					downDist = probeMaxCut(g.getDest(edge), sumYlocal, budget);
					cut.addTerm(downDist, yt[edge.id]);
				}
				cut.addTerm(downDist+edge.dist, yh[edge.id]);

				sumDist += edge.dist + downDist;
			}
		}
		
		return sumDist;
		
	}

	
	
	// generate the quadractic objective function
	private void generateOF(GRBModel model, GRBQuadExpr ofexpr) {

		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			//ofexpr.addTerm(edge.dist*inst.parameters.ro, t[edge.id]);
			ofexpr.addTerm(edge.dist, t[edge.id]);
		}
		
	}

	// variaveis tipo Y -- y_ij = 1 indica que na fronteira do no i existe um
	// sinalizador
	private void defineVarY(GRBModel model) {

		// variaveis tipo Y
		yt = new GRBVar[g.getEdgeCount()];
		yh = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = 1.0;
				yt[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "yt[" + edge + "]");
				yh[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "yh[" + edge + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	// variaveis tipo B -- B_e1e2 , quantidade de FIs entre as arestas e1 e e2
	private void defineVarB(GRBModel model) {

		// variaveis tipo Y
		b = new GRBVar[g.getEdgeCount()][g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					double lb = 0.0;
					double ub = g.getEdgeCount();
					b[edge.id][edge2.id] = model.addVar(lb, ub, 0.0f, tipoInt, "b[" + edge + "," + edge2 + "]");
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	
	// variaveis tipo L -- L_e1e2 , igual a 1 sse nao existe um FI entre as arestas e1 e e2
	private void defineVarL(GRBModel model) {

		// variaveis tipo Y
		l = new GRBVar[g.getEdgeCount()][g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					double lb = 0.0;
					double ub = 1.0;
					l[edge.id][edge2.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "l[" + edge.id + "," + edge2.id + "]");
				}
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	// variaveis tipo T -- t_e , tempo de localizacao para uma falha ocorrida na aresta e
	private void defineVarT(GRBModel model) {

		// variaveis tipo Y
		t = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = Double.MAX_VALUE;
				t[edge.id] = model.addVar(lb, ub, 0.0f, tipoFloat, "t[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	// RESTRICAO (0): sum (yt + yh) = K
	// Alocacao de sinalizadores
	private void addConstraint0(GRBModel model) {
		try {
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				constraint.addTerm(1, yt[edge.id]);
				constraint.addTerm(1, yh[edge.id]);
			}
			model.addConstr(constraint, GRB.EQUAL, inst.getParameters().getNumFI(), "FI");

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (1): b_e1e2 = sum y_e   \forall e in Path(e1,e2)
	private void addConstraint1(GRBModel model) {
		try {
			
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					
					GRBLinExpr constraint = new GRBLinExpr();
					constraint.addTerm(1, b[edge.id][edge2.id]);
					
					if (edge != edge2) {
						//Collection<E> path = getEdgesInPath(edge.node1,edge2.node1);
						ArrayList<E> path = (ArrayList<E>) getEdgesInPath(edge,edge2);
						E lca = path.get(0);
						
						// Inserindo FIs no caminho entre as duas arestas
						//Iterator<E> iterPath = g.getOutEdges(g.getDest(edge)).iterator();
						Iterator<E> iterPath = path.iterator();
						while (iterPath.hasNext()) {
							E pathEdge = iterPath.next();
							if (pathEdge != edge && pathEdge != edge2 && pathEdge != lca) {
								constraint.addTerm(-1, yt[pathEdge.id]);
								constraint.addTerm(-1, yh[pathEdge.id]);
							}
						}
						
						// Tratando o caso de contorno para os FIs nas arestas extremas do caminho
						if (edge != lca) {
							// Caso em que o FI em tail yt deve fazer parte da restricao
							constraint.addTerm(-1, yt[edge.id]);
						} else {
							constraint.addTerm(-1, yh[edge.id]);
						}
						
						// Tratando o caso de contorno para os FIs nas arestas extremas do caminho
						if (edge2 != lca) {
							// Caso em que o FI em tail yt deve fazer parte da restricao
							constraint.addTerm(-1, yt[edge2.id]);
						} else {
							constraint.addTerm(-1, yh[edge2.id]);
						}
					}
					
					model.addConstr(constraint, GRB.EQUAL, 0, "const1[" + edge + "," + edge2 + "]");
				}
			}
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	// RESTRICAO (2): \epsilon - b_e1e2/M <= l_e1e2 <= 1 - b_e1e2/M 
	private void addConstraint2(GRBModel model) {
		try {
			
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				Iterator<E> iterEdges2 = g.getEdges().iterator();
				while (iterEdges2.hasNext()) {
					E edge2 = iterEdges2.next();
					
					GRBLinExpr constraint = new GRBLinExpr();
					constraint.addTerm(1/(double)g.getEdgeCount(), b[edge.id][edge2.id]);
					constraint.addTerm(1, l[edge.id][edge2.id]);
					model.addConstr(constraint, GRB.GREATER_EQUAL, 1e-5, "const2a[" + edge.id + "," + edge2.id + "]");
					
					constraint = new GRBLinExpr();
					constraint.addTerm(1/(double)g.getEdgeCount(), b[edge.id][edge2.id]);
					constraint.addTerm(1, l[edge.id][edge2.id]);
					model.addConstr(constraint, GRB.LESS_EQUAL, 1, "const2b[" + edge.id + "," + edge2.id + "]");

				}
			}
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	

	// RESTRICAO (3): t_e2 = sum_e1 v*d*l_e1e2 
	private void addConstraint3(GRBModel model) {
		try {
			
			Iterator<E> iterEdges2 = g.getEdges().iterator();
			while (iterEdges2.hasNext()) {

				E edge2 = iterEdges2.next();
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, t[edge2.id]);
				
				Iterator<E> iterEdges = g.getEdges().iterator();
				while (iterEdges.hasNext()) {

					E edge = iterEdges.next();
					//constraint.addTerm(inst.parameters.alpha*edge.dist, l[edge.id][edge2.id]);
					constraint.addTerm(-edge.dist, l[edge.id][edge2.id]);

				}
				
				model.addConstr(constraint, GRB.EQUAL, 0, "const3[" + edge2.id + "]");
			}
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}


	private void populateNewModel(GRBModel model) {

		try {

			GRBQuadExpr ofexpr = new GRBQuadExpr();

			// Create variables
			this.defineVarY(model);
			this.defineVarB(model);
			this.defineVarL(model);
			this.defineVarT(model);
			this.generateOF(model, ofexpr);

			//model.set(GRB.IntParam.LazyConstraints, 1);
			//model.setCallback(this);

			// Integrate new variables
			model.update();

			model.setObjective(ofexpr);

			this.addConstraint0(model);
			
			this.addConstraint1(model);

			this.addConstraint2(model);

			this.addConstraint3(model);

			model.update();

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


	public List<E> getEdgesInPath(E orig, E dest) {
		
		List<E> path = new ArrayList<E>();
		E lca = null;
		
		List<E> predecessorsOrig = new ArrayList<E>();
		E pred = orig;
		predecessorsOrig.add(pred);
		while (g.getInEdges(pred.node1).iterator().hasNext()) {
			pred = g.getInEdges(pred.node1).iterator().next();
			predecessorsOrig.add(pred);
		}
				
		List<E> predecessorsDest = new ArrayList<E>();
		pred = dest;
		predecessorsDest.add(pred);
		while (g.getInEdges(pred.node1).iterator().hasNext()) {
			pred = g.getInEdges(pred.node1).iterator().next();
			predecessorsDest.add(pred);
		}

		for (E predOrig : predecessorsOrig) {
			if (predecessorsDest.contains(predOrig)) {
				lca = predOrig;
				path.add(0,predOrig); // first edge is the least common ancestor
				break;
			}
			path.add(predOrig); 
		}
		
		if (lca == null) {
			path.add(0,null);
		}
		
		for (E predDest : predecessorsDest) {
			if (lca == predDest) break;
			path.add(predDest);
		}
		
		return path;
	}

	
}
