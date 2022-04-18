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

public class SolverFlows extends GRBCallback {

	public Instance inst;
	public static Graph<V, E> g;

	final char tipoInt = GRB.INTEGER;
	final char tipoFloat = GRB.CONTINUOUS;
	final char tipoBinary = GRB.BINARY;
	public static GRBEnv env;
	public static GRBModel model;
	public GRBVar[] yt, yh, dt, dh, ft, fh;
	public GRBVar dr;
	public double totalDist;
	//int count = 1000;

	public SolverFlows(String[] args) {
		this.inst = new Instance(args);
		this.g = this.inst.net.getG();
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			totalDist += edge.dist;
		}
	}

	public static void main(String[] args) {

		SolverFlows gurobi = null;
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

						gurobi = new SolverFlows(args);

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

	
	
	private void fractionalSeparation() throws GRBException, IOException {

		yt_cut = getNodeRel(yt);
		yh_cut = getNodeRel(yh);
		
		
		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();

			cut = new GRBLinExpr();
			double downDist = probeMaxCut(g.getDest(edge), 0, yt_cut[edge.id]);
			if (cut.size() > 0) {
				cut.addTerm(1, dt[edge.id]);
				//cut.addTerm(1, ft[edge.id]);
				cut.addTerm(-downDist, yt[edge.id]);
				addLazy(cut, GRB.GREATER_EQUAL, 0);
			}
			
			cut = new GRBLinExpr();
			downDist = probeMaxCut(g.getDest(edge), yt_cut[edge.id], yh_cut[edge.id]);
			if (cut.size() > 0) {
				cut.addTerm(1, dh[edge.id]);
				//cut.addTerm(1, fh[edge.id]);
				cut.addTerm(downDist, yt[edge.id]);
				cut.addTerm(-downDist-edge.dist, yh[edge.id]);
				addLazy(cut, GRB.GREATER_EQUAL, 0);
			}
			
			cut = new GRBLinExpr();
			downDist = probeMaxCut(g.getDest(edge), yt_cut[edge.id], 1);
			if (cut.size() > 0) {
				//cut.addTerm(1, dt[edge.id]);
				cut.addTerm(1, ft[edge.id]);
				cut.addTerm(downDist, yt[edge.id]);
				addLazy(cut, GRB.GREATER_EQUAL, downDist);
			}
			
			cut = new GRBLinExpr();
			downDist = probeMaxCut(g.getDest(edge), yt_cut[edge.id]+yh_cut[edge.id], 1);
			if (cut.size() > 0) {
				//cut.addTerm(1, dh[edge.id]);
				cut.addTerm(1, fh[edge.id]);
				cut.addTerm(downDist, yt[edge.id]);
				cut.addTerm(downDist+edge.dist, yh[edge.id]);
				addLazy(cut, GRB.GREATER_EQUAL, downDist+edge.dist);
			}

		}
		
	}
	
	
	@Override
	protected void callback() {
		try {
			if (where == GRB.CB_MIPNODE) {
				// MIP node callback
				//System.out.println("**** New node fractional sol****");
//				if (count < 100) {
//					count++;
				if (getIntInfo(GRB.CB_MIPNODE_STATUS) == GRB.OPTIMAL) {
					fractionalSeparation();
				}
//				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
			e.printStackTrace();
		} catch (IOException e) {
			System.out.println("Can't write in file!: " + e.getMessage());
			e.printStackTrace();
		}
	}

	// generate the quadractic objective function
	private void generateOF(GRBModel model, GRBQuadExpr ofexpr) {

		Iterator<E> iterEdges = g.getEdges().iterator();
		while (iterEdges.hasNext()) {
			E edge = iterEdges.next();
			ofexpr.addTerm(1, dt[edge.id], dt[edge.id]);
			ofexpr.addTerm(1, dh[edge.id], dh[edge.id]);
		}
		
		ofexpr.addTerm(1, dr, dr);
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
				yt[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "yt[" + edge.id + "]");
				yh[edge.id] = model.addVar(lb, ub, 0.0f, tipoBinary, "yh[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	// variaveis tipo D -- d_ij , distancia do grupo enraizado na aresta ij
	private void defineVarD(GRBModel model) {

		// variaveis tipo Y
		dt = new GRBVar[g.getEdgeCount()];
		dh = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = inst.net.getRoot().sumDist;
				dt[edge.id] = model.addVar(lb, ub, 0.0f, tipoFloat, "dt[" + edge.id + "]");
				dh[edge.id] = model.addVar(lb, ub, 0.0f, tipoFloat, "dh[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	// variaveis tipo F -- f_ij , fluxo acumulado na aresta ij de distância com sentido para a raiz
	private void defineVarF(GRBModel model) {

		// variaveis tipo Y
		ft = new GRBVar[g.getEdgeCount()];
		fh = new GRBVar[g.getEdgeCount()];

		try {

			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				// double objvalsY = 0.0f;
				// fault indicator
				double lb = 0.0;
				double ub = inst.net.getRoot().sumDist;
				ft[edge.id] = model.addVar(lb, ub, 0.0f, tipoFloat, "ft[" + edge.id + "]");
				fh[edge.id] = model.addVar(lb, ub, 0.0f, tipoFloat, "fh[" + edge.id + "]");
				// ofexpr.addTerm(objvalsY, y[edge.id]);
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}
	
	
	// variaveis tipo D -- dr , distancia acumulada na raiz
	private void defineVarDr(GRBModel model) {

		try {

			double lb = 0.0;
			double ub = inst.net.getRoot().sumDist;
			dr = model.addVar(lb, ub, 0.0f, tipoFloat, "dr");

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
	
	// RESTRICAO (1): dt + ft = sum (fh)
	// Alocacao de sinalizadores
	private void addConstraint1(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, dt[edge.id]);
				constraint.addTerm(1, ft[edge.id]);
				Iterator<E> outEdges = g.getOutEdges(g.getDest(edge)).iterator();
				while (outEdges.hasNext()) {
					E outEdge = outEdges.next();
					constraint.addTerm(-1, fh[outEdge.id]);
				}
				model.addConstr(constraint, GRB.EQUAL, 0, "flow_t["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (2): dh + fh = dist + ft
	// Alocacao de sinalizadores
	private void addConstraint2(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, fh[edge.id]);
				constraint.addTerm(1, dh[edge.id]);
				constraint.addTerm(-1, ft[edge.id]);
				model.addConstr(constraint, GRB.EQUAL, edge.dist, "flow_h["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (3): ft <= M (1 - yt)
	// Alocacao de sinalizadores
	private void addConstraint3(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double bigM = g.getDest(edge).sumDist;
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, ft[edge.id]);
				constraint.addTerm(bigM, yt[edge.id]);
				model.addConstr(constraint, GRB.LESS_EQUAL, bigM, "bigM_ft["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (4): fh <= M (1 - yh)
	// Alocacao de sinalizadores
	private void addConstraint4(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double bigM = g.getDest(edge).sumDist + edge.dist;
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, fh[edge.id]);
				constraint.addTerm(bigM, yh[edge.id]);
				model.addConstr(constraint, GRB.LESS_EQUAL, bigM, "bigM_fh["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (5): dt <= M (1 - yt)
	// Alocacao de sinalizadores
	private void addConstraint5(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double bigM = g.getDest(edge).sumDist;
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, dt[edge.id]);
				constraint.addTerm(-bigM, yt[edge.id]);
				model.addConstr(constraint, GRB.LESS_EQUAL, 0, "bigM_dt["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (6): dh <= M (1 - yh)
	// Alocacao de sinalizadores
	private void addConstraint6(GRBModel model) {
		try {
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				double bigM = g.getDest(edge).sumDist + edge.dist;
				GRBLinExpr constraint = new GRBLinExpr();
				constraint.addTerm(1, dh[edge.id]);
				constraint.addTerm(-bigM, yh[edge.id]);
				model.addConstr(constraint, GRB.LESS_EQUAL, 0, "bigM_dh["+edge.id+"]");
			}

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (7): dr = sum fh
	// Distancia acumulada na raiz
	private void addConstraint7(GRBModel model) {
		try {
			
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> outRootEdges = g.getOutEdges(inst.net.getRoot()).iterator();
			while (outRootEdges.hasNext()) {
				E edge = outRootEdges.next();
				constraint.addTerm(-1, fh[edge.id]);
			}
			constraint.addTerm(1, dr);
			model.addConstr(constraint, GRB.EQUAL, 0, "dr_constraint");
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	// RESTRICAO (8): sum d + omega = total_dist
	// Distancia acumulada na raiz
	private void addConstraint8(GRBModel model) {
		try {
			
			GRBLinExpr constraint = new GRBLinExpr();
			Iterator<E> iterEdges = g.getEdges().iterator();
			while (iterEdges.hasNext()) {
				E edge = iterEdges.next();
				constraint.addTerm(1, dh[edge.id]);
				constraint.addTerm(1, dt[edge.id]);
			}
			constraint.addTerm(1, dr);
			model.addConstr(constraint, GRB.GREATER_EQUAL, totalDist, "sumDist");
			
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}

	private void populateNewModel(GRBModel model) {

		try {

			GRBQuadExpr ofexpr = new GRBQuadExpr();

			// Create variables
			this.defineVarY(model);
			this.defineVarD(model);
			this.defineVarF(model);
			this.defineVarDr(model);
			this.generateOF(model, ofexpr);

			model.set(GRB.IntParam.LazyConstraints, 1);
			model.setCallback(this);

			// Integrate new variables
			model.update();

			model.setObjective(ofexpr);

			this.addConstraint0(model);
			
			this.addConstraint1(model);

			this.addConstraint2(model);

			this.addConstraint3(model);

			this.addConstraint4(model);

			this.addConstraint5(model);
			
			this.addConstraint6(model);
			
			this.addConstraint7(model);
			
			this.addConstraint8(model);


			model.update();

		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
		}

	}

}
