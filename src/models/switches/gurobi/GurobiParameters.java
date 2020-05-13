package models.switches.gurobi;

import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;

/* Gurobi parameters */
public class GurobiParameters {
					
	/* See explanations in the get/set functions */ 
	private int threads, cutpasses;
	private double time, memsize, cutoff;
	private CUTS cut;
	private MIPFOCUSES mipfocus;
	private GRBModel model;
	private PRECRUSHES precrush;
	private double intfeastol;
	
	public enum MIPFOCUSES {gurobi_default,feasibility,optimality,lowerbound};
	public enum CUTS {auto,off,moderate,agressive,very_agressive};
	public enum PRECRUSHES {off,on};


	/* Default values for the Gurobi parameters. */
	public GurobiParameters(GRBModel model) {

		super();
		this.setModel(model);
		this.setThreads(1);
		this.setTime(600.0f);
		this.setMemsize(3.0f);
		this.setPrecrush(PRECRUSHES.on);
		this.setMipfocus(MIPFOCUSES.optimality);
		this.setCut(CUTS.off);
		this.setCutoff(Double.MAX_VALUE);
		this.setCutpasses(-1);
		this.setIntfeastol(1e-9);
		
	}
	
	/*
	 * Controls the number of threads to apply to parallel barrier or parallel
	 * MIP. The default value of 0 sets the thread count equal to its maximum
	 * value, which is the number of logical cores in the machine. While you
	 * will generally get the best performance by using all available cores in
	 * your machine, there are a few exceptions. One is of course when you are
	 * sharing a machine with other jobs. In this case, you should select a
	 * thread count that doesn't oversubscribe the machine. We have also found
	 * that certain classes of MIP models benefit from reducing the thread
	 * count, often all the way down to one thread. Starting multiple threads
	 * introduces contention for machine resources. For classes of models where
	 * the first solution found by the MIP solver is almost always optimal, and
	 * that solution isn't found at the root, it is often better to allow a
	 * single thread to explore the search tree uncontended. Another situation
	 * where reducing the thread count can be helpful is when memory is tight.
	 * Each thread can consume a significant amount of memory.
	 */
	public int getThreads() {
		return threads;
	}
	public void setThreads(int threads) {
		try {
			this.threads = threads;
			this.model.getEnv().set(GRB.IntParam.Threads, threads);
		} catch (GRBException e) {
			System.out.println("setThreads: " + e.getErrorCode() + ". " + e.getMessage());
		}
	}
	
	/*
	 * Allows presolve to translate constraints on the original model to
	 * equivalent constraints on the presolved model. You must turn this
	 * parameter on when you are using callbacks to add your own cuts.
	 */
	public PRECRUSHES getPrecrush() {
		return precrush;
	}
	public void setPrecrush(PRECRUSHES precrush) {
		try {
			this.precrush = precrush;
	        switch (this.precrush) {
	            case off:
	            	model.getEnv().set(GRB.IntParam.PreCrush, 0);
	            	break;
	            case on:
	            	model.getEnv().set(GRB.IntParam.PreCrush, 1);
	            	break;
	        }
		} catch (GRBException e) {
			System.out.println("setPrecrush: " + e.getErrorCode() + ". " + e.getMessage());
		}		
	}
	
	/*
	 * Limits the total time expended (in seconds). Optimization returns with a
	 * TIME_LIMIT status if the limit is exceeded.
	 */
	public double getTime() {
		return time;
	}
	public void setTime(double time) {
		try {
			this.time = time;
			model.getEnv().set(GRB.DoubleParam.TimeLimit, time);
		} catch (GRBException e) {
			System.out.println("setTime: " + e.getErrorCode() + ". " + e.getMessage());
		}		
	}
	
	/*
	 * If you find that the Gurobi optimizer exhausts memory when solving a MIP,
	 * you should modify the NodefileStart parameter. When the amount of memory
	 * used to store nodes (measured in GBytes) exceeds the specified parameter
	 * value, nodes are written to disk. We recommend a setting of 0.5, but you
	 * may wish to choose a different value, depending on the memory available
	 * in your machine. By default, nodes are written to the current working
	 * directory. If you still exhaust memory after setting the NodefileStart
	 * parameter to a small value, you should try limiting the thread count.
	 * Each thread in parallel MIP requires a copy of the model, as well as
	 * several other large data structures. Reducing the Threads parameter can
	 * sometimes significantly reduce memory usage.
	 */
	public double getMemsize() {
		return memsize;
	}
	public void setMemsize(double memsize) {
		try {
			this.memsize = memsize;
			model.getEnv().set(GRB.DoubleParam.NodefileStart, memsize);		
		} catch (GRBException e) {
			System.out.println("setMemsize: " + e.getErrorCode() + ". " + e.getMessage());
		}	
	}
	
	/*
	 * Indicates that you aren't interested in solutions whose objective values
	 * are worse than the specified value. If the objective value for the
	 * optimal solution is better than the specified cutoff, the solver will
	 * return the optimal solution. Otherwise, it will terminate with a CUTOFF
	 * status (see the Status Code section for further details).
	 */
	public double getCutoff() {
		return cutoff;
	}
	public void setCutoff(double cutoff) {
		this.cutoff = cutoff;
		try {
			model.getEnv().set(GRB.DoubleParam.Cutoff, cutoff);
		} catch (GRBException e) {
			System.out.println("setCutoff: " + e.getErrorCode() + ". " + e.getMessage());
		}	
	}
	
	/*
	 * Global cut aggressiveness setting. A value of -1 corresponds to an
	 * automatic setting, which allows the solver to determine the appropriate
	 * level of aggressiveness in the cut generation. Use value 0 to shut off
	 * cuts, 1 for moderate cut generation, 2 for aggressive cut generation, and
	 * 3 for very aggressive cut generation. This parameter is overridden by the
	 * parameters that control individual cut types (e.g., CliqueCuts).
	 */
	public CUTS getCut() {
		return cut;
	}
	public void setCut(CUTS cut) {
		try {
			this.cut = cut;
	        switch (this.cut) {
            	case auto:
	            	model.getEnv().set(GRB.IntParam.Cuts, -1);
	            	break;
	        	case off:
	            	model.getEnv().set(GRB.IntParam.Cuts, 0);
	            	break;
	            case moderate:
	            	model.getEnv().set(GRB.IntParam.Cuts, 1);
	            	break;
	            case agressive:
	            	model.getEnv().set(GRB.IntParam.Cuts, 2);
	            	break;	            	
	            case very_agressive:
	            	model.getEnv().set(GRB.IntParam.Cuts, 3);
	            	break;	            	
	        }
		} catch (GRBException e) {
			System.out.println("setCut: " + e.getErrorCode() + ". " + e.getMessage());
		}	
	}
	
	/*
	 * The MIPFocus parameter allows you to modify your high-level solution
	 * strategy, depending on your goals. By default, the Gurobi MIP solver
	 * strikes a balance between finding new feasible solutions and proving that
	 * the current solution is optimal. If you are more interested in finding
	 * feasible solutions quickly, you can select MIPFocus=1. If you believe the
	 * solver is having no trouble finding good quality solutions, and wish to
	 * focus more attention on proving optimality, select MIPFocus=2. If the
	 * best objective bound is moving very slowly (or not at all), you may want
	 * to try MIPFocus=3 to focus on the bound.
	 */
	public MIPFOCUSES getMipfocus() {
		return mipfocus;
	}
	public void setMipfocus(MIPFOCUSES mipfocus) {
		try {
			this.mipfocus = mipfocus;
	        switch (this.mipfocus) {
	            case gurobi_default:
	            	model.getEnv().set(GRB.IntParam.MIPFocus, 0);
	            	break;
	            case feasibility:
	            	model.getEnv().set(GRB.IntParam.MIPFocus, 1);
	            	break;
	            case optimality:
	            	model.getEnv().set(GRB.IntParam.MIPFocus, 2);
	            	break;	            	
	            case lowerbound:
	            	model.getEnv().set(GRB.IntParam.MIPFocus, 3);
	            	break;	            	
	        }
		} catch (GRBException e) {
			System.out.println("setMipfocus: " + e.getErrorCode() + ". " + e.getMessage());
		}	
	}
	
	/*
	 * A non-negative value indicates the maximum number of cutting plane passes
	 * performed during root cut generation. The default value chooses the
	 * number of cut passes automatically. You should experiment with different
	 * values of this parameter if you notice the MIP solver spending
	 * significant time on root cut passes that have little impact on the
	 * objective bound.
	 */
	public int getCutpasses() {
		return cutpasses;
	}
	public void setCutpasses(int cutpasses) {
		try {
			this.cutpasses = cutpasses;
			model.getEnv().set(GRB.IntParam.CutPasses, cutpasses);		
		} catch (GRBException e) {
			System.out.println("setCutpasses: " + e.getErrorCode() + ". " + e.getMessage());
		}			
	}
	
	/*
	 * An integrality restriction on a variable is considered satisfied when the
	 * variable's value is less than IntFeasTol from the nearest integer value.
	 * Tightening this tolerance can produce smaller integrality violations, but
	 * very tight tolerances may significantly increase runtime. Loosening this
	 * tolerance rarely reduces runtime.
	 * 
	 * Note: Only affects mixed integer programming (MIP) models
	 */
	public double getIntfeastol() {
		return intfeastol;
	}
	public void setIntfeastol(double intfeastol) {
		this.intfeastol = intfeastol;
	}	


	public void setModel(GRBModel model) {
		this.model = model;
	}

	
}
