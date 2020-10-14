package instances;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.Reader;
import java.io.StreamTokenizer;

public class InstanceParameters {
	
	/* See explanations in the get/set functions */ 
	private String instanceName;
	private int numSwitches;
	private OBJECTIVES OBJ;
	private double limCost, limENS, limSAIDI;
	private double unitSwitchCost, kwhCost;
	private double protFailRate;
	private boolean transfer;
	//private final String parametersFile = "parameters/parameters.txt";
	private final String parametersFile = "parameters/parameters_fi.txt";
	// fault indicator
	public double alpha, ro;
	private int numFI;
	double crewVelocity, failureRate;
	
	public enum OBJECTIVES {ENS,SAIDI,COST};

	
	public InstanceParameters() {

		try {			
			Reader r = new BufferedReader(new FileReader(parametersFile));
	    	StreamTokenizer tok = new StreamTokenizer(r);
	    	tok.resetSyntax();
	    	tok.whitespaceChars('\u0000', '\u0020');
	    	tok.wordChars('a', 'z');
	    	tok.wordChars('A', 'Z');
	    	tok.wordChars('\u00A0', '\u00FF');
	    	tok.wordChars('/', '/');
	    	tok.quoteChar('\'');
	    	tok.quoteChar('"');
	    	tok.eolIsSignificant(false);
	    	tok.slashSlashComments(false);
	    	tok.slashStarComments(false);
	    	//tok.parseNumbers();  // this WOULD be part of the standard syntax

	    	// syntax additions
	    	tok.wordChars('0', '9');
	    	tok.wordChars('.', '.');
	    	tok.wordChars('_', '_');
	    	//collect instance name
	    	this.collectField(tok);
	    	//this.setInstanceName("instancias/"+stok.sval);
	    	this.setInstanceName(tok.sval);
 
	    	//collect the number of FIs that will be installed on the network.
	    	this.collectField(tok);
	    	String strNI = tok.sval;
	        this.setNumFI(Integer.parseInt(strNI));	        	        
	    	   		    	    	
	    } catch (Exception e) {
	    	System.err.println("Read error of method Parameters()");
	    	e.printStackTrace();
	    }		
		
	}

	
	
	/* Default values for the Instance parameters. */
//	public InstanceParameters() {
//
//		try {
//			
//			Reader r = new BufferedReader(new FileReader(parametersFile));
//	    	StreamTokenizer stok = new StreamTokenizer(r);
//	    	
//	    	//collect instance name
//	    	this.collectField(stok);
//	    	//this.setInstanceName("instancias/"+stok.sval);
//	    	this.setInstanceName(stok.sval);
// 
//	    	//collect objective function
//	    	this.collectField(stok);
//	        if (stok.sval.equals("ENS"))
//	        	this.setOBJ(OBJECTIVES.ENS);
//	        else if (stok.sval.equals("SAIDI"))
//	        	this.setOBJ(OBJECTIVES.SAIDI);
//	        else
//	        	this.setOBJ(OBJECTIVES.COST);
//	        
//	    	//collect the cost of energy ($/kWh).
//	        this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setKwhCost(stok.nval);
//	        else
//	        	this.setKwhCost(Double.MAX_VALUE);
//	        
//	    	//collect the average annual cost of a switch ($).
//	    	this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setUnitSwitchCost(stok.nval);
//	        else
//	        	this.setUnitSwitchCost(Double.MAX_VALUE);
//	        
//	        
//	    	//collect the probability a breaker will fail upon a contingency.
//	    	this.collectField(stok);
//        	this.setProtFailRate(stok.nval);
//	        
//        	
//	    	//collect the number of FIs that will be installed on the network.
//	    	this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setNumFI((int) stok.nval);
//	        else
//	        	this.setNumFI(Integer.MAX_VALUE);
//
//	        
//	    	//collect the cost that can be invested in a solution. 
//	    	this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setLimCost(stok.nval);
//	        else
//	        	this.setLimCost(Double.MAX_VALUE);        
//	        
//	        
//	    	//collect the expected system average interruption duration index (SAIDI) of a solution.
//	    	this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setLimSAIDI(stok.nval);
//	        else
//	        	this.setLimSAIDI(Double.MAX_VALUE);        
//	        
//	        
//	    	//collect the expected amount of energy not supplied (ENS) of a solution.
//	    	this.collectField(stok);
//	        if (stok.sval == null)
//	        	this.setLimENS(stok.nval);
//	        else
//	        	this.setLimENS(Double.MAX_VALUE);    
//	        
//	        
//	    	//collect the boolean variable transfer.
//	    	this.collectField(stok);
//	        if (stok.sval.equals("TRUE"))
//	        	this.setTransfer(true);
//	        else
//	        	this.setTransfer(false);			        
//	        
//	    	   		    	    	
//	    } catch (Exception e) {
//	    	System.err.println("Read error of method Parameters()");
//	    	e.printStackTrace();
//	    }		
//		
//	}
	
	/* Collect a field of the parameters file. */
	private void collectField(StreamTokenizer stok) {
		String line;
		try {
			for (int i=0;i<2;i++)
				stok.nextToken();
			
	    } catch (Exception e) {
	    	System.err.println("Read error of method collectField(StreamTokenizer stok)");
	    	e.printStackTrace();
	    }			
		
	}	
	
	/* Gives the name of the instance being solved.  */	
	public String getInstanceName() {
		return instanceName;
	}
	public void setInstanceName(String instanceName) {
		this.instanceName = instanceName;
	}	
	
	/* Gives the criteria that will be optimized (minimized).  */
	public OBJECTIVES getOBJ() {
		return OBJ;
	}
	public void setOBJ(OBJECTIVES oBJ) {
		OBJ = oBJ;
	}
	
	/* Bounds the number of switches that will be installed on the network. */
	public int getNumSwitches() {
		return numSwitches;
	}
	public void setNumSwitches(int numSwitches) {
		this.numSwitches = numSwitches;
	}

	/* Bounds the cost that can be invested in a solution. */
	public double getLimCost() {
		return limCost;
	}
	public void setLimCost(double limCost) {
		this.limCost = limCost;
	}

	/* Bounds the expected amount of energy not supplied (ENS) of a solution. */	
	public double getLimENS() {
		return limENS;
	}
	public void setLimENS(double limENS) {
		this.limENS = limENS;
	}

	
	/* Bounds the expected system average interruption duration index (SAIDI) of a solution. */
	public double getLimSAIDI() {
		return limSAIDI;
	}
	public void setLimSAIDI(double limSAIDI) {
		this.limSAIDI = limSAIDI;
	}


	/* The average annual cost of a switch ($). */
	public double getUnitSwitchCost() {
		return unitSwitchCost;
	}
	public void setUnitSwitchCost(double unitSwitchCost) {
		this.unitSwitchCost = unitSwitchCost;
	}

	
	/* The cost of energy ($/kWh). */
	public double getKwhCost() {
		return kwhCost;
	}
	public void setKwhCost(double kwhCost) {
		this.kwhCost = kwhCost;
	}


	/* The probability a breaker will fail upon a contingency. */
	public double getProtFailRate() {
		return protFailRate;
	}
	public void setProtFailRate(double protFailRate) {
		this.protFailRate = protFailRate;
	}	

	
	/* transfer == TRUE is transfer through tie lines is possible, transfer == FALSE otherwise. */	
	public boolean isTransfer() {
		return transfer;
	}
	public void setTransfer(boolean transfer) {
		this.transfer = transfer;
	}	
	
	/* number of fault indicartors */	
	public int getNumFI() {
		return numFI;
	}
	public void setNumFI(int numFI) {
		this.numFI = numFI;
	}
	
	
	
	public double getCrewVelocity() {
		return crewVelocity;
	}



	public void setCrewVelocity(double crewVelocity) {
		this.crewVelocity = crewVelocity;
	}



	public double getFailureRate() {
		return failureRate;
	}



	public void setFailureRate(double failureRate) {
		this.failureRate = failureRate;
	}



	public static void main(String[] args) {
		
		InstanceParameters p = new InstanceParameters();
		//System.out.println(p.toString());
		
	}

	@Override
	public String toString() {
		return "InstanceParameters [instanceName=" + instanceName + ", numSwitches="
				+ ", parametersFile=" + parametersFile
				+ ", numFI=" + numFI + "]";
	}
//	public String toString() {
//		return "InstanceParameters [instanceName=" + instanceName + ", numSwitches="
//				+ numSwitches + ", OBJ=" + OBJ + ", limCost=" + limCost
//				+ ", limENS=" + limENS + ", limSAIDI=" + limSAIDI
//				+ ", unitSwitchCost=" + unitSwitchCost + ", kwhCost=" + kwhCost
//				+ ", protFailRate=" + protFailRate + ", transfer=" + transfer
//				+ ", parametersFile=" + parametersFile + "]";
//	}
	
	
}
