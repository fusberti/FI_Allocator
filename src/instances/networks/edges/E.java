package instances.networks.edges;

import instances.networks.vertices.V;

public class E {

	public int id, idProt, idNoProt, idGoodSec;
	public V node1, node2;
	public enum SwitchType {PROT,SWITCH,NONE};
	public SwitchType status;
	public double fl;
	public double p;
	// fault indicator
	public double dist; // km
	
	public E(int id, V node1, V node2, boolean prot, int idProt, int idNoProt) {
		this.id = id; // This is defined in the outer class.
		this.node1 = node1;
		this.node2 = node2;
		if (prot)
			this.status = SwitchType.PROT;
		else
			this.status = SwitchType.NONE;
		this.idProt = idProt; 
		this.idNoProt = idNoProt; 
		this.idGoodSec = -1; 
	}
	
	public E(int id, V node1, V node2, SwitchType status, int idProt, int idNoProt) {
		this.id = id; // This is defined in the outer class.
		this.node1 = node1;
		this.node2 = node2;	
		this.status = status;
		this.idProt = idProt; 
		this.idNoProt = idNoProt; 
		this.idGoodSec = -1; 
	}	
	
	public E(int id, V node1, V node2, boolean prot, int idProt, int idNoProt, int dist) {
		this.id = id; // This is defined in the outer class.
		this.node1 = node1;
		this.node2 = node2;
		if (prot)
			this.status = SwitchType.PROT;
		else
			this.status = SwitchType.NONE;
		this.idProt = idProt; 
		this.idNoProt = idNoProt; 
		this.idGoodSec = -1; 
		this.dist = dist;
	}

	public String toString() {
		//return "E("+this.node1.label+","+this.node2.label+","+this.p+")";
		return "E("+this.node1.label+","+this.node2.label+")";
	}
					
}
