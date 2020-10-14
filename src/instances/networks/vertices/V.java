package instances.networks.vertices;

public class V {

	public int id, label; // good coding practice would have this as private
	public int clients, clientsSum;
	public double thetaL, thetaR, demand, demandSum;
	public double M;
	public enum TYPES {ROOT,SUBSTATION,CLIENT};
	public TYPES type;
	public V subNode;
	public double capacity;
	public double remainCapacity;
	public double upFlow;
//	// data for Dynamic Programming
//	public enum TOPOLOGIES {LINE, BRANCH, LEAF};
//	public TOPOLOGIES topo;
//	public List<V> branches;
	public double thetaTil;
	public double ENDmin;
	public V nextBranch, firstSucessor;
	public int numSubNodes;
	// fault indicator
	public double distFromSE; // km
	public double coordX;
	public double coordY;

	public V(int id, int label,double x, double y) {
		this.id = id;
		this.label = label;	
		//if clients == -1, then this node represents a substation
		coordX = x;
		coordY = y;
	}
	
	public V(int id, int label, double thetaL, double thetaR, double power, int clients) {
		this.id = id;
		this.label = label;
		this.thetaL = thetaL;
		this.thetaR = thetaR;	
		//if clients == -1, then this node represents a substation
		if (clients < 0) {
			this.type = TYPES.SUBSTATION;
			this.clients = 0;
			this.demand = 0;
			this.capacity = power;
		} else {
			this.type = TYPES.CLIENT;
			this.clients = clients;
			this.demand = power;
			this.capacity = 0;
		}
		if (id == 0) {
			this.type = TYPES.ROOT;
		}
	}

	public String toString() {
		return "V"+label;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (obj instanceof V) {
			V v = (V) obj;
			return (this.id == v.id);
		}
		return false;
	}
	
}
