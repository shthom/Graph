package roadgraph;

/**
 * @author Shibu T
 * 
 * A class which represents a edge of the graph
 * 
 */
public class MapEdge {
	private MapNode start;
	private MapNode end;
	private String streetName;
	private String streetType;
	private double distance;
	
	/** 
	 * create an edge of the Graph
	 */
	public MapEdge(MapNode start,MapNode end,String streetName, String streetType,double distance) {
		this.setStart(start);
		this.setEnd(end);
		this.setStreetName(streetName);
		this.setStreetType(streetType);
		this.setDistance(distance);
	}


	public MapNode getStart() {
		return start;
	}


	private void setStart(MapNode start) {
		this.start = start;
	}


	public String getStreetName() {
		return streetName;
	}


	private void setStreetName(String streetName) {
		this.streetName = streetName;
	}


	public String getStreetType() {
		return streetType;
	}


	private void setStreetType(String streetType) {
		this.streetType = streetType;
	}


	public MapNode getEnd() {
		return end;
	}


	private void setEnd(MapNode end) {
		this.end = end;
	}


	public double getDistance() {
		return distance;
	}


	private void setDistance(double distance) {
		this.distance = distance;
	}
	
	public MapNode getOtherNode(MapNode node) {
		
		if (node == start) {
			return end;
		}else if (node == end) {
			return start;
		}else {
			throw new IllegalArgumentException();
		}
	}

}
