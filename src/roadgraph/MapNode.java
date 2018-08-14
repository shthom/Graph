package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author Shibu T
 * 
 * A class which represents a vertex of the graph
 * 
 *
 */
public class MapNode {
	
	private GeographicPoint location;
	private HashSet<MapEdge> adjListsMap;
	private double actualDistance;
	private double predictedDistance;
	
	/** 
	 * Create a new vertex of the Graph without neighbors
	 */
	public MapNode (GeographicPoint location) {
		this(location,null);
	}
	
	/** 
	 * Create a new vertex of the Graph with neighbors
	 */
	public MapNode (GeographicPoint location, MapEdge neighbor) {
		this.setNode(location);
		adjListsMap = new HashSet<MapEdge>();
		if(neighbor != null) {
			adjListsMap.add(neighbor);
		}
		
	}
	
	/** 
	 * Add a new egde to the existing vertex of the Graph
	 */
	public void addNeighbor(MapEdge neighbor) {
		
		if ( adjListsMap == null) {
			adjListsMap = new HashSet<MapEdge>();
		}
		
		adjListsMap.add(neighbor);
	}

	public GeographicPoint getLocation() {
		return location;
	}

	private void setNode(GeographicPoint location) {
		this.location = location;
	}
	
	public Set<MapNode> getNeighbors(){
		Set<MapNode> nodes = new HashSet<MapNode>();
		for(MapEdge e: adjListsMap) {
		
			nodes.add(e.getOtherNode(this));
		}
		
		return nodes;
	}
	
	public Set<MapEdge> getNeighborEdges(){
				
		return this.adjListsMap;
	}

	public double getActualDistance() {
		return actualDistance;
	}

	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}

	public double getPredictedDistance() {
		return predictedDistance;
	}

	public void setPredictedDistance(double predictedDistance) {
		this.predictedDistance = predictedDistance;
	}

	public double getCumulativeDistance() {
		return this.actualDistance + this.predictedDistance;
	}
	
	public double getLocationDistanceFromNode(MapNode other) {
		GeographicPoint otherLocation = other.getLocation();
		return this.location.distance(otherLocation);
	}
	

}
