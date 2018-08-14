/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numVertices;
	private int numEdges;
	private HashMap<GeographicPoint,MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<GeographicPoint,MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		
		MapNode node = new MapNode(location);
		this.vertices.put(location, node);
		numVertices++;
			
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		MapNode nodeFrom = vertices.get(from);
		MapNode nodeTo =  vertices.get(to);
		
		//nodes not found in the graph
		if( nodeFrom == null || nodeTo == null) {		
			
			throw new IllegalArgumentException();
		}
		
		MapEdge edge = new MapEdge(nodeFrom,nodeTo,roadName,roadType,length);
		
		//Add edge as neighbor of the from node
		nodeFrom.addNeighbor(edge);
		numEdges++;
		
	
		
		
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		Queue<MapNode> queue = new LinkedList<MapNode>();
		
		
			
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		
		queue.add(startNode);
		visited.add(startNode);
		nodeSearched.accept(start);
		MapNode currNode = null;
		
		
		while (!queue.isEmpty()) {
			
			currNode = queue.poll();
			nodeSearched.accept(currNode.getLocation());
			if (currNode == goalNode) {
				break;
			}
			
			for(MapNode w: currNode.getNeighbors()) {			
				
				//check if node has been visited
				if(!visited.contains(w)) {
					queue.add(w);
					visited.add(w);
					parentMap.put(w,currNode);
					
				}
				
			}
		}
		
		
		return unrollPath(startNode,goalNode,parentMap,currNode.equals(goalNode));
		
	}
	
	/*
	 * Procedure to get return the Path as Geographic points using the HashMap parentMap
	 */
	
	private List<GeographicPoint> unrollPath(MapNode start, MapNode end, HashMap<MapNode,MapNode> parentMap, boolean pathFound) {
		
		if(!pathFound) {
			return null;
		}
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		MapNode current = end;
		
		while(current != start) {
			path.addFirst(current.getLocation());
			current =  parentMap.get(current);
		
		}
		path.addFirst(start.getLocation());
		
		
		return path;
	}
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode> ();
		//PriorityQueue
		//Comparator<MapNode> comparator = new MapNodeComparator();
	    //PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(getNumVertices(), comparator);
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(getNumVertices(), new Comparator<MapNode>() {

			@Override
			public int compare(MapNode node1, MapNode node2) {
				return (node1.getActualDistance() > node2.getActualDistance() ? 1 : -1);
			}}
		);
			
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		
		//initialize Node distance to infinity
		for(GeographicPoint w:vertices.keySet()) {
			MapNode n = vertices.get(w);
			n.setActualDistance(Double.POSITIVE_INFINITY);			
		}
		
		//initialize start node distance to zero
		MapNode currentNode = null;
		startNode.setActualDistance(0);
		queue.add(startNode);
		
		while(!queue.isEmpty()) {
			
			currentNode = queue.poll();
			nodeSearched.accept(currentNode.getLocation());
			System.out.println("DIJKSTRA visiting[NODE at location(Lat: " + currentNode.getLocation().getX() + " Lon: "
	 + 	currentNode.getLocation().getY() + ")");	
			if(!visited.contains(currentNode)) {
				visited.add(currentNode);
				
				if (currentNode == goalNode) {
					break;
				}
				
				
				for(MapEdge e: currentNode.getNeighborEdges()) {
					MapNode nextNode = e.getOtherNode(currentNode);
					//add current node distance and edge distance 
					double newDistance = currentNode.getActualDistance()  + e.getDistance(); //currentNode.getLocationDistanceFromNode(nextNode); //
									
					
					if  (!visited.contains(nextNode)) 
					 {
						
						//update distance of the node only if the distance from current node is less than previously computed
						//update the parent node only if the path from the current node is less than previously computed
						if(newDistance < nextNode.getActualDistance()) {
						nextNode.setActualDistance(newDistance);						
						parentMap.put(nextNode, currentNode);	
						}
						queue.add(nextNode);
					}
					
					
				}
			}
			
		}
		
		
		return unrollPath(startNode,goalNode,parentMap,currentNode.equals(goalNode));
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
	    PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(getNumVertices(), new Comparator<MapNode>() {

			@Override
			public int compare(MapNode node1, MapNode node2) {
				// TODO Auto-generated method stub
				return node1.getCumulativeDistance() > node2.getCumulativeDistance() ? 1:-1;
			}
	    	
	    });
		
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		
		// initialize distance to infinity
		
		for(MapNode n: vertices.values()) {
			
			n.setActualDistance(Double.POSITIVE_INFINITY);
			n.setPredictedDistance(Double.POSITIVE_INFINITY);
		
		}
		
		startNode.setActualDistance(0);
		startNode.setPredictedDistance(0);
		
		queue.add(startNode);
		
		MapNode currentNode = null;
		
		while(!queue.isEmpty()) {
			
			currentNode = queue.poll();
			nodeSearched.accept(currentNode.getLocation());
			System.out.println("A* visiting[NODE at location(Lat: " + currentNode.getLocation().getX() + " Lon: "
					 + 	currentNode.getLocation().getY() + ")");
			
			if(!visited.contains(currentNode)) {
				visited.add(currentNode);
				
				if (currentNode == goalNode) {
					break;
				}
				
				
				for(MapEdge e: currentNode.getNeighborEdges()) {
					MapNode nextNode = e.getOtherNode(currentNode);
					
					double predictedDistance =  nextNode.getLocationDistanceFromNode(goalNode);
					
					double actualDistance = currentNode.getActualDistance() + e.getDistance() ;
										
					if ( (!visited.contains(nextNode))  )
					{
						if (actualDistance < nextNode.getActualDistance()) {
						nextNode.setActualDistance(actualDistance);
						nextNode.setPredictedDistance(predictedDistance);
						parentMap.put(nextNode, currentNode);
						}
						queue.add(nextNode);
					}
				}
				
			}
			
		}
		
		
		return unrollPath(startNode,goalNode,parentMap,currentNode.equals(goalNode));
	}

	


	public static void main(String[] args)
	{
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		List<GeographicPoint> testroute = firstMap.bfs(testStart,testEnd);
		
		System.out.println("DONE.");
		*/
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("DONE.");
		*/
		/*
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
		
		/*
	    	MapGraph simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
			GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
			
			System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
			List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
			List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
			
			
			MapGraph testMap = new MapGraph();
			GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
			// A very simple test using real data
			testStart = new GeographicPoint(32.869423, -117.220917);
			testEnd = new GeographicPoint(32.869255, -117.216927);
			
	
			
			System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
		
			 */
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}
	
}
