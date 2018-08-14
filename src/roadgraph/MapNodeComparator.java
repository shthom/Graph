package roadgraph;

import java.util.Comparator;

public class MapNodeComparator implements Comparator<MapNode> {

	@Override
	public int compare(MapNode v, MapNode w) {
		
		return (int) (v.getCumulativeDistance() - w.getCumulativeDistance());
	}

}
