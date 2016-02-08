package searchclient;

import java.util.Comparator;
import java.util.HashMap;
import java.awt.Point;
import java.lang.Character;

public abstract class Heuristic implements Comparator< Node > {

	public Node initialState;
  public HashMap goals;

	public Heuristic(Node initialState) {
		this.initialState = initialState;
    goals = new HashMap();
    for(int x=0; x<initialState.max_row; x++){
      for(int y=0; y<initialState.max_column; y++){
        if(Character.isLowerCase(initialState.goals[x][y]))
          goals.put(initialState.goals[x][y], new Point(x, y));
      }
    }
	}

	public int compare( Node n1, Node n2 ) {
		return f( n1 ) - f( n2 );
	}

	public int h( Node n ) {
    int total = 0;
    for(int x=0; x<n.max_row; x++){
      for(int y=0; y<n.max_column; y++){
        char c = n.boxes[x][y];
        if(Character.isUpperCase(c)){
          Point p = (Point)goals.get(Character.toLowerCase(c));
          total += Math.abs(x-p.x) + Math.abs(y-p.y);
        }
      }
    }
		return total;
	}

	public abstract int f( Node n );

	public static class AStar extends Heuristic {
		public AStar(Node initialState) {
			super( initialState );
		}

		public int f( Node n ) {
			return n.g() + h( n );
		}

		public String toString() {
			return "A* evaluation";
		}
	}

	public static class WeightedAStar extends Heuristic {
		private int W;

		public WeightedAStar(Node initialState) {
			super( initialState );
			W = 5; // You're welcome to test this out with different values, but for the reporting part you must at least indicate benchmarks for W = 5
		}

		public int f( Node n ) {
			return n.g() + W * h( n );
		}

		public String toString() {
			return String.format( "WA*(%d) evaluation", W );
		}
	}

	public static class Greedy extends Heuristic {

		public Greedy(Node initialState) {
			super( initialState );
		}

		public int f( Node n ) {
			return h( n );
		}

		public String toString() {
			return "Greedy evaluation";
		}
	}
}
