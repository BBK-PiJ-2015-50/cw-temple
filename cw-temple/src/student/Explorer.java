package student;

import com.sun.scenario.effect.impl.sw.sse.SSEBlend_SRC_OUTPeer;
import game.*;

import java.util.*;

public class Explorer {

    /**
     * Explore the cavern, trying to find the orb in as few steps as possible.
     * Once you find the orb, you must return from the function in order to pick
     * it up. If you continue to move after finding the orb rather
     * than returning, it will not count.
     * If you return from this function while not standing on top of the orb,
     * it will count as a failure.
     * <p>
     * There is no limit to how many steps you can take, but you will receive
     * a score bonus multiplier for finding the orb in fewer steps.
     * <p>
     * At every step, you only know your current tile's ID and the ID of all
     * open neighbor tiles, as well as the distance to the orb at each of these tiles
     * (ignoring walls and obstacles).
     * <p>
     * To get information about the current state, use functions
     * getCurrentLocation(),
     * getNeighbours(), and
     * getDistanceToTarget()
     * in ExplorationState.
     * You know you are standing on the orb when getDistanceToTarget() is 0.
     * <p>
     * Use function moveTo(long id) in ExplorationState to move to a neighboring
     * tile by its ID. Doing this will change state to reflect your new position.
     * <p>
     * A suggested first implementation that will always find the orb, but likely won't
     * receive a large bonus multiplier, is a depth-first search.
     *
     * @param state the information available at the current state
     */
    public void explore(ExplorationState state) {
        Set<Long> visitedTiles = new LinkedHashSet<>();
        Deque<Long> route = new LinkedList();
        long nextTile, shortestDistance;
        Collection<NodeStatus> neighbours;
        Boolean noUnvisitedNeighbourFound;

        // Stop when the orb is reached, ie. distance is zero
        while (state.getDistanceToTarget() > 0) {
            visitedTiles.add(state.getCurrentLocation());
            neighbours = state.getNeighbours();
            // Initialise nextTile before checking for unvisited tiles
            nextTile = -1L;
            // Initialise shortestDistance before looking for tile closest to orb
            shortestDistance = Integer.MAX_VALUE;
            noUnvisitedNeighbourFound = true;
            // Look for neighbouring tiles not yet visited
            for (NodeStatus tile : neighbours) {
                if (!visitedTiles.contains(tile.getId())) {
                    noUnvisitedNeighbourFound = false;
                    // Look for neighbouring tile closest to orb
                    if (tile.getDistanceToTarget() < shortestDistance) {
                        shortestDistance = tile.getDistanceToTarget();
                        nextTile = tile.getId();
                    }
                }
            }
            // Go back if no unvisited neighbouring tiles are found
            if (noUnvisitedNeighbourFound) {
                nextTile = route.pop();
            } else {
                // Record route so we can retrace our steps
                route.push(state.getCurrentLocation());
            }
            state.moveTo(nextTile);
        }
    }

    /**
     * Escape from the cavern before the ceiling collapses, trying to collect as much
     * gold as possible along the way. Your solution must ALWAYS escape before time runs
     * out, and this should be prioritized above collecting gold.
     * <p>
     * You now have access to the entire underlying graph, which can be accessed through EscapeState.
     * getCurrentNode() and getExit() will return you Node objects of interest, and getVertices()
     * will return a collection of all nodes on the graph.
     * <p>
     * Note that time is measured entirely in the number of steps taken, and for each step
     * the time remaining is decremented by the weight of the edge taken. You can use
     * getTimeRemaining() to get the time still remaining, pickUpGold() to pick up any gold
     * on your current tile (this will fail if no such gold exists), and moveTo() to move
     * to a destination node adjacent to your current node.
     * <p>
     * You must return from this function while standing at the exit. Failing to do so before time
     * runs out or returning from the wrong location will be considered a failed run.
     * <p>
     * You will always have enough time to escape using the shortest path from the starting
     * position to the exit, although this will not collect much gold.
     *
     * @param state the information available at the current state
     */
    public void escape(EscapeState state) {

        /**
         *  Implementation of A* algorithm for finding a path through the cavern
         *
         *  Searches possible paths for one with the lowest cost
         *  - in this case that's the shortest distance/lowest number of steps
         *
         *  The equation used in the algorithm is F = G + H
         *  G - the cost of moving from the start position to the node in question
         *  H - the heuristic...estimated cost of moving this node to the exit
         *
         *  The algorithm looks for the lowest F cost at each stage
         *
         *  The supplied Priority Queue is used for the "open" nodes
         */

        PriorityQueue<Node> openNodes = new PriorityQueueImpl<>();
        HashMap<Node, NodeInformation> nodeMap = new HashMap<>();
        Node pathNode, start;
        double lengthStartToParent, lengthStartToNeighbour, heuristic, lengthStartToExitViaNeighbour;

        // All nodes on the graph are mapped to NodeInformation objects
        Collection<Node> vertices = state.getVertices();
        for (Node vertex : vertices) {
            nodeMap.put(vertex, new NodeInformation());
        }
        // Add the Start node to the Open queue with priority = 0
        start = state.getCurrentNode();
        addToOpenNodesQueue(openNodes, nodeMap, start, 0);
        pathNode = start;
        // Repeat until the exit node is reached
        while (pathNode != state.getExit()) {
            // Get lowest F cost node from the Open queue and set to Closed
            pathNode = openNodes.poll();
            nodeMap.get(pathNode).setClosed();
            // Stop once the exit node is set to Closed
            if (pathNode != state.getExit()) {
                /* Get the set of edges leaving this node
                 * and the destination nodes at the ends of the edges */
                for (Edge edge : pathNode.getExits()) {
                    Node neighbour = edge.getDest();
                    // Don't check a node if it's Closed
                    if (!nodeMap.get(neighbour).getClosed()) {
                        // Go through nodes that aren't Open
                        if (!nodeMap.get(neighbour).getOpen()) {
                            // Set the parent of this neighbour node as the current pathNode
                            nodeMap.get(neighbour).setParent(pathNode);
                            // Calculate G cost (length from Start) for this neighbour node & store value
                            lengthStartToParent = nodeMap.get(nodeMap.get(neighbour)
                                    .getParent()).getLengthStartToNode();
                            lengthStartToNeighbour = lengthStartToParent + edge.length();
                            nodeMap.get(neighbour).setLengthStartToNode(lengthStartToNeighbour);
                            // Calculate heuristic (H) and store value
                            heuristic = calculateHeuristic(state, neighbour);
                            nodeMap.get(neighbour).setHeuristic(heuristic);
                            // Calculate F (length from Start + heuristic) for this neighbour node & store value
                            lengthStartToExitViaNeighbour = lengthStartToNeighbour + heuristic;
                            nodeMap.get(neighbour).setLengthStartToExitViaNode(lengthStartToExitViaNeighbour);
                            /* Add this neighbour node to the Open queue
                             * with priority being the length from Start to Exit via this node */
                            addToOpenNodesQueue(openNodes, nodeMap, neighbour, lengthStartToExitViaNeighbour);
                        } else {
                            /*  If this neighbour node is already on the Open queue,
                                check if path to Exit is shorter via this pathNode than previous path
                                First calculate length of path from Start to this neighbour node via pathNode */
                            double lengthStartToPathNode = nodeMap.get(pathNode).getLengthStartToNode();
                            double lengthPathNodeToNeighbour = pathNode.getEdge(neighbour).length();
                            double lengthStartToNeighbourViaPathNode = lengthStartToPathNode + lengthPathNodeToNeighbour;
                            // Then compare with previous path length
                            if (lengthStartToNeighbourViaPathNode < nodeMap.get(neighbour).getLengthStartToNode()) {
                                // If shorter - set parent of neighbour to pathNode
                                nodeMap.get(neighbour).setParent(pathNode);
                                // Update G cost (length from Start) for this neighbour node
                                nodeMap.get(neighbour).setLengthStartToNode(lengthStartToNeighbourViaPathNode);
                                // Update F (length from Start + heuristic) for this neighbour node
                                double lengthStartToExitViaPathNode = lengthStartToNeighbourViaPathNode
                                        + nodeMap.get(neighbour).getHeuristic();
                                nodeMap.get(neighbour).setLengthStartToExitViaNode(lengthStartToExitViaPathNode);
                                // Update priority on Open queue
                                openNodes.updatePriority(neighbour, lengthStartToExitViaPathNode);
                            }
                        }
                    }
                }
            }
        }

        // Save path
        ArrayList<Node> path = new ArrayList<>();
        pathNode = state.getExit();
        while (pathNode != start) {
            path.add(pathNode);
            pathNode = nodeMap.get(pathNode).getParent();
        }
        // Reverse path then follow it from start to exit
        Collections.reverse(path);
        for (Node node : path) {
            state.moveTo(node);
            // Pick up gold if there's any on a tile
            if (node.getTile().getGold() > 0) {
                state.pickUpGold();
            }
        }
    }

    /* Adds nodes to the Open queue and marks them as Open in NodeInformation
     * and sets priority as the length from the Start to Exit via the node */
    private void addToOpenNodesQueue(PriorityQueue<Node> openNodes, HashMap<Node, NodeInformation> nodeMap,
                                     Node node, double priority) {
        openNodes.add(node, priority);
        nodeMap.get(node).setOpen(true);
        nodeMap.get(node).setLengthStartToExitViaNode(priority);
    }

    private static class NodeInformation {
        Boolean closed = false;
        Boolean open = false;
        Node parent;
        // A* algorithm: G cost
        double lengthStartToNode;
        // A* algorithm: H
        double heuristic;
        // A* algorithm: F
        double lengthStartToExitViaNode;

        public NodeInformation() {}

        public void setOpen(Boolean open) {
            this.open = open;
        }

        public Boolean getOpen () {
            return open;
        }

        public void setClosed() {
            closed = true;
        }

        public Boolean getClosed() {
            return closed;
        }

        public void setParent(Node parent) {
            this.parent = parent;
        }

        public Node getParent() {
            return parent;
        }

        public void setLengthStartToNode(double length) {
            lengthStartToNode = length;
        }

        public double getLengthStartToNode() {
            return lengthStartToNode;
        }

        public void setHeuristic(double heuristic) {
            this.heuristic = heuristic;
        }

        public double getHeuristic() {
            return heuristic;
        }

        public void setLengthStartToExitViaNode(double length) {
            lengthStartToExitViaNode = length;
        }

        public double getLengthStartToExitViaNode() {
            return lengthStartToExitViaNode;
        }
    }

    // Calculate distance from node to exit
    private int calculateDistanceToTarget(EscapeState state, Node node) {
        Tile currentTile = node.getTile();
        int currentRow = currentTile.getRow();
        int currentColumn = currentTile.getColumn();
        Tile exitTile = state.getExit().getTile();
        int exitRow = exitTile.getRow();
        int exitColumn = exitTile.getColumn();
        return Math.abs(currentRow - exitRow) + Math.abs(currentColumn - exitColumn);
    }

    private Double calculateHeuristic(EscapeState state, Node node) {
        Tile currentTile = node.getTile();
        int currentRow = currentTile.getRow();
        int currentColumn = currentTile.getColumn();
        Tile exitTile = state.getExit().getTile();
        int exitRow = exitTile.getRow();
        int exitColumn = exitTile.getColumn();
        return (double) (Math.abs(currentRow - exitRow) + Math.abs(currentColumn - exitColumn));
    }
}
