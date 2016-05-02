package student;

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
                // Record route in case we need to retrace our steps
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

        //TODO: Implement A* algorithm for finding a path through the cavern

        PriorityQueue<Node> openNodes = new PriorityQueueImpl<>();
        HashMap<Node, NodeInformation> nodeMap = new HashMap<>();
        Collection<Node> vertices = state.getVertices();
        Node currentNode;
        Node start = state.getCurrentNode();

        for (Node vertex : vertices) {
            nodeMap.put(vertex, new NodeInformation());
        }
        System.out.println("Maze size = " + nodeMap.size());

        double priority = 0;
        addToOpenNodesQueue(openNodes, nodeMap, start, priority);
        currentNode = start;
        while (currentNode != state.getExit()) {
            currentNode = openNodes.poll();
            nodeMap.get(currentNode).setClosed();
            if (currentNode != state.getExit()) {
                for (Edge edge : currentNode.getExits()) {
                    Node neighbour = edge.getDest();
                    if (!nodeMap.get(neighbour).getClosed()) {
                        if (!nodeMap.get(neighbour).getOpen()) {
                            nodeMap.get(neighbour).setParent(currentNode);
                            double lengthStartToParent = nodeMap.get(nodeMap.get(neighbour)
                                    .getParent()).getLengthStartToNode();
                            double lengthStartToNeighbour = lengthStartToParent + edge.length();
                            double heuristic = calculateHeuristic(state, neighbour);
                            nodeMap.get(neighbour).setHeuristic(heuristic);
                            double lengthStartToExitViaNeighbour = lengthStartToNeighbour + heuristic;
                            addToOpenNodesQueue(openNodes, nodeMap, neighbour, lengthStartToExitViaNeighbour);
                        }
                    }
                }
            }
        }
        // Save path
        ArrayList<Node> path = new ArrayList<>();
        Node tempNode = state.getExit();
        while (tempNode != start) {
            path.add(tempNode);
            tempNode = nodeMap.get(tempNode).getParent();
        }
        // Reverse path so it will run from start to exit
        Collections.reverse(path);
        //path.remove(0);
        for (Node node : path) {
            state.moveTo(node);
            if (node.getTile().getGold() > 0) {
                state.pickUpGold();
            }
        }
    }

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
        double lengthStartToNode;
        double heuristic;
        double lengthStartToExitViaNode;

        public NodeInformation() {

        }

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
    }

    public void escapeBasic(EscapeState state) {

        //TODO: Adapt method so George always escapes before time runs out
        // Basic implementation - adapting explore() method
        // Often takes too many steps

        Set<Long> visitedNodes = new LinkedHashSet<>();
        Deque<Node> route = new LinkedList();
        int shortestDistance;
        Node nextNode;
        Collection<Node> neighbours;
        Boolean noUnvisitedNeighbourFound;

        // Stop when the orb is reached, ie. distance is zero
        while (calculateDistanceToTarget(state, state.getCurrentNode()) > 0) {
            visitedNodes.add(state.getCurrentNode().getId());
            neighbours = state.getCurrentNode().getNeighbours();
            // Initialise nextTile before checking for unvisited tiles
            nextNode = null;
            // Initialise shortestDistance before looking for tile closest to orb
            shortestDistance = Integer.MAX_VALUE;
            noUnvisitedNeighbourFound = true;
            // Look for neighbouring tiles not yet visited
            for (Node node : neighbours) {
                if (!visitedNodes.contains(node.getId())) {
                    noUnvisitedNeighbourFound = false;
                    // Look for neighbouring tile closest to orb
                    if (calculateDistanceToTarget(state, node) < shortestDistance) {
                        shortestDistance = calculateDistanceToTarget(state, node);
                        nextNode = node;
                    }
                }
            }
            // Go back if no unvisited neighbouring tiles are found
            if (noUnvisitedNeighbourFound) {
                nextNode = route.pop();
            } else {
                // Record route in case we need to retrace our steps
                route.push(state.getCurrentNode());
            }
            state.moveTo(nextNode);
            if (nextNode.getTile().getGold() > 0) {
                state.pickUpGold();
            }
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
