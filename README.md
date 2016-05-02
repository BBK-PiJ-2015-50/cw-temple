# cw-temple

PiJ Coursework 4: George Osborne And The Temple Of Gloom

Final test: ran 250,000 times with no apparent issues - Average Score = 6,985
- all the final tests have averaged around 7,000.

All my changes have been to the Explorer class in Student.

For the escape() method the A* algorithm was implemented to find the shortest path.
The supplied PriorityQueueImpl was used for storing "Open" nodes, as part of this algorithm.

The gold-hunting is fairly basic. Given more time I would have developed a better approach...selecting paths that include the most gold, taking account of edge weights, time remaining, distance back to a known short path, etc.

