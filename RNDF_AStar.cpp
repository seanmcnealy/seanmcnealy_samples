int RoadNetwork::replanFrom(double lat, double lon) {
	Node *start, *goal;
	vector<Node *> openList, closedList;
	
	if (checkpoints.size() == 0) {
		cError("replanFrom: No checkpoints loaded!");
		return -1;
	}
	
	// BUG/TODO: fix how this is done, make sure we don't get a node from a different lane!
	cLog("finding closest Node to: <%.6f, %.6f>\n", lat, lon);
	if ( ! (start = closestNode(lat, lon)) ) {
		cError("replanFrom: couldn't find closest node!");
		return -1;
	}
	
	if (nextCheckpoint >= checkpoints.size()) {
		// TODO: properly handle this
		cError("replanFrom: No need to replan, we're at the final checkpoint!");
		return -1;
	}
	
	goal = checkpoints[checkpoints.size()-1];
	
	if (start == goal) {
		// TODO: handle this properly... add it to missionpath list and get out of here
		cError("replanFrom: FIXME! goal = start!\n");
	}
	
	missionpoints.clear(); // reset array
	
	missionpoints.push_back(start);
	
	cLog("replanning:\n\tstart\t%d.%d.%d <%.6f, %.6f>\n\tfinish\t%d.%d.%d <%.6f, %.6f>\n",
										NODE_HIGH(start), NODE_MED(start), NODE_LOW(start),
										NODE_LAT(start), NODE_LON(start),
										NODE_HIGH(goal), NODE_MED(goal), NODE_LOW(goal),
										NODE_LAT(goal), NODE_LON(goal));
	
	Node::size_type currCheckpoint = nextCheckpoint;
	while (currCheckpoint < checkpoints.size()) {
		// start is already set, we will change it at the end of this while loop
		goal = checkpoints[currCheckpoint]; // we'll use this as the start next time around
		
		cLog("generating intermediate path:"
			"\n\tcheckpoint %d\t%d.%d.%d <%.6f, %.6f>"
			"\n\tcheckpoint %d\t%d.%d.%d <%.6f, %.6f>\n", currCheckpoint,
														  NODE_HIGH(start), NODE_MED(start), NODE_LOW(start),
											              NODE_LAT(start), NODE_LON(start),
														  currCheckpoint + 1,
											              NODE_HIGH(goal), NODE_MED(goal), NODE_LOW(goal),
											              NODE_LAT(goal), NODE_LON(goal));
											             
		
		// Commence A*!

		bool foundPath = false;
		openList.push_back(start);
		make_heap(openList.begin(), openList.end(), node_compare);
		
		// TODO: Take this out
		int safety = 5000; // allow this many iterations max, protect us from an infinite loop

		while (!openList.empty()) {
			pop_heap(openList.begin(), openList.end(), node_compare);
			Node *current = openList.back();
			openList.pop_back();
			
#ifndef NO_UI
			if (current->parent != NULL) {
				UI::addConnection(current->parent, current, UI_BLUE, 5, -2);
				// UI::update();
				// usleep(250000);
			}
#endif

			// the following couple of lines must be executed in this order,
			// this saves us from having to put the goal node on the list and reseting it
			current->onList = false;
			if (current == goal) {
				foundPath = true;
				break;
			}
			
			current->visited = true;
			closedList.push_back(current);

			if (--safety < 0) break;

			cDebug(4, "Expanding %d.%d.%d\n", NODE_HIGH(current), NODE_MED(current), NODE_LOW(current));
			cDebug(4, "openList.size: %ld closedList.size: %ld\n", openList.size(), closedList.size());

			for (Node::size_type i=0; i < current->children.size(); i++) {
				Node *child = current->children[i];
				
				// WARNING: DO NOT DELETE ANY OF THESE COMMENTS!!!!!!!!
				
				// to compensate for possibly screwed up gVal values whereby they result in a path
				// that is actually lower in cost than that from 'h' (as the crow flies)
				// we will not ignore nodes on the closed list immediately
				// TODO: see if we can guarantee the total g to be greater than h
				//if (child->visited) continue;

				// check if it's already on the open list, or (new check) screwed up gVals
				if (child->onList || child->visited) {
					// check if g score is better by going through this node
					if ((current->gVal + Node::calculateG(current, child)) < child->gVal) {
						cLog("\n");
						NODE_DISP("\nfound better path for node: ", child);
						NODE_DISP("old parent: ", child->parent);
						NODE_DISP("new parent: ", current);
						
						// compensate for fucked up gVal values
						if (child->visited) {
							openList.push_back(child);
							child->visited = false;
							child->onList = true;
						}
						
						child->parent = current;
						Node::calculateNextFGH(current, child, goal);
						make_heap(openList.begin(), openList.end(), node_compare); // resort heap
					} else {
						// new check to compensate for the fact that
						// now we do not ignore things on the closed list (child->parent could be NULL)
						if (child->onList) {
							cLog("\n");
							NODE_DISP("This node was passed up: ", current);
							NODE_DISP("In favor of already connected node: ", child->parent);
							NODE_DISP("To child: ", child);
						}
					}
				} else {
					child->parent = current;
					child->onList = true;
					Node::calculateNextFGH(current, child, goal);
					openList.push_back(child);
					push_heap(openList.begin(), openList.end(), node_compare);
				}
			}

		}
		// reset everything
		for (Node::size_type i=0; i < closedList.size(); i++) {
			closedList[i]->visited = false;
		}
		for (Node::size_type i=0; i < openList.size(); i++) {
			openList[i]->onList = false;
		}
		closedList.clear();
		openList.clear();
		
		if (foundPath) {
			cLog("Found path!\n");
			Node *current = goal;
			Node::size_type startIter = missionpoints.size();
			
			while (current != start) {
#ifndef NO_UI
				UI::addConnection(current, current->parent, UI_RED, 5, -1);
#endif
				missionpoints.push_back(current);
				current = current->parent;
			}
			
			// reverse, first time this is run missionpoints contains only start (closest) node
			for (Node::size_type i = missionpoints.size() - 1; i > startIter; --i, ++startIter) {
				Node *temp = missionpoints[i];
				missionpoints[i] = missionpoints[startIter];
				missionpoints[startIter] = temp;
			}
		} else {
			cError("replanFrom: Could not find path :-(\n");
			return -1;
		}
		
		start = checkpoints[currCheckpoint++];
	}
	
	// fill up missionpath array with checkpoints, exit and entry points
	
	missionpath.clear();
	vector<Node*>::iterator checkpoint = checkpoints.begin();
	for (Node::size_type i=0; i<missionpoints.size(); i++) {
		// check for exit points before checking for checkpoints because of the case
		// where a checkpoint could also be an exit point, we would then miss the entry point
		if (missionpoints[i]->flags & Node::NODE_EXIT) {
			if (i + 1 < missionpoints.size()) {
				// add entry/exit points only if they are exit points that exit
				// onto a different lane or segment
				if (missionpoints[i+1]->flags & Node::NODE_ENTRY
					&& (NODE_HIGH(missionpoints[i]) != NODE_HIGH(missionpoints[i+1])
						|| NODE_MED(missionpoints[i]) != NODE_MED(missionpoints[i+1])))
				{
					//NODE_DISP("Mission point: ", missionpoints[i]);
					missionpath.push_back(missionpoints[i++]);
					//NODE_DISP("Mission point: ", missionpoints[i]);
					missionpath.push_back(missionpoints[i]);
				}
			}
		} else if (missionpoints[i]->flags & Node::NODE_CHECKPOINT) {
			// only add checkpoints in the order they must be hit
			if (checkpoint != checkpoints.end() && *checkpoint == missionpoints[i]) {
				missionpath.push_back(missionpoints[i]);
				//NODE_DISP("Mission point: ", missionpoints[i]);
				checkpoint++;
			}
		}
	}
	
	return 0;
}

mRoadNetwork RoadNetwork::roadNetworkMessage() {
	mRoadNetwork roadNetworkStruct;
	mSegment segments;
	mZone zones;
	mMissionPoint missionPoints;
	SpeedLimit sl;
	int i;
	unsigned int ui;

	vector<mConnection> connections;
	
	_box->searchProximity(nodes);
	
	ProximityBox::Iterator segmentsIter = _box->proximateSegments();
	set<Zone*>& proximateZones = _box->proximateZones();
	
	roadNetworkStruct = (mRoadNetwork)malloc( sizeof(RoadNetworkStruct) );
	
	roadNetworkStruct->numberSegments = segmentsIter.count();
	//cLog("%d proximate segments\n", roadNetworkStruct->numberSegments);
	
	segments = (mSegment)malloc( sizeof(SegmentStruct) * roadNetworkStruct->numberSegments );
	for(i = 0; segmentsIter; ++i, ++segmentsIter){
		segmentToStruct( &(segments[i]), segmentsIter.childIterator(), connections, _box );
		sl = mdf->getSpeedLimit(NODE_HIGH(*segmentsIter) - 1);
		segments[i].speedMinimum = sl.mMinSpeed * MPH_TO_METERSPERSECOND;
		segments[i].speedLimit = sl.mMaxSpeed * MPH_TO_METERSPERSECOND;
	}
	roadNetworkStruct->segments = segments;
	
	roadNetworkStruct->numberZones = proximateZones.size();
	//if (roadNetworkStruct->numberZones) {
		zones = (mZone)malloc( sizeof(ZoneStruct) * roadNetworkStruct->numberZones );
		//cLog("%d proximate zones\n", roadNetworkStruct->numberZones);
		if (roadNetworkStruct->numberZones) {
			set<Zone*>::iterator zoneIter = proximateZones.begin();
			for(i = 0; i < roadNetworkStruct->numberZones && zoneIter != proximateZones.end(); ++i, ++zoneIter) {
				Zone &zone = **zoneIter;
				zoneToStruct( &(zones[i]), zone, connections, _box );
				sl = mdf->getSpeedLimit(zone.getZoneID() - 1);
				zones[i].speedMinimum = sl.mMinSpeed * MPH_TO_METERSPERSECOND;
				zones[i].speedLimit = sl.mMaxSpeed * MPH_TO_METERSPERSECOND;
			}
		}
		roadNetworkStruct->zones = zones;
	//}
	roadNetworkStruct->numberConnections = connections.size();
	//if (roadNetworkStruct->numberConnections) {
		roadNetworkStruct->connections = (mConnection)malloc( sizeof(ConnectionStruct) * roadNetworkStruct->numberConnections );
	
		for(ui=0; ui<connections.size(); ui++){
			memcpy( &(roadNetworkStruct->connections[ui]), connections[ui], sizeof(ConnectionStruct) );
			free(connections[ui]);
		}
	//}
	roadNetworkStruct->numberMissionPoints = missionpath.size() - nextCheckpoint;
	//if (roadNetworkStruct->numberMissionPoints) {
		missionPoints = (mMissionPoint)malloc( sizeof(MissionPointStruct) * roadNetworkStruct->numberMissionPoints );
		for(i = nextCheckpoint; i < roadNetworkStruct->numberMissionPoints; i++){
			missionPointToStruct( &(missionPoints[i]), i, missionpath[i]->data);
		}
		roadNetworkStruct->missionPoints = missionPoints;
	//}	
	return roadNetworkStruct;
}
