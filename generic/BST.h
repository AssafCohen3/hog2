//
//  BST.h
//  hog2
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef BST_h
#define BST_h

#include "FMMBDOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>



// template <class state>
// struct Comparef {
	// bool operator()(const FMMBDOpenClosedData<state> &i1, const FMMBDOpenClosedData<state> &i2) const
	// {
		// double p1 = i1.g+i1.h;
		// double p2 = i2.g+i2.h;
		// if (fequal(p1, p2))
		// {
			// //return (fgreater(i1.g, i2.g)); // low g-cost over high
			// return (fless(i1.g, i2.g)); // high g-cost over low
		// }
		// return (fgreater(p1, p2)); // low priority over high
	// }
// };

// template <class state>
// struct Compareg{
	// bool operator()(const FMMBDOpenClosedData<state> &i1, const FMMBDOpenClosedData<state> &i2) const
	// {
		// double p1 = i1.g;
		// double p2 = i2.g;
		// return (fgreater(p1, p2)); // low priority over high
	// }
// };

// struct fMMCompare {
	// bool operator()(const FMMBDOpenClosedData<state> &i1, const FMMBDOpenClosedData<state> &i2) const
	// {
		// // FIXME: Note that i2 happens (but isn't guaranteed) to be the uninitialized element,
		// // so we can use the fraction from i1 properly. But, this could be broken.
// //		printf("%f - %f\n", i1.frac, i2.frac);
		// double p1 = std::max((i1.g+i1.h), i1.g/i1.frac+epsilon);
		// double p2 = std::max((i2.g+i2.h), i2.g/i1.frac+epsilon);
		// if (fequal(p1, p2))
		// {
			// return (fgreater(i1.g, i2.g)); // low g-cost over high
			// //return (fless(i1.g/i1.frac, i2.g/i1.frac)); // high g-cost over low
		// }
		// return (fgreater(p1, p2)); // low priority over high
	// }
// };

// template <class state, int epsilon = 0>
// struct improvedHeuristicCompare {
  // int g;
  // improvedHeuristicCompare(int g_value) : g(g_value)
  // {
  // }
	// bool operator()(const FMMBDOpenClosedData<state> &i1, const FMMBDOpenClosedData<state> &i2) const
	// {

		// double p1 = std::max((i1.g+i1.h)-g, i1.g+epsilon);
		// double p2 = std::max((i2.g+i2.h)-g, i2.g+epsilon);
		// if (fequal(p1, p2))
		// {
			// return (fgreater(i1.g, i2.g)); // low g-cost over high
		// }
		// return (fgreater(p1, p2)); // low priority over high
	// }
// };

template <class state, class action, class environment, int epsilon = 0,class priorityQueue = FMMBDOpenClosed<state,fMMCompare<state,epsilon>,Compareg<state>,Comparef<state>,improvedHeuristicCompare<state,epsilon>>>
class BST {
public:
	BST(bool imp = false,bool newTB = false){ forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount(); improved = imp; additionTB = newTB; }
	virtual ~BST() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	virtual const char *GetName() { return "BST"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = uniqueNodesExpanded = 0; }
	
	inline const int GetNumForwardItems() { return forwardQueue.size(); }
	inline const FMMBDOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return backwardQueue.size(); }
	inline const FMMBDOpenClosedData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }
	
	uint64_t GetUniqueNodesExpanded() const { return uniqueNodesExpanded; }
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	//void FullBPMX(uint64_t nodeID, int distance);
	

	void PrintHDist()
	{
		std::vector<uint64_t> d;
		for (auto i = dist.begin(); i != dist.end(); i++)
		{
			if (i->first.first < i->first.second)
			{
				int h = (int)i->first.second;
				if (h >= d.size())
					d.resize(h+1);
				d[h] += i->second;
			}
		}
		printf("BST Dynamic Distribution\n");
		for (int x = 0; x < d.size(); x++)
		{
			if (d[x] != 0)
				printf("%d\t%llu\n", x, d[x]);
		}
	}
	void PrintOpenStats(std::unordered_map<std::pair<double, double>, int>  &s)
	{
		printf("Search distributions: (%s)\n", ((&s)==(&f))?"forward":"backward");
		for (auto i = s.begin(); i != s.end(); i++)
		{
			if (i->second > 0)
			{
				bool ignore = false;
				ignore = (i->first.first+i->first.second >= currentCost);
				printf("%c g: %1.1f h: %1.1f count: %d\n", ignore?'*':' ',
					   i->first.first, i->first.second, i->second);
			}
		}
	}
	
	//	void SetWeight(double w) {weight = w;}
private:
	
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		 while (backwardQueue.Lookup(node).parentID != node){
			thePath.push_back(backwardQueue.Lookup(node).data);
			node = backwardQueue.Lookup(node).parentID;
		}
		thePath.push_back(backwardQueue.Lookup(node).data);
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		while (forwardQueue.Lookup(node).parentID != node) {
			thePath.push_back(forwardQueue.Lookup(node).data);
			node = forwardQueue.Lookup(node).parentID;
		} 
		thePath.push_back(forwardQueue.Lookup(node).data);
	}
	
	void Trim();
  
	void Expand(priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic,
				const state &target,
				std::unordered_map<std::pair<double, double>, int> &count);
	//				std::unordered_map<double, int> &ming,
	//				std::unordered_map<double, int> &minf);
	priorityQueue forwardQueue, backwardQueue;
	state goal, start;
	std::unordered_map<std::pair<double, double>, int> dist;
	std::unordered_map<std::pair<double, double>, int> f, b;
	uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
	state middleNode;
	double currentCost;
	double lastMinForwardG;
	double lastMinBackwardG;
	//double epsilon;
	bool alternate;
  bool additionTB;
  bool expandForward;
	
	std::vector<state> neighbors;
	environment *env;
	Timer t;
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
	
	double oldp1;
	double oldp2;
	bool recheckPath;
	bool improved;
};

template <class state, class action, class environment, int epsilon, class priorityQueue>
void BST<state, action, environment, epsilon, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
															Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	t.StartTimer();
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment, int epsilon, class priorityQueue>
bool BST<state, action, environment, epsilon, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																	 Heuristic<state> *forward, Heuristic<state> *backward,
																	 std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentCost = DBL_MAX;
	forwardQueue.Reset();
	backwardQueue.Reset();
  expandForward = true;
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;
	oldp1 = oldp2 = 0;
	lastMinForwardG = 0;
	lastMinBackwardG = 0;
//	forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
//	backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
	uint64_t i;
	if(!improved){
		i = forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
		backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
	}
	else{
		double hur = std::max(forwardHeuristic->HCost(start, goal),backwardHeuristic->HCost(goal, start));
		i = forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, hur);
		backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, hur);
	}


	f.clear();
	b.clear();
	recheckPath = false;
	return true;
}

template <class state, class action, class environment, int epsilon, class priorityQueue>
bool BST<state, action, environment, epsilon, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
	{
		return true;
	}
	
	//	if (forwardQueue.OpenSize() == 0)
	//		//Expand(backwardQueue, forwardQueue, backwardHeuristic, start, g_b, f_b);
	//		Expand(backwardQueue, forwardQueue, backwardHeuristic, start, b);
	//
	//	if (backwardQueue.OpenSize() == 0)
	//		//Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, g_f, f_f);
	//		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);
	
	uint64_t forward = forwardQueue.Peek(fpriority);
	uint64_t backward = backwardQueue.Peek(fpriority);
	
	const FMMBDOpenClosedData<state> &nextForward = forwardQueue.Lookat(forward);
	const FMMBDOpenClosedData<state> &nextBackward = backwardQueue.Lookat(backward);
	
	double p1 = nextForward.g + nextForward.h;
	double p2 = nextBackward.g + nextBackward.h;

  if (forwardQueue.OpenSize() <= backwardQueue.OpenSize()){
    Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);
  }
  else{
    Expand(backwardQueue, forwardQueue, backwardHeuristic, start, b);
  }

	if (true)
	{
		recheckPath = false;
		bool done = false;
		double minForwardG = DBL_MIN;
		double minForwardF = DBL_MAX;
		double minBackwardG = DBL_MIN;
		double minBackwardF = DBL_MAX;
		if (forwardQueue.OpenSize()>0){
			minForwardG = forwardQueue.PeekAt(gpriority).g;
			minForwardF = forwardQueue.PeekAt(fpriority).g + forwardQueue.PeekAt(fpriority).h;
		}
		if (backwardQueue.OpenSize()>0){
			minBackwardG = backwardQueue.PeekAt(gpriority).g;
			minBackwardF = backwardQueue.PeekAt(fpriority).g + backwardQueue.PeekAt(fpriority).h;
		}
		if (minForwardF == DBL_MAX)
		{
			minForwardF  = currentCost+1;
		}
		if (minBackwardF == DBL_MAX)
		{
			minBackwardF  = currentCost+1;
		}
    //printf("CurrentCost: %f, minForwardG: %f, minForwardF: %f,minBackwardG: %f,minBackwardF: %f\n", currentCost,minForwardG, minForwardF, minBackwardG,minBackwardF);
		if (!fgreater(currentCost, minForwardF))
		{
			//			printf("Terminated on forwardf (%f >= %f)\n", minForwardF, currentCost);
			done = true;
		}
		if (!fgreater(currentCost, minBackwardF))
		{
			//			printf("Terminated on backwardf (%f >= %f)\n", minBackwardF, currentCost);
			done = true;
		}
		if (additionTB && !fgreater(currentCost, minForwardG+minBackwardG+epsilon)) // TODO: epsilon
		{
			//			printf("Terminated on g+g+epsilon (%f+%f+%f >= %f)\n", minForwardG, minBackwardG, epsilon, currentCost);
			done = true;
		}
		/*
		if (!fgreater(currentCost, std::min(forwardP, backwardP)))
		{
			//			printf("Terminated on forwardP/backwardP (min(%f, %f) >= %f)\n", forwardP, backwardP, currentCost);
			done = true;
		}
		*/
		//		if (!fgreater(currentCost, backwardP))
		//		{
		//			printf("Terminated on backwardP\n");
		//			done = true;
		//		}
		// for now, always terminate
		//lastMinBackwardG = minBackwardG;
		//lastMinForwardG = minForwardG;
		if (done)
		{
			//			PrintOpenStats(f);
			//			PrintOpenStats(b);
			
			std::vector<state> pFor, pBack;
			ExtractPathToGoal(middleNode, pBack);
			ExtractPathToStart(middleNode, pFor);
			reverse(pFor.begin(), pFor.end());
			thePath = pFor;
			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
			
			return true;
		}
	}
	return false;
}

template <class state, class action, class environment, int epsilon, class priorityQueue>
void BST<state, action, environment, epsilon, priorityQueue>::Expand(priorityQueue &current,
														   priorityQueue &opposite,
														   Heuristic<state> *heuristic, const state &target,
														   std::unordered_map<std::pair<double, double>, int> &count)
{
  bool toUpdate = false;
  double currentSum = current.GetPrioritySum(epsilon);
	uint64_t nextID = current.Close(fpriority);
  if (currentSum!= current.GetPrioritySum(epsilon)){
    toUpdate = true;
  }
	nodesExpanded++;
	if (current.Lookup(nextID).reopened == false)
		uniqueNodesExpanded++;
		
	// decrease count from parent
	{
		auto &parentData = current.Lookup(nextID);
		count[{parentData.g,parentData.h}]--;
		if (count[{parentData.g,parentData.h}] == 0 && currentCost < DBL_MAX)
		{
			recheckPath = true;
		}
	}
  bool betterSolutionFound = false;
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		uint64_t hash = env->GetStateHash(succ);
		auto loc = current.Lookup(hash, childID);
		auto &childData = current.Lookup(childID);
		auto &parentData = current.Lookup(nextID);
		double edgeCost = env->GCost(parentData.data, succ);
		switch (loc)
		{
			case flocClosed: // ignore
				if (fless(parentData.g+edgeCost, childData.g))
				{
					childData.h = std::max(childData.h, parentData.h-edgeCost);
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
					current.ReOpen(childID);				}
				break;
			case flocOpen: // update cost if needed
			{
				// 1-step BPMX
				parentData.h = std::max(childData.h-edgeCost, parentData.h);
				
				if (fgreater(parentData.h-edgeCost, childData.h))
				{
					count[{childData.g,childData.h}]--;
					dist[{childData.g,childData.h}]--;
					//minf[childData.g+childData.h]--;
					childData.h = parentData.h-edgeCost;
					//minf[childData.g+childData.h]++;
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
				}
				if (fless(parentData.g+edgeCost, childData.g))
				{
					count[{childData.g,childData.h}]--;
					dist[{childData.g,childData.h}]--;
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					current.KeyChanged(childID);
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
					
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(hash, reverseLoc);
					if (loc == flocOpen)
					{
						if (fless(parentData.g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							recheckPath = true;
							// TODO: store current solution
							//printf("Potential updated solution found, cost: %1.2f + %1.2f = %1.2f\n",
							//	   parentData.g+edgeCost,
							//	   opposite.Lookup(reverseLoc).g,
							//	   parentData.g+edgeCost+opposite.Lookup(reverseLoc).g);
							currentCost = parentData.g+edgeCost + opposite.Lookup(reverseLoc).g;
              betterSolutionFound = true;
              //printf("1: currentCost: %f %f %f %f %d\n", currentCost, parentData.g,edgeCost , opposite.Lookup(reverseLoc).g,nodesExpanded);
							middleNode = succ;
							//							PrintOpenStats(f);
							//							PrintOpenStats(b);
						}
					}
				}
			}
				break;
			case flocUnseen:
			{
				double g = parentData.g+edgeCost;
				double h = std::max(heuristic->HCost(succ, target), parentData.h-edgeCost);
				
				// Ignore nodes that don't have lower f-cost than the incumbant solution
				if (!fless(g+h, currentCost))
					break;
				//				ming[g]++;
				//				minf[g+h]++;
				count[{g,h}]++;
				dist[{g,h}]++;
				// 1-step BPMX
        if (parentData.h < h-edgeCost){
          parentData.h = std::max(h-edgeCost, parentData.h);
          //if (improved)
          //  parentData.f = std::max(parentData.h + parentData.g,parentData.f);
          //else
        }

				
				uint64_t i;
				if (!improved){
					i = current.AddOpenNode(succ, // This may invalidate our references
											hash,
											g,
											h,
											nextID);
				}
				else{
          double newh = std::max(h,std::max(opposite.PeekAtG(g).g + epsilon,opposite.PeekAtG(g).g + opposite.PeekAtG(g).h - g));
					i = current.AddOpenNode(succ, // This may invalidate our references
						hash,
						g,
            newh,
						nextID);
          for (auto it = current.Lookup(i).indexMap.begin(); it != current.Lookup(i).indexMap.end(); ++it){
            if (it->second == 0){
              toUpdate = true;
              break;
            }
          }
				}
        //printf("%f ",current.Lookup(i).g);
				// check for solution
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(hash, reverseLoc);
				if (loc == flocOpen)
				{
					if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
					{
						recheckPath = true;
						// TODO: store current solution
						//printf("Potential solution found, cost: %1.2f + %1.2f = %1.2f\n",
						//	   current.Lookup(nextID).g+edgeCost,
						//	   opposite.Lookup(reverseLoc).g,
						//	   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
						currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
            betterSolutionFound = true;
            //printf("2: currentCost: %f, %f, %f, %f, %d\n", currentCost,current.Lookup(nextID).g,edgeCost , opposite.Lookup(reverseLoc).g, nodesExpanded);
						middleNode = succ;
						//						PrintOpenStats(f);
						//						PrintOpenStats(b);
					}
				}
			}
		}
	}
	
	if (toUpdate && improved && current.OpenSize() > 0 && opposite.OpenSize() > 0){
    bool keepGoing = true;
    while (keepGoing){
      keepGoing = false;
      size_t currentSize = current.OpenSize();
      size_t opositeSize = opposite.OpenSize();
      bool loop = false;
      for (uint64_t i = 0; i< opositeSize; i++){
        double myG = opposite.Lookup(opposite.GetOpenItem(i)).g;             
        double maxOther = std::max(opposite.Lookup(opposite.GetOpenItem(i)).h,std::max(current.PeekAtG(myG).g + epsilon,current.PeekAtG(myG).g + current.PeekAtG(myG).h - myG));
        if (opposite.Lookup(opposite.GetOpenItem(i)).h < maxOther){
          double sum = opposite.GetPrioritySum(epsilon);
          opposite.Lookup(opposite.GetOpenItem(i)).h = maxOther;
          opposite.KeyChanged(opposite.GetOpenItem(i));
          if (sum!= opposite.GetPrioritySum(epsilon)){
            loop = true;
          }
        }            
      }   
      if (loop){
        for (uint64_t i = 0; i< currentSize; i++){
          double myG = current.Lookup(current.GetOpenItem(i)).g;             
          double maxOther = std::max(current.Lookup(current.GetOpenItem(i)).h,std::max(opposite.PeekAtG(myG).g + epsilon,opposite.PeekAtG(myG).g + opposite.PeekAtG(myG).h - myG));
          if (current.Lookup(current.GetOpenItem(i)).h < maxOther){
            double sum = current.GetPrioritySum(epsilon);
            current.Lookup(current.GetOpenItem(i)).h = maxOther;
            current.KeyChanged(current.GetOpenItem(i));
            if (sum!= current.GetPrioritySum(epsilon)){
              keepGoing = true;
            }
          }            
        }   
      }     
    }
	}
  
  if (betterSolutionFound){
    Trim();
  }
    
}

template <class state, class action, class environment, int epsilon, class priorityQueue>
uint64_t BST<state, action, environment, epsilon, priorityQueue>::GetNecessaryExpansions() const
{
	uint64_t count = 0;
	for (unsigned int x = 0; x < forwardQueue.size(); x++)
	{
		const FMMBDOpenClosedData<state> &data = forwardQueue.Lookat(x);
		if ((data.where == flocClosed) && (fless(data.g+data.h, currentCost)))
			count++;
	}
	for (unsigned int x = 0; x < backwardQueue.size(); x++)
	{
		const FMMBDOpenClosedData<state> &data = backwardQueue.Lookat(x);
		if ((data.where == flocClosed) && (fless(data.g+data.h, currentCost)))
			count++;
	}
	return count;
}

template <class state, class action, class environment, int epsilon, class priorityQueue>
void BST<state, action, environment, epsilon, priorityQueue>::Trim()
{
	for (unsigned int x = 0; x < forwardQueue.OpenSize(); x++)
	{
		const FMMBDOpenClosedData<state> &data = forwardQueue.Lookat(forwardQueue.GetOpenItem(x));
		if (data.where == flocOpen && fgreatereq(data.g + data.h, currentCost) && (data.data != middleNode)) // and not start or goal
		{
			forwardQueue.Remove(forwardQueue.GetOpenItem(x));
		}
	}

	for (unsigned int x = 0; x < backwardQueue.OpenSize(); x++)
	{
		const FMMBDOpenClosedData<state> &data = backwardQueue.Lookat(backwardQueue.GetOpenItem(x));
		if (data.where == flocOpen && fgreatereq(data.g + data.h, currentCost) && (data.data != middleNode)) // and not start or goal
		{
			backwardQueue.Remove(backwardQueue.GetOpenItem(x));
		}
	}

}



#endif 
