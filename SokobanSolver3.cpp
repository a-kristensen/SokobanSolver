// SokobanSolver.cpp : Defines the entry point for the console application.

//only for windows
#include "stdafx.h"


#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <unordered_map>
#include <list>
#include <queue>
#include <time.h>
#include <algorithm> 

using namespace std;

#define _CRT_SECURE_NO_WARNINGS

#define WALL			'X'
#define DIAMOND			'J'
#define DESTINATION 	'G'
#define MAN				'M'
#define EMPTY_FIELD 	'.'

#define DRIVEAHEAD	"DRIVE_AHEAD"
#define TURN90L		"TURN_LEFT"
#define TURN90R		"TURN_RIGHT"
#define BACK		"BACK_UP"


#define INFORMED
//#define BREADTH
//#define DEPTH


struct xy
{
	char x;
	char y;

	xy& operator+=(const xy& a)
	{
		x += a.x;
		y += a.y;
		return *this;
	}
	bool operator==(const xy& a) const
	{
		return (x == a.x) && (y == a.y);
	}
	bool operator<(const xy& rhs) const
	{
		if (x == rhs.x)
			return y < rhs.y;

		return x < rhs.x;
	}
	xy operator*(char i)
	{
		return {x*i, y*i};
	}
};

struct value
{
	uint32_t lvl;
	uint32_t heuristic;
	size_t hash_p;
};

struct state
{
	xy man;			//destination of man
	vector<xy> dia;	//destiantion of diamonds


	bool operator==(const state& a) const
	{
		for(unsigned char i=0; i<dia.size(); i++)
			for (unsigned char ii = 0; ii < dia.size(); ii++)
				if (a.dia[ii] == dia[i])
					break;						//diamond found, continue loop
				else if (ii == dia.size() - 1)
					return false;				//diamond not found		

		return man == a.man;
	}

	state& operator=(const state& s)
	{
		man.x = s.man.x;
		man.y = s.man.y;

		dia.clear();
		for (unsigned int i = 0; i < s.dia.size(); i++)
			dia.push_back(s.dia[i]);

		return *this;
	}

};

struct diaDest
{
	xy d;
	unsigned char** heu;
	bool flag;
};

template<>
struct hash<state>
{
	size_t operator()(const state &s) const
	{
		//return 64 bits hash
		//each 8 bits is a x,y coordinate

		size_t t = 0;
		unsigned char b = 4;

		t |= s.man.x;
		t |= s.man.y << b;

		for (unsigned char i = 0; i < s.dia.size(); i++)
		{
			b += 4;
			t |= (uint64_t)s.dia[i].x << b;
			b += 4;
			t |= (uint64_t)s.dia[i].y << b;
		}

		return t;
	}
}; 

class SokobanSolver {

private:
	int wdth = 0, hght = 0, dmndCnt = 0, dstCnt = 0;

	enum SEARCH_STATE
	{
		CONTINUE_SEARCH = 0,
		GOAL_REACHED = 1,
		OPENLIST_EMPTY = 2
	};

	#ifdef INFORMED
	priority_queue<pair<state, value>> pq_openList;
	#else
	list<pair<state, value>> openList;
	#endif

	unordered_map<state, value> closedSet;
	SEARCH_STATE searchState = CONTINUE_SEARCH;
	time_t tmr;
	vector<state> solution;

	char **map;
	bool **invalidDiamondPositions;
	bool **bool_Goals;

	vector<diaDest> Goals;


	ifstream infile;
	ofstream outfile;
	char str[100];
	string fileloc;

	inline void openNode();
	inline bool GoalReached(const pair<state, value>* const);
	inline bool isFldDmnd(xy d, char* c, const pair<state, value>* const);
	inline bool alreadyVisited(const pair<state, value>* const);
	inline bool isDiamondInCorner(const xy* const d);
	void printSolution();


public:
	SokobanSolver();

	pair<state, value> initState;
	static const xy mv[4];
	static const xy corner[4][2];

	bool scanFile(string);
	void printState(state);
	void printMap();
	void startSearch();
	void printCommands();
	void printStat();

	unsigned int calc_heuristic(const state* const);
};


bool operator<(const pair<state, value>& lhs, const pair<state, value>& rhs)
{
	return lhs.second.heuristic > rhs.second.heuristic;
}


const xy SokobanSolver::mv[4] =
{
	{ 0, -1 }, //down
	{ 1, 0 },  //right
	{ 0, 1 },  //up
	{ -1, 0 }  //left
};

const xy SokobanSolver::corner[4][2] =
{
	{ { -1, 0 },{ 0, 1 } },	//left-up
	{ { 0, 1 },{ 1, 0 } },	//up-right
	{ { 0, -1 },{ 1, 0 } },	//down-right
	{ { -1, 0 },{ 0, -1 } }	//left-down
};


SokobanSolver::SokobanSolver()
{
	//closedSet.reserve(16777215);
}

bool SokobanSolver::scanFile(string s)
{
	infile.open(s);

	if (infile.fail())
	{
		cout << "failed to open file" << endl;
		return false;
	}

	//get height,width,number
	infile.getline(str, 100);
	sscanf_s(str, "%d %d %d", &wdth, &hght, &dmndCnt);


	
	//put goals in a bool map
	bool_Goals = new bool *[wdth];
	for (int i = 0; i < wdth; i++)
		bool_Goals[i] = new bool[hght] { false };

	//create the map in memory
	map = new char *[wdth];
	for (int i = 0; i < wdth; i++)
		map[i] = new char[hght] {0};


	for (char h = hght - 1; h >= 0; h--)
	{
		memset(str, 0, 100);
		infile.getline(str, 100);


		for (char w = 0; w < wdth; w++)
		{
			//put map into array
			map[w][h] = str[w];
			
			//save diamond destinations
			if (str[w] == DESTINATION)
			{
				diaDest diadst;
				diadst.d = { w, h };
				bool_Goals[w][h] = true;

				//calculate heuristics for current diamond destination
				diadst.heu = new unsigned char *[wdth];;
				for (unsigned char i = 0; i < wdth; i++)
				{
					diadst.heu[i] = new unsigned char[hght] {0};
					for (unsigned char x = 0; x < hght; x++)
					{
						//distance from current dia destination to any field
						diadst.heu[i][x] = abs(w - i) + abs(h - x);
					}
				}

				Goals.push_back(diadst);
			}

			//save initial diamond destination
			if (str[w] == DIAMOND)
			{
				map[w][h] = EMPTY_FIELD;
				initState.first.dia.push_back({ w, h });
			}

			//save initial man destination
			if (str[w] == MAN)
			{
				map[w][h] = EMPTY_FIELD;
				initState.first.man = { w, h };
			}
		}
	}


	//create a table over invalid positions for diamonds
	invalidDiamondPositions = new bool *[wdth];
	for (int i = 0; i < wdth; i++)
		invalidDiamondPositions[i] = new bool[hght] {false};


	//check which position are in a corner and not a goal
	for (unsigned char w = 1; w < wdth-1; w++)
	{
		for (unsigned char h = 1; h < hght-1; h++)
		{
			for (unsigned char c = 0; c < 4; c++)
			{
				if ((map[w + corner[c][0].x][h + corner[c][0].y] == WALL) &&
					(map[w + corner[c][1].x][h + corner[c][1].y] == WALL) &&
					(map[w][h] != DESTINATION) )
				{
					invalidDiamondPositions[w][h] = true;
				}
			}
		}
	}



	infile.close();
	return true;
}

void SokobanSolver::printMap()
{
	//print map
	for (int h = hght - 1; h >= 0; h--)
	{
		cout << endl << h << "| ";
		for (int w = 0; w < wdth; w++)
		{
			cout << map[w][h];
		}
	}
	cout << endl;
	for (int w = 0; w < wdth + 3; w++)
		cout << "_";
	cout << endl << "   ";
	for (int w = 0; w < wdth; w++)
		cout << w;
}

void SokobanSolver::printState(state s)
{
	//initial man position
	cout << endl << "man: (" << (int)s.man.x << "," << (int)s.man.y << ")";

	//inital diamond postions
	for (unsigned int d = 0; d < s.dia.size(); d++)
		cout << endl << "dia" << d << ": (" << (int)s.dia[d].x << "," << (int)s.dia[d].y << ")";

	cout << endl;
}

void SokobanSolver::startSearch()
{
	time(&tmr);

	//copy inital state into openlist and start search
	initState.second.lvl = 0;
	initState.second.hash_p = 0;

	sort(initState.first.dia.begin(), initState.first.dia.end());

	#ifdef INFORMED
	pq_openList.push(initState);
	#else
	openList.push_back(initState);
	#endif
	

	while (searchState == CONTINUE_SEARCH)
	{
		openNode();
	}

	switch (searchState)
	{
	case OPENLIST_EMPTY:
		cout << endl << "openlist empty";
		break;
	case GOAL_REACHED:
		tmr = time(NULL) - tmr;
		cout << endl << "Goal reached" << endl << endl;
		printSolution();
		break;
	}

	cout << "\n\n";
	printStat();
}

void SokobanSolver::printSolution()
{
	unordered_map<state, value>::iterator it;

	#ifdef INFORMED
	state key = pq_openList.top().first;
	#else
	state key = openList.begin()->first;
	#endif

	unsigned char b = 8;

	#ifdef INFORMED
	closedSet.insert(pq_openList.top());
	#else
	closedSet.insert(*openList.begin());
	#endif

	do
	{
		it = closedSet.find(key);
		if (it == closedSet.end())
			return;
		
		//printState(it->first);

		key.man.x = it->second.hash_p & 0x0F;
		key.man.y = (it->second.hash_p >> 4) & 0x0F;
		b = 8;
		for (unsigned char i = 0; i < key.dia.size(); i++)
		{
			key.dia[i].x = (it->second.hash_p >> b) & 0x0F;
			b += 4;
			key.dia[i].y = (it->second.hash_p >> b) & 0x0F;
			b += 4;
		}

		solution.push_back(it->first);
	}
	while (it->second.hash_p != 0);

	//reverse order in solution vector
	std::reverse(solution.begin(), solution.end());

	//print steps from start to end
	for (vector<state>::iterator is = solution.begin(); is != solution.end(); ++is)
		printState(*is);

	cout << endl;
	printCommands();
	cout << endl;

	printf("\nsteps: %zi\n", solution.size());
	printf("time: %i:%i:%i\n", (int)tmr / 3600, (int)(tmr % 3600) / 60, (int)tmr % 60);

	return;
}


void SokobanSolver::printCommands()
{
	enum DIR
	{
		UP = 0,
		DOWN = 1,
		RIGHT = 2,
		LEFT = 3
	};

	enum ACTION
	{
		NO_TURN = 0,
		TURN_LEFT = 1,
		TURN_RIGHT = 2,
		TURN_180 = 3
	};

	//turn action relative to robot
	const ACTION turnAction[4][4] =
	{
		//DIR	face_up		  face_down	  face_right  face_left
		{ NO_TURN,	  TURN_180,   TURN_LEFT,  TURN_RIGHT },	//move_UP
		{ TURN_180,   NO_TURN,    TURN_RIGHT, TURN_LEFT },	//move_DOWN
		{ TURN_RIGHT, TURN_LEFT,  NO_TURN,    TURN_180 },	//move_RIGHT
		{ TURN_LEFT,  TURN_RIGHT, TURN_180,   NO_TURN }		//Move_LEFT
	};
	//        dir/turn  
	const DIR NewDir[4][4] =
	{
		//DIR	face_up  face_down  face_right  face_left
		{ UP,	    DOWN,    RIGHT,     LEFT },		//NO_TURN
		{ LEFT,     LEFT,    UP,	    DOWN },		//TURN_LEFT
		{ RIGHT,    RIGHT,   DOWN,      UP },		//TURN_RIGHT
		{ DOWN,     UP,	     LEFT,      RIGHT }		//TURN_180
	};

	xy d;
	DIR moveAction;
	DIR Dir = UP;
	bool dia;
	char c;

	for (vector<state>::iterator is = solution.begin(); is != --solution.end(); ++is)
	{
		//next coordinate minus current
		d.x = (is + 1)->man.x - is->man.x;
		d.y = (is + 1)->man.y - is->man.y;

		if (d.x == -1)
		{
			c = 'l';
			moveAction = LEFT;
		}
		else if (d.x == 1)
		{
			c = 'r';
			moveAction = RIGHT;
		}
		else if (d.y == -1)
		{
			c = 'd';
			moveAction = DOWN;
		}
		else if (d.y == 1)
		{
			c = 'u';
			moveAction = UP;
		}

		dia = false;
		for (char i = 0; i < is->dia.size(); i++)
		{
			//is the next coodinate of the man the same as the current coordinate for a diamond
			if ((is + 1)->man == is->dia[i])
			{
				c -= 32;
				//cout << "_DIA";
				break;
			}
		}

		cout << c;

	}
}

void SokobanSolver::printStat()
{
	cout << endl;

	#ifdef INFORMED
	cout << "openList:         " << pq_openList.size()			 << endl;
	#else
	cout << "openList:         " << openList.size()				 << endl;
	#endif

	cout << "closedSet:        " << closedSet.size()			 << endl;
	printf( "bucket count/max: %zi / %zi\n", closedSet.bucket_count(), closedSet.max_bucket_count());
	printf( "load fact/max:    %f / %f\n", closedSet.load_factor(), closedSet.max_load_factor());
	cout << "max size:         " << closedSet.max_size()		 << endl;

	printf("time used:	   %i:%i:%i\n", (int)(time(NULL) - tmr) / 3600, (int)((time(NULL) - tmr) % 3600) / 60, (int)(time(NULL) - tmr) % 60);

	cout << flush;
}



unsigned int SokobanSolver::calc_heuristic(const state* const s)
{
	unsigned int ret = 0;
	
	//reset flags
	for (auto it = Goals.begin(); it != Goals.end(); ++it)
		it->flag = false;


	//iterate for all diamonds
	for (auto it_dia = s->dia.cbegin(); it_dia != s->dia.cend(); ++it_dia)
	{
		std::vector<diaDest>::iterator it_min;

		//find first unflagged dia-destination
		for (auto it_dst = Goals.begin(); it_dst != Goals.end(); ++it_dst)
		{
			if (!it_dst->flag)
			{
				it_min = it_dst;
				break;
			}
		}


		//find smallest heu (distance) amongst unflagged
		for (auto it_dst = it_min; it_dst != Goals.end(); ++it_dst)
		{
			if (!it_dst->flag)
			{
				if (it_dst->heu[it_dia->x][it_dia->y] < it_min->heu[it_dia->x][it_dia->y])
					it_min = it_dst;
			}
		}

		//closest dia-destination found
		//now sum value and set flag
		it_min->flag = true;
		ret += it_min->heu[it_dia->x][it_dia->y];
	}

	return ret;
}


bool SokobanSolver::isDiamondInCorner(const xy* const d)
{
	return invalidDiamondPositions[d->x][d->y];
}

bool SokobanSolver::GoalReached(const pair<state, value>* const head)
{
	for (auto it = head->first.dia.cbegin(); it != head->first.dia.cend(); ++it)
	{
		if (!bool_Goals[it->x][it->y])
			return false;
	}
	
	return true;
}

bool SokobanSolver::alreadyVisited(const pair<state, value>* const st)
{
	auto f = closedSet.find(st->first);
	if (f == closedSet.end())
		return false;

	return true;
}

bool SokobanSolver::isFldDmnd(xy d, char* c, const pair<state, value>* const head)
{
	for (unsigned char i = 0; i < head->first.dia.size(); i++)
	{
		if (head->first.dia[i] == d)
		{
			*c = i;			//return position of diamond
			return true;
		}
	}

	return false;
}

void SokobanSolver::openNode()
{
	pair<state, value> child;
	pair<state, value> head;
	char diaNo = 0;
	static time_t tmr = 0;


	//if openlist is empty, fail
	#ifdef INFORMED
	if(pq_openList.empty())
	#else
	if (openList.empty())
	#endif	
	{
		searchState = OPENLIST_EMPTY;
		return;
	}
	#ifdef INFORMED
	else if ( alreadyVisited( &pq_openList.top() ))
	#else
	else if (alreadyVisited(&(*openList.begin())))
	#endif
	{
		//the head has already been visited. Discard.

		#ifdef INFORMED
		pq_openList.pop();
		#else
		openList.erase(openList.begin());
		#endif
		

		return;
	}

	#ifdef INFORMED
	head = pq_openList.top();
	pq_openList.pop();
	#else
	head = *openList.begin();
	openList.erase(openList.begin());
	#endif



	//if head is goal, success
	if (GoalReached(&head))
	{
		#ifdef INFORMED
		pq_openList.push(head);
		#else
		openList.insert(openList.begin(), head);
		#endif


		searchState = GOAL_REACHED;
		return;
	}
	//expand children of head
	else
	{
		

		if (time(NULL) > tmr)
		{
			time(&tmr);
			tmr += 5;

			cout << endl;
			printStat();
		}


		//check for possible moves to left/up/right/down
		for (char i = 0; i < 4; i++)
		{
			switch (map[head.first.man.x + mv[i].x][head.first.man.y + mv[i].y])
			{
			case DESTINATION:
			case EMPTY_FIELD:
				//empty field on map, in front of man


				//is there a diamond on the field?
				if (isFldDmnd({ head.first.man.x + mv[i].x, head.first.man.y + mv[i].y }, &diaNo, &head))
				{
					//is the field in front of the diamond empty?
					switch (map[head.first.man.x + 2 * mv[i].x][head.first.man.y + 2 * mv[i].y])
					{
					case DESTINATION:
					case EMPTY_FIELD:
						//no walls blocking diamond
						
						//is the diamond blocked by another diamond?
						if (isFldDmnd({ head.first.man.x + 2 * mv[i].x , head.first.man.y + 2 * mv[i].y }, &diaNo, &head))
							continue; //for
						
						//create a new state for man and diamond
						child = head;
						child.first.man += mv[i];			//move man 1 field
						child.first.dia[diaNo] += mv[i];	//move diamond 1 field



						//discard if diamond is on invalid position
						if (isDiamondInCorner(&child.first.dia[diaNo]))
							continue; //for

						//diamonds are not unique, so sort after their position
						sort(child.first.dia.begin(), child.first.dia.end());

						//is state already visited?
						if (alreadyVisited(&child))
							continue; //for

						break; //switch
					default:
						continue; //field in front of diamond is blocked
					}
				}
				else //moving man to empty field (no diamond)
				{
					//no diamond on empty field, make new child state
					child = head;
					child.first.man += mv[i];      //move man 1 field

					//is state already visited?
					if (alreadyVisited(&child))
						continue; //for
				}

				//only ends here if a child is to be added

				child.second.lvl++;
				child.second.hash_p = closedSet.hash_function()(head.first);

				#ifdef INFORMED
				child.second.heuristic = calc_heuristic(&child.first) + child.second.lvl;
				//informed search--------------------------------
				pq_openList.push(child);
				#else

					#ifdef DEPTH

					//depth first -----------------------------------
					openList.insert(openList.begin(), child);
					#elif defined BREADTH

					//breadth first----------------------------------
					openList.push_back(child);
					#else
					#error "missing define"
					#endif
				#endif



				break; //switch
			}
		}

		
		//put the head into closedset
		closedSet.insert(head);
	}
}

int main(int argc, char *argv[])
{
	SokobanSolver s;


	if(argc <= 1)
		//s.scanFile("ex_map.txt");
		s.scanFile("2017-competation-map.txt");
	else
		s.scanFile(argv[1]);

	s.printMap();
	s.printState((s.initState.first));

	s.startSearch();

	cout << endl << endl;


	cout << "press any key";
	cin.ignore().get(); //Pause Command for Linux Terminal

	//system("pause");

    return 0;
}
