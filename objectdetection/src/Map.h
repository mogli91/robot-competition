/*
 * Map.h
 *
 *  Created on: Jun 1, 2015
 *      Author: snake
 */

#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include "stdio.h"

enum Type{CORRIDOR, ROCKS, ROCKAREA, RAMP, GRASS, HALFDEPOSIT, FULLDEPOSIT};

#define NX 16//number of tiles in x
#define NY 16//number of tiles in y

//structure containing the different tile values for the map

struct Tile
{
	bool containsObstacle;
	bool containsBottle;
	int type;
};

class Map {
public:
	Map();
	~Map();
	void print();
	Tile getTile(int x, int y);
	void setTile(int x, int y, Tile newTile);

private:
	Tile m_map[NX][NY];
};

#endif /* MAP_H_ */
