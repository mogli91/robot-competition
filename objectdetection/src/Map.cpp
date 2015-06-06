/*
 * Map.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: snake
 */

#include "Map.h"

Map::Map() {
	// Creates an empty map
	for(int i = 0; i < NX; i++)
	{
		for(int j = 0; j < NY; j++)
		{
			m_map[i][j].containsBottle = false;
			m_map[i][j].containsObstacle = false;
			m_map[i][j].type = CORRIDOR;
		}
	}

	for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				m_map[i][j].type = HALFDEPOSIT;
			}
		}

	for(int i = 0; i < 2; i++)
		{
			for(int j = 0; j < 2; j++)
			{
				m_map[i][j].type = FULLDEPOSIT;
			}
		}

	//TODO : change the value of the boundaries accordingly
	for(int i = 10; i < NX; i++)
		{
			for(int j = 0; j < 6; j++)
			{
				m_map[i][j].type = GRASS;
			}
		}
	//TODO : change the value of the boundaries accordingly
	for(int i = 0; i < 6; i++)
		{
			for(int j = 10; j < NY; j++)
			{
				m_map[i][j].type = ROCKAREA;
				if(i == 5 || j == 10)
					m_map[i][j].type = ROCKS;
			}
		}
	//TODO : change the value of the boundaries accordingly
	for(int i = 10; i < NX; i++)
		{
			for(int j = 10; j < NY; j++)
			{
				m_map[i][j].type = RAMP;
			}
		}

}

Map::~Map() {
	// TODO Auto-generated destructor stub
}

void Map::print()
{
	for(int j = NY-1; j >= 0; j--)
	{
		for(int i = 0; i < NX; i++)
		{
			switch(m_map[i][j].type)
			{
				case(CORRIDOR):
					std::cout<<".";
				break;
				case(ROCKS):
					std::cout<<"X";
				break;
				case(ROCKAREA):
					std::cout<<"3";
				break;
				case(RAMP):
					std::cout<<"4";
				break;
				case(GRASS):
					std::cout<<"2";
				break;
				case(HALFDEPOSIT):
					std::cout<<"5";
				break;
				case(FULLDEPOSIT):
					std::cout<<"0";
				break;
			}
		}
		std::cout<<std::endl;
	}
}

Tile Map::getTile(int x, int y)
{
	return m_map[x][y];
}

void Map::setTile(int x, int y, Tile newTile)
{
	m_map[x][y] = newTile;
}
