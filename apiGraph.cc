#include "glovehttpserver.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include "json.hpp"

#include <bits/stdc++.h>
#include "planning/graph.h"
#include "planning/planning_graph.h"
#include "planning/util.h"
#include <exception>
#include <planning/latlong_utm.h>
#include <planning/rapidxml/rapidxml.hpp>
#include <planning/rapidxml/rapidxml_utils.hpp>

vector<double> getStringCoordinates(string s, string name)
{
	string value = s.substr(s.find(name), s.length());
	size_t pos1 = value.find("{");
	size_t pos2 = value.find("}");
	string sub = value.substr(pos1 + 1, pos2 - pos1 - 1);
	vector<double> coordinates;
	string number = "";

	for (int i = 0; i < sub.length(); i++)
	{

		if (sub[i] == ',')
		{
			coordinates.push_back(std::stod(number));
			number = "";
			continue;
		}
		else
		{
			number += sub[i];
		}
	}

	coordinates.push_back(std::stod(number));
	return coordinates;
}

vector<vector<double>> getStringMatrix(string s, string name)
{
	string value = s.substr(s.find(name), s.length());
	size_t pos1 = value.find("{");
	size_t pos2 = value.find("}");
	string sub = value.substr(pos1 + 1, pos2 - pos1 - 1);
	vector<vector<double>> matrix(4);
	string number = "";
	int j = 1, z = 0;
	for (int i = 0; i < sub.length(); i++)
	{
		if (sub[i] == ',')
		{
			matrix[z].push_back(std::stod(number));
			if (j % 4 == 0)
			{
				z++;
			}
			number = "";
			j++;
			continue;
		}
		else
		{
			number += sub[i];
		}
	}
	matrix[z].push_back(std::stod(number));
	return matrix;
}

/* 
---------------------------------------------------------------------------
--------------- Código  para hacer bonitos los resultados -----------------
--------------------------------------------------------------------------- 
*/

vector<double> newVector(double x, double y, double z, double yaw)
{
	vector<double> newVector;
	newVector.push_back(x);
	newVector.push_back(y);
	newVector.push_back(z);
	newVector.push_back(yaw);
	return newVector;
}

vector<vector<double>> newMatrix(double x, double y, double z, double yaw)
{
	vector<vector<double>> newVector(4);
	for (int i = 0; i < 4; i++)
	{
		if (i == 0)
		{
			newVector[i].push_back(x);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
		}
		else if (i == 1)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(y);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
		}
		else if (i == 2)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(z);
			newVector[i].push_back(0);
		}
		else if (i == 3)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(yaw);
		}
	}

	return newVector;
}

// Crea links y nodos en el objeto grafo
void createNodesAndLinks(PlanningGraph &grafo)
{

	int contLinks = 0;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			grafo.addNode(newVector(i * 5, j * 5, 0, i + j), Util::newMatrix());
			if (i != 0 && j != 0)
			{
				/*grafo.addLink();
         grafo.addLink();*/
			}
			else if (i != 0)
			{
				//grafo.addLink();
			}
			else if (j != 0)
			{
				//grafo.addLink();
			}
		}
	}

	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (i != 0 && j != 0)
			{
				vector<double> actual = newVector(i * 5, j * 5, 0, i + j);
				vector<double> vecino1 = newVector(i * 5, (j - 1) * 5, 0, i + j);
				vector<double> vecino2 = newVector((i - 1) * 5, j * 5, 0, i + j);
				grafo.addLinkBetweenNodes(actual, vecino1);
				grafo.addLinkBetweenNodes(actual, vecino2);
			}
			else if (i != 0)
			{
				vector<double> actual = newVector(i * 5, j * 5, 0, i + j);
				vector<double> vecino2 = newVector((i - 1) * 5, j * 5, 0, i + j);
				grafo.addLinkBetweenNodes(actual, vecino2);
			}
			else if (j != 0)
			{
				vector<double> actual = newVector(i * 5, j * 5, 0, i + j);
				vector<double> vecino1 = newVector(i * 5, (j - 1) * 5, 0, i + j);
				grafo.addLinkBetweenNodes(actual, vecino1);
			}
		}
	}
}

/* 
---------------------------------------------------------------------------
--------------------------------------------------------------------------- 
*/

class GraphApi
{
private:
	Graph *g;

public:
	GraphApi()
	{
		string file = "./mapAux.osm";
		Util::Distances typeDistance = Util::Mahalanobis;
		double radiusDistance = 2;
		g = new Graph(file, Util::newMatrix(), typeDistance, radiusDistance);

		/* 
		---------------------------------------------------------------------------
		--------------- Código  para hacer bonitos los resultados -----------------
		--------------------------------------------------------------------------- 
		
		PlanningGraph grafo;
		createNodesAndLinks(grafo);
		grafo.setDistances(typeDistance);
		g->planning_graph_ = grafo;

		/* 
		---------------------------------------------------------------------------
		--------------------------------------------------------------------------- 
		*/
	}
	void getNext(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		Pose myPosition, endGoal;
		string s = request.getData();
		myPosition.coordinates = getStringCoordinates(s, "initialCoordinates");
		endGoal.coordinates = getStringCoordinates(s, "finalCoordinates");
		myPosition.matrix = getStringMatrix(s, "initialMatrix");
		endGoal.matrix = getStringMatrix(s, "finalMatrix");

		Pose next = g->getNextPose(myPosition, endGoal);

		std::ostringstream vts, vts2;
		if (!next.coordinates.empty())
		{
			std::copy(next.coordinates.begin(), next.coordinates.end() - 1,
					  std::ostream_iterator<int>(vts, ", "));
			vts << next.coordinates.back();
		}
		if (!next.matrix.empty())
		{
			for (int i = 0; i < 4; i++)
			{
				std::copy(next.matrix[i].begin(), next.matrix[i].end() - 1,
						  std::ostream_iterator<int>(vts2, ", "));
				vts2 << next.matrix[i].back();
				if (i < 3)
				{
					vts2 << ", ";
				}
			}
		}

		response << '\n' << "Coordinates: {" << vts.str() << "} ; Matriz: {" << vts2.str() << "}" << '\n';;
	}
	void getPath(GloveHttpRequest &request, GloveHttpResponse &response)
	{

		Pose myPosition, endGoal;
		string s = request.getData();
		myPosition.coordinates = getStringCoordinates(s, "initialCoordinates");
		endGoal.coordinates = getStringCoordinates(s, "finalCoordinates");
		myPosition.matrix = getStringMatrix(s, "initialMatrix");
		endGoal.matrix = getStringMatrix(s, "finalMatrix");

		vector<Pose> nexts = g->getPathPoses(myPosition, endGoal);

		std::ostringstream allnodes;
		if (!nexts.empty())
		{
			for (int i = 0; i < nexts.size(); i++)
			{
				std::ostringstream vts, vts2;

				Pose next = nexts[i];
				if (!next.coordinates.empty())
				{
					std::copy(next.coordinates.begin(), next.coordinates.end() - 1,
							  std::ostream_iterator<int>(vts, ", "));
					vts << next.coordinates.back();
				}
				if (!next.matrix.empty())
				{
					for (int j = 0; j < 4; j++)
					{
						std::copy(next.matrix[j].begin(), next.matrix[j].end() - 1,
								  std::ostream_iterator<int>(vts2, ", "));
						vts2 << next.matrix[j].back();
						if (i < 3)
						{
							vts2 << ", ";
						}
					}
				}
				allnodes << "Node [" << i << "] "
						 << "Coordinates: " << vts.str() << " ; Matriz: " << vts2.str() << '\n';
			}
		}

		response << '\n' << allnodes.str() << '\n';;
	}
	void setDijkstra(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		g->setDijkstraAlgorithm();
		response << "Algorithm has changed to dijsktra"<< '\n';
	}
	void setAStar(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		g->setAStarAlgorithm();
		response << "Algorithm has changed to A*"<< '\n';
	}
	void setEuclidean(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		Util::Distances typeDistance = Util::Euclidean;
		g->planning_graph_.setDistances(typeDistance);
		response << "Type idstance has changed to euclidean"<< '\n';
	}
	void setMahalanobis(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		Util::Distances typeDistance = Util::Mahalanobis;
		g->planning_graph_.setDistances(typeDistance);
		response << "Type idstance has changed to mahalanobis"<< '\n';
	}
	void initGraph(GloveHttpRequest &request, GloveHttpResponse &response)
	{
		response << 2;
	}
};

int main(int argc, char *argv[])
{
	GraphApi graph;
	GloveHttpServer serv(8080, "", 2048);
	serv.compression("gzip, deflate");
	namespace ph = std::placeholders;
	serv.addRest("/initGraph", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::initGraph, &graph, ph::_1, ph::_2),
				 std::bind(&GraphApi::initGraph, &graph, ph::_1, ph::_2));
	serv.addRest("/getNextPose", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::getNext, &graph, ph::_1, ph::_2),
				 std::bind(&GraphApi::getNext, &graph, ph::_1, ph::_2));
	serv.addRest("/getNextPath", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::getPath, &graph, ph::_1, ph::_2),
				 std::bind(&GraphApi::getPath, &graph, ph::_1, ph::_2));
	serv.addRest("/setDijkstra", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::setDijkstra, &graph, ph::_1, ph::_2));
	serv.addRest("/setAStar", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::setAStar, &graph, ph::_1, ph::_2));
	serv.addRest("/setEuclidean", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::setEuclidean, &graph, ph::_1, ph::_2));
	serv.addRest("/setMahalanobis", 1,
				 GloveHttpServer::jsonApiErrorCall,
				 std::bind(&GraphApi::setMahalanobis, &graph, ph::_1, ph::_2));
	std::cout << "localhost:8080" << std::endl;
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
