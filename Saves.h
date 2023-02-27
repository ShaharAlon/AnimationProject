#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include "json.hpp"
#include <string> 


typedef struct {
	std::string name;
	int score;
} save;

class Saves {
public:
	Saves();
	void saveScore(std::string playerName, int score);
	void readScores();
	std::vector<save>& getScores() {
		return scores;
	}
private:
	nlohmann::json j;
	int numOfScores=0;
	std::vector <save> scores;
};
