#include "Saves.h"

Saves::Saves() {
    this->readScores();
}


inline bool fileExists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}
int sortScores(save a, save b) {
    return (a.score - b.score < 0);
}

void Saves::readScores() {
        scores.clear();
        std::ifstream fileReader("C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/data/saves.txt");
        fileReader >> j;
        numOfScores = j["numOfScores"];
        for (int i = 0; i < numOfScores; i++)
        {
            std::string currentName = j[std::to_string(i)]["name"];
            int currentScore = j[std::to_string(i)]["score"];
            save* s = new save();
            s->name = currentName;
            s->score = currentScore;
            scores.push_back(*s);
        }
        std::sort(scores.begin(), scores.end(), sortScores);
}

void Saves::saveScore(std::string playerName, int score) {
        this->readScores();
        bool isFull = true;
        if (scores.size() < 3)
            isFull = false;
        std::ofstream fileWriter("C:/Users/xxsha/OneDrive/Desktop/ngineForAnimationCourse/tutorial/data/saves.txt");
        if (!isFull) {
            save* nSave = new save();
            nSave->score = score;
            nSave->name = playerName;
            scores.push_back(*nSave);
            std::sort(scores.begin(), scores.end(), sortScores);
            j["numOfScores"] = scores.size();
            for (int i = 0; i < scores.size(); i++) {
                std::string indexStr = std::to_string(i);
                j[indexStr]["name"] = scores[i].name;
                j[indexStr]["score"] = scores[i].score;
            }
            fileWriter << std::setw(4) << j << std::endl;
            fileWriter.flush();
            fileWriter.close();
        }
        else {
            save* newRow = new save();
            newRow->score = score;
            newRow->name = playerName;
            std::vector<save> temp;
            for (int i = 0; i < scores.size(); i++)
                temp.push_back(scores[i]);
            temp.push_back(*newRow);
            std::sort(temp.begin(), temp.end(), sortScores);
            j["numOfScores"] = scores.size();
            for (int i = 0; i < scores.size(); i++) {
                std::string indexStr = std::to_string(i);
                j[indexStr]["name"] = temp[i + 1].name;
                j[indexStr]["score"] = temp[i + 1].score;
            }
            fileWriter << std::setw(4) << j << std::endl;
            fileWriter.flush();
            fileWriter.close();
        }
}