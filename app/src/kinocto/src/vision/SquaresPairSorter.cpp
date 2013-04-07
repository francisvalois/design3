#include "vision/SquaresPairSorter.h"

using namespace cv;
using namespace std;

SquaresPairSorter::SquaresPairSorter() {
}

SquaresPairSorter::~SquaresPairSorter() {
}

bool compareXPos(const SquarePair& pair1, const SquarePair& pair2) {
    return pair1.rect.x < pair2.rect.x;
}

bool compareYPos(const SquarePair& pair1, const SquarePair& pair2) {
    return pair1.rect.y < pair2.rect.y;
}

vector<vector<SquarePair> > SquaresPairSorter::sortSquaresPair(vector<SquarePair> squaresPair, const int frameWidth) {
    sort(squaresPair.begin(), squaresPair.end(), compareXPos); //Tri selon les x des Rect

    //Séparation des colonnes selon la variation importante en x entre i et i+1
    int dx = frameWidth / 20; //Seuil pour tester la variation
    int actualXColumn = 0;
    vector<vector<SquarePair> > colonnesX(8);
    vector<SquarePair>::iterator it;
    for (it = squaresPair.begin(); it != squaresPair.end() && actualXColumn < 8; ++it) {
        colonnesX[actualXColumn].push_back(*it);

        if ((it + 1) != squaresPair.end()) {
            if ((it->rect.x + dx) < (it + 1)->rect.x) {
                actualXColumn++;
            }
        }
    }

    for (int i = 0; i < 8; i++) {   //Tri selon les y des Rect
        sort(colonnesX[i].begin(), colonnesX[i].end(), compareYPos);
    }

    return colonnesX;
}
