#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <thread>
#include <map>
#include <sstream>
#include <fstream>
#include "../CImg-3.2.4_pre042123/CImg.h"
using namespace cimg_library;

class SARSA {
public:
    SARSA(int stateSize, int actionSize, double alpha, double gamma, double epsilon)
        : stateSize(stateSize), actionSize(actionSize), alpha(alpha), gamma(gamma), epsilon(epsilon) {
        qTable.resize(stateSize, std::vector<double>(actionSize, 0.0));
        // std::srand(std::time(nullptr));
    }

    int chooseAction(int state) {
        if (static_cast<double>(std::rand()) / RAND_MAX < epsilon) {
            return std::rand() % actionSize;
        } else {
            return std::distance(qTable[state].begin(), std::max_element(qTable[state].begin(), qTable[state].end()));
        }
    }

    void update(int state, int action, int nextState, int nextAction, double reward) {
        double predict = qTable[state][action];
        double target = reward + gamma * qTable[nextState][nextAction];
        qTable[state][action] += alpha * (target - predict);
    }



    void updateEpsilon(double decay) {
    epsilon *= decay;
    }

    void saveModel(const std::string& filename) const {
        std::ofstream outFile(filename);
        for (const auto& row : qTable) {
            for (size_t i = 0; i < row.size(); ++i) {
                outFile << row[i];
                if (i < row.size() - 1) {
                    outFile << ",";
                }
            }
            outFile << std::endl;
        }
    }

private:
    int stateSize;
    int actionSize;
    double alpha;
    double gamma;
    double epsilon;
    std::vector<std::vector<double>> qTable;
};


class Robot;

class Forest {
public:
    enum class ObstacleType { None, Tree, Rock, Water };

    Forest(int width, int height, int numObstacles)
        : width(width), height(height), numObstacles(numObstacles) {
        std::srand(std::time(nullptr));
        generateObstacles();
    }

    ObstacleType getObstacle(int x, int y) const {
        auto it = obstacles.find(std::make_pair(x, y));
        if (it != obstacles.end()) {
            return it->second;
        } else {
            return ObstacleType::None;
        }
    }

    // void printForest(const Robot& robot) const;
    void printForest(const Robot& robot, const std::string& filename = "") const;
    void generateObstacles();
    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    int width;
    int height;
    int numObstacles;
    std::map<std::pair<int, int>, ObstacleType> obstacles;

    void generateWater();
    int countWaterNeighbors(int x, int y);
};

class Robot {
public:
    Robot(int x, int y, const Forest& forest)
        : x(x), y(y), dx(1), dy(0), forest(forest), sensorOffset(10) {}

    enum class Action { Forward, TurnLeft, TurnRight };

    // return x * forest.getWidth() * 4 + y * 4 + orientation;
    void performAction(Action action) {
        switch (action) {
            case Action::Forward:
                moveForward();
                break;
            case Action::TurnLeft:
                turnLeft();
                break;
            case Action::TurnRight:
                turnRight();
                break;
        }
    }

    int getX() const { return x; }
    int getY() const { return y; }

    // int getState() const { return x * forest.getWidth() + y; }
    int getState() const {
        int orientation = 0;
        if (dx == 1 && dy == 0) orientation = 0;
        if (dx == 0 && dy == 1) orientation = 1;
        if (dx == -1 && dy == 0) orientation = 2;
        if (dx == 0 && dy == -1) orientation = 3;
        return x * forest.getWidth() * 4 + y * 4 + orientation;
    }
    void reset() { x = 0; y = 0; dx = 1; dy = 0; }
    bool goalReached() const { return x == forest.getWidth() - 1 && y == forest.getHeight() - 1; }
    double calculateReward() const { return goalReached() ? 100.0 : -1.0; }

private:
    int x, y, dx, dy, sensorOffset;
    const Forest& forest;

    void moveForward() {
        int newX = x + dx;
        int newY = y + dy;
        if (newX >= 0 && newX < forest.getWidth() && newY >= 0 && newY < forest.getHeight() && forest.getObstacle(newX, newY) == Forest::ObstacleType::None) {
            x = newX;
            y = newY;
        }
    }

    void turnLeft() {
        int tmp = dx;
        dx = dy;
        dy = -tmp;
    }

    void turnRight() {
        int tmp = dx;
        dx = -dy;
        dy = tmp;
    }
};


// void Forest::printForest(const Robot& robot, const std::string& filename) const {
//     int cellSize = 20; // You can adjust this value to change the size of the image
//     CImg<unsigned char> image(width * cellSize, height * cellSize, 1, 3, 255);

//     for (int y = 0; y < height; ++y) {
//         for (int x = 0; x < width; ++x) {
//             unsigned char color[3];
//             if (robot.getX() == x && robot.getY() == y) {
//                 color[0] = 0; color[1] = 0; color[2] = 255; // Robot: Blue
//             } else {
//                 switch (getObstacle(x, y)) {
//                     case ObstacleType::Tree:
//                         color[0] = 0; color[1] = 128; color[2] = 0; // Tree: Green
//                         break;
//                     case ObstacleType::Rock:
//                         color[0] = 128; color[1] = 128; color[2] = 128; // Rock: Gray
//                         break;
//                     case ObstacleType::Water:
//                         color[0] = 0; color[1] = 255; color[2] = 255; // Water: Cyan
//                         break;
//                     default:
//                         color[0] = 255; color[1] = 255; color[2] = 255; // Empty: White
//                         break;
//                 }
//             }
//             image.draw_rectangle(x * cellSize, y * cellSize, (x + 1) * cellSize, (y + 1) * cellSize, color, 1.0f, ~0U);
//         }
//     }

//     image.save(filename.c_str());
// }

void Forest::printForest(const Robot& robot, const std::string& filename) const {
    int cellSize = 20;
    CImg<unsigned char> image(width * cellSize, height * cellSize, 1, 3, 0);

    unsigned char emptyColor[] = {0, 128, 0};
    image.draw_rectangle(0, 0, width * cellSize, height * cellSize, emptyColor);

    unsigned char borderColor[] = {0, 0, 0};
    int borderWidth = 2;
    image.draw_rectangle(0, 0, width * cellSize, borderWidth, borderColor);
    image.draw_rectangle(0, height * cellSize - borderWidth, width * cellSize, height * cellSize, borderColor);
    image.draw_rectangle(0, 0, borderWidth, height * cellSize, borderColor);
    image.draw_rectangle(width * cellSize - borderWidth, 0, width * cellSize, height * cellSize, borderColor);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            unsigned char color[3];
            int centerX = x * cellSize + cellSize / 2;
            int centerY = y * cellSize + cellSize / 2;

            int startX = centerX - cellSize / 2;
            int startY = centerY - cellSize / 2;
            int endX = centerX + cellSize / 2;
            int endY = centerY + cellSize / 2;

            switch (getObstacle(x, y)) {
                case ObstacleType::Tree:
                    color[0] = 0; color[1] = 64; color[2] = 0; // Tree: Green
                    image.draw_circle(centerX, centerY, cellSize / 3, color);
                    break;
                case ObstacleType::Rock:
                    color[0] = 32; color[1] = 32; color[2] = 32; // Rock: Gray
                    {
                        int dotSpacing = 5; // Adjust this value to change the spacing between dots

                        for (int x = centerX - cellSize / 2 + dotSpacing / 2; x < centerX + cellSize / 2; x += dotSpacing) {
                            for (int y = centerY - cellSize / 2 + dotSpacing / 2; y < centerY + cellSize / 2; y += dotSpacing) {
                                image.draw_circle(x, y, 1, color); // Draw a gray dot
                            }
                        }
                    }
                    break;
                case ObstacleType::Water:
                    color[0] = 0; color[1] = 200; color[2] = 200;
                    startX += (x == 0 ? borderWidth : 0);
                    startY += (y == 0 ? borderWidth : 0);
                    endX -= (x == width - 1 ? borderWidth : 0);
                    endY -= (y == height - 1 ? borderWidth : 0);
                    image.draw_rectangle(startX, startY, endX, endY, color);
                    break;

                default:
                    break;
            }

            if (robot.getX() == x && robot.getY() == y) {
                color[0] = 0; color[1] = 0; color[2] = 255; // Robot: Blue
                image.draw_circle(centerX, centerY, cellSize / 4, color);
            }
        }
    }

    image.save(filename.c_str());
}



void Forest::generateObstacles() {
    // Modified probability distribution for obstacles
    std::vector<std::pair<ObstacleType, double>> obstacleTypes = {
        {ObstacleType::Tree, 0.7},
        {ObstacleType::Rock, 0.2},
        {ObstacleType::Water, 0.1}
    };

    std::discrete_distribution<> dist({0.7, 0.2, 0.1});

    for (int i = 0; i < numObstacles; ++i) {
        int x = std::rand() % width;
        int y = std::rand() % height;
        ObstacleType obstacleType = obstacleTypes[dist(std::mt19937{std::random_device{}()})].first;
        if (getObstacle(x, y) == ObstacleType::None) {
            obstacles[std::make_pair(x, y)] = obstacleType;
        } else {
            --i;
        }
    }
    generateWater();
}


void Forest::generateWater() {
    int waterPercentage = 45; // Adjust the percentage of water coverage
    int iterations = 3; // Number of iterations for the cellular automata algorithm

    // Initialize water randomly
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if ((std::rand() % 100) < waterPercentage && getObstacle(x, y) == ObstacleType::None) {
                obstacles[std::make_pair(x, y)] = ObstacleType::Water;
            }
        }
    }

    // Apply the cellular automata algorithm
    for (int i = 0; i < iterations; ++i) {
        std::map<std::pair<int, int>, ObstacleType> newObstacles = obstacles;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int waterNeighbors = countWaterNeighbors(x, y);
                if (waterNeighbors >= 5) {
                    newObstacles[std::make_pair(x, y)] = ObstacleType::Water;
                } else {
                    if (getObstacle(x, y) == ObstacleType::Water) {
                        newObstacles[std::make_pair(x, y)] = ObstacleType::None;
                    }
                }
            }
        }
        obstacles = newObstacles;
    }
}

int Forest::countWaterNeighbors(int x, int y) {
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width && ny >= 0 && ny < height && getObstacle(nx, ny) == ObstacleType::Water) {
                count++;
            }
        }
    }
    return count;
}

const int stateSize = 40000; // You can adjust this value based on the size of the forest
const int actionSize = 3; // Forward, TurnLeft, TurnRight
const int episodes = 400;
const int evaluationEpisodes = 100;
double totalSteps = 0.0;
double totalStepsSquared = 0.0;
double varianceSteps = 0.0;
int successfulEpisodes = 0;

int main() {

    const int maxStepsPerEpisode = 70000000;
    const int stepsToStartCalculatingVariance = episodes - evaluationEpisodes;
    SARSA sarsa(stateSize, actionSize, 0.1, 0.99, 0.5); // alpha=0.1, gamma=0.99, epsilon=0.1

    std::ofstream stepsFile("steps_per_episode.txt");

    for (int e = 0; e < episodes; ++e) {
        Forest forest(100, 100, 2000); // Generate a new forest for each episode
        Robot robot(0, 0, forest);

        robot.reset(); // Reset robot position and orientation
        int state = robot.getState();
        int action = sarsa.chooseAction(state);

        int steps = 0;
        while (!robot.goalReached() && steps < maxStepsPerEpisode) {
            robot.performAction(static_cast<Robot::Action>(action));
            int nextState = robot.getState();
            int nextAction = sarsa.chooseAction(nextState);

            double reward = robot.calculateReward();
            sarsa.update(state, action, nextState, nextAction, reward);

            state = nextState;
            action = nextAction;
            ++steps;
        }
        if (robot.goalReached()) {
            ++successfulEpisodes;
            totalSteps += steps;
            totalStepsSquared += pow(totalSteps,2);
            std::cout << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
            // forest.printForest(robot); // Print the final state of each episode
            forest.printForest(robot, "forest_state_" + std::to_string(e + 1) + ".png");
            stepsFile << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
            // break;
        } else {
            std::cout << "Episode " << e + 1 << ": Failed after " << steps << " steps" << std::endl;
            // stepsFile << "Episode " << e + 1 << ": Failed after " << steps << " steps" << std::endl;
        }
        if (e >= stepsToStartCalculatingVariance) {
            totalStepsSquared += steps * steps;
            if (e == episodes - 1) {
                double mean = totalSteps / evaluationEpisodes;
                double variance = (totalStepsSquared / evaluationEpisodes) - (mean * mean);
                varianceSteps = variance;
            }
        }
            
        
        // std::cout << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
        if (e % evaluationEpisodes == 0 && e != 0) {
        sarsa.updateEpsilon(0.99); // Update epsilon with a decay rate of 0.99
        }

        // std::cout << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
        // forest.printForest(robot); // Print the final state of each episode
    }

    double averageSteps = totalSteps / successfulEpisodes;
    std::cout << "Average number of steps to reach the goal: " << std::fixed << std::setprecision(2) << averageSteps << std::endl;
    
    double successRate = (static_cast<double>(successfulEpisodes) / episodes) * 100;
    std::cout << "Overall success rate: " << std::fixed << std::setprecision(2) << successRate << "%" << std::endl;
    double meanSteps = totalSteps / episodes;
    double variancePercentage = (std::sqrt(varianceSteps) / meanSteps) * 100;
    std::cout << "Variance of steps for the last 100 episodes (as percentage): " << variancePercentage << "%" << std::endl;
    sarsa.saveModel("sarsa_model.csv");

    return 0;
}