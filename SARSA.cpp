#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <cmath>
#include <iomanip>
#include<chrono>
#include <thread>

// class SARSA {
// public:
//     SARSA(int stateSize, int actionSize, double alpha, double gamma, double epsilon)
//         : stateSize(stateSize), actionSize(actionSize), alpha(alpha), gamma(gamma), epsilon(epsilon), gen(std::random_device{}()) {
//         qTable.reserve(stateSize);
//         for (int i = 0; i < stateSize; ++i) {
//             qTable.emplace_back(actionSize, 0.0);
//         }
//     }

//     int chooseAction(int state) {
//         std::uniform_real_distribution<double> dist(0.0, 1.0);
//         if (dist(gen) < epsilon) {
//             std::uniform_int_distribution<int> actionDist(0, actionSize - 1);
//             return actionDist(gen);
//         } else {
//             return std::distance(qTable[state].begin(), std::max_element(qTable[state].begin(), qTable[state].end()));
//         }
//     }

//     void update(int state, int action, int nextState, int nextAction, double reward) {
//         double predict = qTable[state][action];
//         double target = reward + gamma * qTable[nextState][nextAction];
//         qTable[state][action] += alpha * (target - predict);
//     }

//     void updateEpsilon(double decay) {
//         epsilon *= decay;
//     }

// private:
//     int stateSize;
//     int actionSize;
//     double alpha;
//     double gamma;
//     double epsilon;
//     std::vector<std::vector<double>> qTable;
//     std::mt19937 gen;
// };

class SARSA {
public:
    SARSA(int stateSize, int actionSize, double alpha, double gamma, double epsilon, double lambda)
        : stateSize(stateSize), actionSize(actionSize), alpha(alpha), gamma(gamma), epsilon(epsilon), lambda(lambda), gen(std::random_device{}()) {
        qTable.reserve(stateSize);
        eTable.reserve(stateSize);
        for (int i = 0; i < stateSize; ++i) {
            qTable.emplace_back(actionSize, 0.0);
            eTable.emplace_back(actionSize, 0.0);
        }
    }

    int chooseAction(int state) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        if (dist(gen) < epsilon) {
            std::uniform_int_distribution<int> actionDist(0, actionSize - 1);
            return actionDist(gen);
        } else {
            return std::distance(qTable[state].begin(), std::max_element(qTable[state].begin(), qTable[state].end()));
        }
    }

    void update(int state, int action, int nextState, int nextAction, double reward) {
        double delta = reward + gamma * qTable[nextState][nextAction] - qTable[state][action];
        eTable[state][action] += 1;

        for (int s = 0; s < stateSize; ++s) {
            for (int a = 0; a < actionSize; ++a) {
                qTable[s][a] += alpha * delta * eTable[s][a];
                eTable[s][a] *= gamma * lambda;
            }
        }
    }

    void updateEpsilon(double decay) {
        epsilon *= decay;
    }

    void resetTraces() {
        for (int s = 0; s < stateSize; ++s) {
            for (int a = 0; a < actionSize; ++a) {
                eTable[s][a] = 0.0;
            }
        }
    }

private:
    int stateSize;
    int actionSize;
    double alpha;
    double gamma;
    double epsilon;
    double lambda;
    std::vector<std::vector<double>> qTable;
    std::vector<std::vector<double>> eTable;
    std::mt19937 gen;
};

class Robot;

class Forest {
public:
    Forest(int width, int height, int numObstacles)
        : width(width), height(height), numObstacles(numObstacles) {
        std::srand(std::time(nullptr));
        generateObstacles();
    }

    bool isObstacle(int x, int y) const {
        return std::find(obstacles.begin(), obstacles.end(), std::make_pair(x, y)) != obstacles.end();
    }
    void printForest(const Robot& robot) const;
    void generateObstacles() {
        for (int i = 0; i < numObstacles; ++i) {
            int x = std::rand() % width;
            int y = std::rand() % height;
            if (!isObstacle(x, y)) {
                obstacles.emplace_back(x, y);
            } else {
                --i;
            }
        }
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    int width;
    int height;
    int numObstacles;
    std::vector<std::pair<int, int>> obstacles;
};

class Robot {
public:
    Robot(int x, int y, const Forest& forest)
        : x(x), y(y), dx(1), dy(0), forest(forest), sensorOffset(10) {}

    enum class Action { Forward, TurnLeft, TurnRight };

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

    int getState() const { return x * forest.getWidth() + y; }
    void reset() { x = 0; y = 0; dx = 1; dy = 0; }
    bool goalReached() const { return x == forest.getWidth() - 1 && y == forest.getHeight() - 1; }
    double calculateReward() const { return goalReached() ? 100.0 : -1.0; }

private:
    int x, y, dx, dy, sensorOffset;
    const Forest& forest;

    void moveForward() {
        int newX = x + dx;
        int newY = y + dy;
        if (newX >= 0 && newX < forest.getWidth() && newY >= 0 && newY < forest.getHeight() && !forest.isObstacle(newX, newY)) {
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

void Forest::printForest(const Robot& robot) const {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (robot.getX() == x && robot.getY() == y) {
                std::cout << "R";
            } else if (isObstacle(x, y)) {
                std::cout << "X";
            } else {
                std::cout << ".";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
const int stateSize = 10000; // You can adjust this value based on the size of the forest
const int actionSize = 3; // Forward, TurnLeft, TurnRight
const int episodes = 100;
const int evaluationEpisodes = 20;
double totalSteps = 0.0;
double totalStepsSquared = 0.0;
double varianceSteps = 0.0;
int successfulEpisodes = 0;

int main() {
    Forest forest(100, 100, 1623); // Forest with 100x100 grid and 20 obstacles
    Robot robot(0, 0, forest);
    SARSA sarsa(stateSize, actionSize, 0.4, 0.99, 0.02, 0.9); // alpha=0.1, gamma=0.99, epsilon=0.1

    for (int e = 0; e < episodes; ++e) {
        sarsa.resetTraces();
        robot.reset(); // Reset robot position and orientation
        int state = robot.getState();
        int action = sarsa.chooseAction(state);

        int steps = 0;
        while (!robot.goalReached()) {
            // forest.printForest(robot); // Print the current state
            // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add a small delay

            robot.performAction(static_cast<Robot::Action>(action));
            int nextState = robot.getState();
            int nextAction = sarsa.chooseAction(nextState);

            double reward = robot.calculateReward();
            sarsa.update(state, action, nextState, nextAction, reward);

            state = nextState;
            action = nextAction;
            ++steps;
            // std::cout << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
            // std::cout << e + 1 << "," << steps << std::endl; // Add this line
            
        }
        // if(e < episodes && e > evaluationEpisodes){

        //     }
        if (robot.goalReached()) {
            ++successfulEpisodes;
            totalSteps += steps;
            totalStepsSquared += pow(totalSteps,2);
            }
        
        std::cout << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
        if (e % evaluationEpisodes == 0 && e != 0) {
        sarsa.updateEpsilon(0.99); // Update epsilon with a decay rate of 0.99
        }
    }
    double averageSteps = totalSteps / successfulEpisodes;
    std::cout << "Average number of steps to reach the goal: " << std::fixed << std::setprecision(2) << averageSteps << std::endl;
    
    double successRate = (static_cast<double>(successfulEpisodes) / episodes) * 100;
    std::cout << "Overall success rate: " << std::fixed << std::setprecision(2) << successRate << "%" << std::endl;

    
    double averageStepsSquared = totalStepsSquared / successfulEpisodes;
    varianceSteps = (averageStepsSquared - pow(averageSteps,2))/(episodes-1);

    std::cout << "Variance of steps to reach the goal: " << std::fixed << std::setprecision(2) << varianceSteps << std::endl;
    return 0;
}
