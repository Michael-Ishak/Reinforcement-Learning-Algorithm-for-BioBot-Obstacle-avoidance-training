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
#include <queue>
#include <cmath>
using namespace cimg_library;

class SARSA
{
public:
    SARSA(int stateSize, int actionSize, double alpha, double gamma, double epsilon)
        : stateSize(stateSize), actionSize(actionSize), alpha(alpha), gamma(gamma), epsilon(epsilon)
    {
        qTable.resize(stateSize, std::vector<double>(actionSize, 0.0));
        // std::srand(std::time(nullptr));
    }

    int chooseAction(int state)
    {
        if (static_cast<double>(std::rand()) / RAND_MAX < epsilon)
        {
            return std::rand() % actionSize;
        }
        else
        {
            return std::distance(qTable[state].begin(), std::max_element(qTable[state].begin(), qTable[state].end()));
        }
    }

    void update(int state, int action, int nextState, int nextAction, double reward)
    {
        double predict = qTable[state][action];
        double target = reward + gamma * qTable[nextState][nextAction];
        qTable[state][action] += alpha * (target - predict);
    }

    void updateEpsilon(double decay)
    {
        epsilon *= decay;
    }

    void saveModel(const std::string &filename) const
    {
        std::ofstream outFile(filename);
        for (const auto &row : qTable)
        {
            for (size_t i = 0; i < row.size(); ++i)
            {
                outFile << row[i];
                if (i < row.size() - 1)
                {
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

class Forest
{
public:
    enum class ObstacleType
    {
        None,
        Tree,
        Rock,
        Water,
        Marshy
    };

    Forest(int width, int height, int numObstacles)
        : width(width), height(height), numObstacles(numObstacles)
    {
        std::srand(std::time(nullptr));
        std::cout << "Check 1" << std::endl;
        generateObstacles();
        std::cout << "Check 2" << std::endl;
        shortestPathLength = getShortestPathLength();
        std::cout << "Check 3" << std::endl;
    }

    struct Node
    {
        int x;
        int y;
        int gScore;
        int hScore;
        int fScore() const { return gScore + hScore; }
    };

    static bool compare(const Node &a, const Node &b)
    {
        return a.fScore() > b.fScore();
    }

    int shortestPathLength;
    std::vector<std::pair<int, int>> shortestPath;
    int getShortestPathLength()
    {
        shortestPath.clear();
        std::priority_queue<Node, std::vector<Node>, decltype(&compare)> openSet(&compare);
        std::vector<std::vector<int>> gScores(width, std::vector<int>(height, INT_MAX));
        std::vector<std::vector<int>> hScores(width, std::vector<int>(height, INT_MAX));
        std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
        std::vector<std::vector<Node>> parent(width, std::vector<Node>(height));
        Node startNode = {0, 0, 0, std::abs(width - 1) + std::abs(height - 1)};
        openSet.push(startNode);
        gScores[0][0] = 0;

        while (!openSet.empty())
        {
            Node current = openSet.top();
            openSet.pop();
            if (current.x == width - 1 && current.y == height - 1)
            {
                // Reached the goal
                int pathLength = 0;
                shortestPath.clear(); // Clear the vector before adding new elements
                while (!(current.x == 0 && current.y == 0))
                {
                    ++pathLength;
                    shortestPath.push_back(std::make_pair(current.x, current.y)); // Add current node to the shortest path
                    current = parent[current.x][current.y];
                }
                shortestPath.push_back(std::make_pair(0, 0));           // Add start node to the shortest path
                std::reverse(shortestPath.begin(), shortestPath.end()); // Reverse the order of elements to get the correct path
                return pathLength;
            }
            visited[current.x][current.y] = true;

            // Check neighbors
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 && dy == 0)
                        continue; // Don't consider current node as neighbor
                    int neighborX = current.x + dx;
                    int neighborY = current.y + dy;
                    if (neighborX < 0 || neighborX >= width || neighborY < 0 || neighborY >= height)
                        continue; // Out of bounds
                    if (visited[neighborX][neighborY])
                        continue; // Already visited
                    ObstacleType obstacle = getObstacle(neighborX, neighborY);
                    if (obstacle != ObstacleType::None)
                        continue;                                            // Obstacle
                    int tentativeGScore = gScores[current.x][current.y] + 1; // Distance of 1 between neighboring nodes
                    if (tentativeGScore < gScores[neighborX][neighborY])
                    {
                        gScores[neighborX][neighborY] = tentativeGScore;
                        hScores[neighborX][neighborY] = std::abs(neighborX - (width - 1)) + std::abs(neighborY - (height - 1));
                        Node neighborNode = {neighborX, neighborY, tentativeGScore, hScores[neighborX][neighborY]};
                        openSet.push(neighborNode);
                        parent[neighborX][neighborY] = current;
                    }
                }
            }
        }
        shortestPath.clear();
        return -1;
    }

    ObstacleType getObstacle(int x, int y) const
    {
        auto it = obstacles.find(std::make_pair(x, y));
        if (it != obstacles.end())
        {
            return it->second;
        }
        else
        {
            return ObstacleType::None;
        }
    }

    // void printForest(const Robot& robot) const;
    void printForest(const Robot &robot, const std::string &filename = "") const;
    void generateObstacles();
    int getWidth() const { return width; }
    int getHeight() const { return height; }

    const std::map<ObstacleType, int> &getObstacleCounts() const { return obstacleCounts; }

    int numObstaclesOfType(ObstacleType type) const
    {
        int count = 0;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                if (getObstacle(i, j) == type)
                {
                    ++count;
                }
            }
        }
        return count;
    }

private:
    int width;
    int height;
    int numObstacles;
    std::map<std::pair<int, int>, ObstacleType> obstacles;

    void generateWater();
    int countWaterNeighbors(int x, int y);
    void generateMarshy();
    int countMarshyNeighbors(int x, int y);
    std::map<ObstacleType, int> obstacleCounts;
};

class Robot
{
public:
    Robot(int x, int y, const Forest &forest)
        : x(x), y(y), dx(1), dy(0), forest(forest), sensorOffset(10), hoursPassed(0), path() {}

    struct Coordinates
    {
        int x;
        int y;
    };

    std::vector<Coordinates> path;

    void addToPath(int x, int y)
    {
        Coordinates coords{x, y};
        path.push_back(coords);
    }

    enum class Action
    {
        Forward,
        TurnLeft,
        TurnRight
    };

    // return x * forest.getWidth() * 4 + y * 4 + orientation;
    void performAction(Action action)
    {
        switch (action)
        {
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
    int getState() const
    {
        int orientation = 0;
        if (dx == 1 && dy == 0)
            orientation = 0;
        if (dx == 0 && dy == 1)
            orientation = 1;
        if (dx == -1 && dy == 0)
            orientation = 2;
        if (dx == 0 && dy == -1)
            orientation = 3;
        return x * forest.getWidth() * 4 + y * 4 + orientation;
    }
    void reset()
    {
        x = 0;
        y = 0;
        dx = 1;
        dy = 0;
    }
    bool goalReached() const { return x == forest.getWidth() - 1 && y == forest.getHeight() - 1; }
    double calculateReward() const { return goalReached() ? 100.0 : -1.0; }

    // Add a new method to reset the hours passed counter
    void resetHoursPassed() { hoursPassed = 0; }

    // Add a new method to increment the hours passed counter
    void incrementHoursPassed() { hoursPassed++; }

    // Add a new method to check if the robot needs to recharge
    bool needsRecharge() const { return hoursPassed % 504 == 0 && hoursPassed != 0; }

    int simulationHoursPassed;
    void incrementSimulationHoursPassed();
    void resetSimulationHoursPassed();
    void recharge();

    int waterFailures = 0;
    int rockFailures = 0;
    int treeFailures = 0;
    int marshyFailures = 0;

    void printFailureStats()
    {
        int totalObstacles = forest.numObstaclesOfType(Forest::ObstacleType::Water) + forest.numObstaclesOfType(Forest::ObstacleType::Rock) + forest.numObstaclesOfType(Forest::ObstacleType::Tree) + forest.numObstaclesOfType(Forest::ObstacleType::Marshy);
        double waterFailuresPercentage = (static_cast<double>(waterFailures) / forest.numObstaclesOfType(Forest::ObstacleType::Water)) * 100;
        double rockFailuresPercentage = (static_cast<double>(rockFailures) / forest.numObstaclesOfType(Forest::ObstacleType::Rock)) * 100;
        double treeFailuresPercentage = (static_cast<double>(treeFailures) / forest.numObstaclesOfType(Forest::ObstacleType::Tree)) * 100;
        double marshyFailuresPercentage = (static_cast<double>(marshyFailures) / forest.numObstaclesOfType(Forest::ObstacleType::Marshy)) * 100;

        std::cout << "Water Failures: " << waterFailures << " (" << std::fixed << std::setprecision(2) << waterFailuresPercentage << "%)" << std::endl;
        std::cout << "Rock Failures: " << rockFailures << " (" << std::fixed << std::setprecision(2) << rockFailuresPercentage << "%)" << std::endl;
        std::cout << "Tree Failures: " << treeFailures << " (" << std::fixed << std::setprecision(2) << treeFailuresPercentage << "%)" << std::endl;
        std::cout << "Marshy Failures: " << marshyFailures << " (" << std::fixed << std::setprecision(2) << marshyFailuresPercentage << "%)" << std::endl;
    }

private:
    int x, y, dx, dy, sensorOffset;
    const Forest &forest;
    int hoursPassed;
    int batteryPercentage;

    void moveForward()
    {
        int newX = x + dx;
        int newY = y + dy;
        addToPath(newX, newY);
        if (newX >= 0 && newX < forest.getWidth() && newY >= 0 && newY < forest.getHeight())
        {
            Forest::ObstacleType obstacle = forest.getObstacle(newX, newY);
            if (obstacle == Forest::ObstacleType::None)
            {
                x = newX;
                y = newY;
            }
            else
            {
                if (obstacle == Forest::ObstacleType::Water)
                {
                    waterFailures++;
                }
                else if (obstacle == Forest::ObstacleType::Rock)
                {
                    rockFailures++;
                }
                else if (obstacle == Forest::ObstacleType::Tree)
                {
                    treeFailures++;
                }
                else if (obstacle == Forest::ObstacleType::Marshy)
                {
                    marshyFailures++;
                }
            }
        }
    }

    void turnLeft()
    {
        int tmp = dx;
        dx = dy;
        dy = -tmp;
    }

    void turnRight()
    {
        int tmp = dx;
        dx = -dy;
        dy = tmp;
    }
};

void Forest::printForest(const Robot &robot, const std::string &filename) const
{
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

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            unsigned char color[3];
            int centerX = x * cellSize + cellSize / 2;
            int centerY = y * cellSize + cellSize / 2;

            int startX = centerX - cellSize / 2;
            int startY = centerY - cellSize / 2;
            int endX = centerX + cellSize / 2;
            int endY = centerY + cellSize / 2;

            switch (getObstacle(x, y))
            {
            case ObstacleType::Tree:
                color[0] = 0;
                color[1] = 64;
                color[2] = 0; // Tree: Green
                image.draw_circle(centerX, centerY, cellSize / 3, color);
                break;
            case ObstacleType::Rock:
                color[0] = 32;
                color[1] = 32;
                color[2] = 32; // Rock: Gray
                {
                    int dotSpacing = 5; // Adjust this value to change the spacing between dots

                    for (int x = centerX - cellSize / 2 + dotSpacing / 2; x < centerX + cellSize / 2; x += dotSpacing)
                    {
                        for (int y = centerY - cellSize / 2 + dotSpacing / 2; y < centerY + cellSize / 2; y += dotSpacing)
                        {
                            image.draw_circle(x, y, 1, color); // Draw a gray dot
                        }
                    }
                }
                break;
            case ObstacleType::Water:
                color[0] = 0;
                color[1] = 200;
                color[2] = 200;
                startX += (x == 0 ? borderWidth : 0);
                startY += (y == 0 ? borderWidth : 0);
                endX -= (x == width - 1 ? borderWidth : 0);
                endY -= (y == height - 1 ? borderWidth : 0);
                image.draw_rectangle(startX, startY, endX, endY, color);
                break;

            case ObstacleType::Marshy:
                color[0] = 164;
                color[1] = 87;
                color[2] = 41;
                startX += (x == 0 ? borderWidth : 0);
                startY += (y == 0 ? borderWidth : 0);
                endX -= (x == width - 1 ? borderWidth : 0);
                endY -= (y == height - 1 ? borderWidth : 0);
                image.draw_rectangle(startX, startY, endX, endY, color);
                break;

            default:
                break;
            }

            // if (robot.getX() == x && robot.getY() == y) {
            //     color[0] = 255; color[1] = 192; color[2] = 203; // Robot: Blue
            //     image.draw_circle(centerX, centerY, cellSize / 4, color);
            //     std::cout << "printed robot";
            // }
        }
    }

    // for (Robot::Coordinates coord : robot.path)
    // {
    //     int x = coord.x;
    //     int y = coord.y;
    //     int centerX = x * cellSize + cellSize / 2;
    //     int centerY = y * cellSize + cellSize / 2;
    //     unsigned char color[3];
    //     color[0] = 255;
    //     color[1] = 192;
    //     color[2] = 203; // Robot: Blue
    //     image.draw_circle(centerX, centerY, cellSize / 4, color);
    // }

    // for (const auto &entry : shortestPath)
    // {
    //     int x = entry.first;
    //     int y = entry.second;
    //     int centerX = x * cellSize + cellSize / 2;
    //     int centerY = y * cellSize + cellSize / 2;
    //     unsigned char color[3];
    //     color[0] = 255;
    //     color[1] = 0;
    //     color[2] = 0; // Shortest Path: Red
    //     image.draw_circle(centerX, centerY, cellSize / 4, color);
    // }

    std::string path = "SimPhotos/" + filename;
    image.save(path.c_str());
}
void Robot::recharge()
{
    batteryPercentage = 100;
    resetSimulationHoursPassed();
}

void Robot::resetSimulationHoursPassed()
{
    simulationHoursPassed = 0;
}

void Robot::incrementSimulationHoursPassed()
{
    ++simulationHoursPassed;
    if (needsRecharge())
    {
        recharge();
    }
}

void Forest::generateObstacles()
{
    // Modified probability distribution for obstacles
    std::vector<std::pair<ObstacleType, double>> obstacleTypes = {
        {ObstacleType::Tree, 0.6},
        {ObstacleType::Rock, 0.2},
        {ObstacleType::Water, 0.05},
        {ObstacleType::Marshy, 0.15}};

    std::discrete_distribution<> dist({0.6, 0.2, 0.05, 0.15});

    for (int i = 0; i < numObstacles; ++i)
    {
        int x = std::rand() % width;
        int y = std::rand() % height;
        ObstacleType obstacleType = obstacleTypes[dist(std::mt19937{std::random_device{}()})].first;
        if (getObstacle(x, y) == ObstacleType::None)
        {
            obstacles[std::make_pair(x, y)] = obstacleType;
            obstacleCounts[obstacleType]++;
        }
        else
        {
            --i;
        }
    }
        std::cout << "Check Obst" << std::endl;
    generateWater();
    generateMarshy();
}

void Forest::generateWater()
{
    int waterPercentage = 45; // Adjust the percentage of water coverage
    int iterations = 3;       // Number of iterations for the cellular automata algorithm

    // Initialize water randomly
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if ((std::rand() % 100) < waterPercentage && getObstacle(x, y) == ObstacleType::None)
            {
                obstacles[std::make_pair(x, y)] = ObstacleType::Water;
            }
        }
    }

    // Apply the cellular automata algorithm
    for (int i = 0; i < iterations; ++i)
    {
        std::map<std::pair<int, int>, ObstacleType> newObstacles = obstacles;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int waterNeighbors = countWaterNeighbors(x, y);
                if (waterNeighbors >= 5)
                {
                    newObstacles[std::make_pair(x, y)] = ObstacleType::Water;
                }
                else
                {
                    if (getObstacle(x, y) == ObstacleType::Water)
                    {
                        newObstacles[std::make_pair(x, y)] = ObstacleType::None;
                    }
                }
            }
        }
        obstacles = newObstacles;
    }
}

int Forest::countWaterNeighbors(int x, int y)
{
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width && ny >= 0 && ny < height && getObstacle(nx, ny) == ObstacleType::Water)
            {
                count++;
            }
        }
    }
    return count;
}

void Forest::generateMarshy()
{
    int marshyPercentage = 45; // Adjust the percentage of water coverage
    int iterations = 3;        // Number of iterations for the cellular automata algorithm

    // Initialize marsh randomly
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if ((std::rand() % 100) < marshyPercentage && getObstacle(x, y) == ObstacleType::None)
            {
                obstacles[std::make_pair(x, y)] = ObstacleType::Marshy;
            }
        }
    }

    // Apply the cellular automata algorithm
    for (int i = 0; i < iterations; ++i)
    {
        std::map<std::pair<int, int>, ObstacleType> newObstacles = obstacles;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int marshyNeighbors = countMarshyNeighbors(x, y);
                if (marshyNeighbors >= 5)
                {
                    newObstacles[std::make_pair(x, y)] = ObstacleType::Marshy;
                }
                else
                {
                    if (getObstacle(x, y) == ObstacleType::Marshy)
                    {
                        newObstacles[std::make_pair(x, y)] = ObstacleType::None;
                    }
                }
            }
        }
        obstacles = newObstacles;
    }
}

int Forest::countMarshyNeighbors(int x, int y)
{
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width && ny >= 0 && ny < height && getObstacle(nx, ny) == ObstacleType::Marshy)
            {
                count++;
            }
        }
    }
    return count;
}

const int stateSize = 40000; // You can adjust this value based on the size of the forest
const int actionSize = 3;    // Forward, TurnLeft, TurnRight
const int episodes = 400;
const int evaluationEpisodes = 100;
double totalSteps = 0.0;
double totalStepsSquared = 0.0;
double varianceSteps = 0.0;
int successfulEpisodes = 0;

int main()
{

    const int maxStepsPerEpisode = 90000000;
    const int stepsToStartCalculatingVariance = episodes - evaluationEpisodes;
    SARSA sarsa(stateSize, actionSize, 0.1, 0.99, 0.5); // alpha=0.1, gamma=0.99, epsilon=0.1

    std::ofstream stepsFile("steps_per_episode.txt");

    Forest forest(100, 100, 2000);
    std::cout << "RUNNING" << std::endl;
     // Generate a new forest for each episode
    for (int e = 0; e < episodes; ++e)
    {
        Robot robot(0, 0, forest);

        robot.reset(); // Reset robot position and orientation
        robot.resetSimulationHoursPassed();
        int state = robot.getState();
        int action = sarsa.chooseAction(state);

        int stepCount = 0;

        int steps = 0;
        while (!robot.goalReached() && steps < maxStepsPerEpisode)
        {
            robot.incrementSimulationHoursPassed(); // Increment hours passed counter at each step

            if (robot.needsRecharge())
            { // Simulate the recharge process
                std::this_thread::sleep_for(std::chrono::hours(10));
                robot.resetSimulationHoursPassed();
            }

            robot.performAction(static_cast<Robot::Action>(action));
            int nextState = robot.getState();
            int nextAction = sarsa.chooseAction(nextState);

            double reward = robot.calculateReward();

            Forest::ObstacleType obstacle = forest.getObstacle(robot.getX(), robot.getY());
            sarsa.update(state, action, nextState, nextAction, reward);

            state = nextState;
            action = nextAction;
            ++steps;
        }
        if (robot.goalReached())
        {
            ++successfulEpisodes;
            double efficiencyInt = 100 - 100 * (steps - forest.shortestPathLength) / (forest.shortestPathLength * 4);
            std::cout << "Episode " << e + 1 << ": " << steps << " steps"
                      << " Efficiency: " << efficiencyInt << " - Shortest Path: " << forest.shortestPathLength << std::endl;
            forest.printForest(robot, "forest_state_" + std::to_string(e + 1) + ".png");
            stepsFile << "Episode " << e + 1 << ": " << steps << " steps" << std::endl;
            // break;
        }
        else
        {
            std::cout << "Episode " << e + 1 << " Failed - Efficiency 0" << std::endl;
        }
        if (e % evaluationEpisodes == 0 && e != 0)
        {
            sarsa.updateEpsilon(0.99); // Update epsilon with a decay rate of 0.99
        }
    }
    stepsFile.close();
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