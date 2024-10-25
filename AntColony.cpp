#include "AntColony.h"
#include <limits>
#include <random>
#include <algorithm>
#include <cmath>

/**
 * @brief Конструктор муравьиной колонии.
 *
 * @param graph Граф для поиска пути.
 * @param numAnts Количество муравьев.
 * @param maxIterations Максимальное количество итераций.
 * @param alpha Влияние феромонов.
 * @param beta Влияние эвристической информации.
 * @param evaporationRate Коэффициент испарения феромонов.
 */
AntColony::AntColony(const Graph& graph, int numAnts, int maxIterations, double alpha, double beta, double evaporationRate)
        : graph(graph), numAnts(numAnts), maxIterations(maxIterations), alpha(alpha), beta(beta), evaporationRate(evaporationRate), bestPathLength(std::numeric_limits<double>::infinity()) {
    initializePheromones();
}

/**
 * @brief Запускает выполнение муравьиного алгоритма.
 */
void AntColony::run() {
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        std::vector<std::vector<int>> solutions;
        std::vector<double> lengths;

        // Каждая муравей строит решение
        for (int ant = 0; ant < numAnts; ++ant) {
            std::vector<int> path = constructSolution();
            double length = calculatePathLength(path);
            solutions.push_back(path);
            lengths.push_back(length);

            // Обновление лучшего пути
            if (length < bestPathLength) {
                bestPathLength = length;
                bestPath = path;
            }
        }

        // Обновление феромонов
        updatePheromones(solutions, lengths);
        bestPathLengths.push_back(bestPathLength);
    }
}

/**
 * @brief Инициализирует матрицу феромонов.
 */
void AntColony::initializePheromones() {
    int n = graph.numVertices();
    pheromone.resize(n, std::vector<double>(n, 1.0));
}

/**
 * @brief Строит решение (путь) для одного муравья.
 *
 * @return std::vector<int> Построенный путь.
 */
std::vector<int> AntColony::constructSolution() {
    std::vector<int> path;
    std::vector<bool> visited(graph.numVertices(), false);
    std::vector<int> nodes(graph.getVertices().begin(), graph.getVertices().end());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, nodes.size() - 1);

    // Случайный выбор начальной вершины
    int currentNode = nodes[dist(gen)];
    path.push_back(currentNode);
    visited[currentNode] = true;

    // Построение пути
    while (path.size() < nodes.size()) {
        int nextNode = selectNextNode(currentNode, visited);
        if (nextNode == -1)
            break; // Нет доступных вершин

        path.push_back(nextNode);
        visited[nextNode] = true;
        currentNode = nextNode;
    }

    return path;
}

/**
 * @brief Выбирает следующую вершину для муравья.
 *
 * @param currentNode Текущая вершина.
 * @param visited Вектор посещенных вершин.
 * @return int Индекс следующей вершины.
 */
int AntColony::selectNextNode(int currentNode, const std::vector<bool>& visited) {
    const std::vector<int>& neighbors = graph.getNeighbors(currentNode);
    std::vector<double> probabilities;
    std::vector<int> candidateNodes;
    double sum = 0.0;

    // Рассчитываем вероятности перехода к каждой соседней не посещенной вершине
    for (int neighbor : neighbors) {
        if (!visited[neighbor]) {
            double tau = pow(pheromone[currentNode][neighbor], alpha);
            double eta = pow(1.0 / graph.getWeight(currentNode, neighbor), beta);
            double value = tau * eta;
            probabilities.push_back(value);
            sum += value;
            candidateNodes.push_back(neighbor);
        }
    }

    if (sum == 0.0)
        return -1; // Нет доступных соседей

    // Нормализуем вероятности
    for (double& probability : probabilities) {
        probability /= sum;
    }

    // Рулетка для выбора следующей вершины
    std::discrete_distribution<> dist(probabilities.begin(), probabilities.end());
    std::random_device rd;
    std::mt19937 gen(rd());

    int index = dist(gen);

    return candidateNodes[index];
}

/**
 * @brief Обновляет феромоны на основе решений муравьев.
 *
 * @param solutions Решения муравьев.
 * @param lengths Длины путей.
 */
void AntColony::updatePheromones(const std::vector<std::vector<int>>& solutions, const std::vector<double>& lengths) {
    // Испарение феромонов
    for (size_t i = 0; i < pheromone.size(); ++i)
        for (size_t j = 0; j < pheromone[i].size(); ++j)
            pheromone[i][j] *= (1.0 - evaporationRate);

    // Добавление новых феромонов на пройденных путях
    for (size_t k = 0; k < solutions.size(); ++k) {
        const std::vector<int>& path = solutions[k];
        double length = lengths[k];
        double delta = 1.0 / length;

        for (size_t i = 0; i < path.size() - 1; ++i) {
            int u = path[i];
            int v = path[i + 1];
            pheromone[u][v] += delta;
            if (!graph.isDirected())
                pheromone[v][u] += delta;
        }
    }
}

/**
 * @brief Вычисляет длину пути.
 *
 * @param path Путь.
 * @return double Длина пути.
 */
double AntColony::calculatePathLength(const std::vector<int>& path) {
    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int u = path[i];
        int v = path[i + 1];
        length += graph.getWeight(u, v);
    }
    return length;
}

/**
 * @brief Возвращает лучший найденный путь.
 *
 * @return const std::vector<int>& Лучший путь.
 */
const std::vector<int>& AntColony::getBestPath() const {
    return bestPath;
}

/**
 * @brief Возвращает длину лучшего найденного пути.
 *
 * @return double Длина лучшего пути.
 */
double AntColony::getBestPathLength() const {
    return bestPathLength;
}

/**
 * @brief Возвращает историю длин лучших путей на каждой итерации.
 *
 * @return const std::vector<double>& История длин лучших путей.
 */
const std::vector<double>& AntColony::getBestPathLengths() const {
    return bestPathLengths;
}
