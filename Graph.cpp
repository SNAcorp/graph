#include "Graph.h"
#include <algorithm>
#include <queue>

/**
 * @brief Конструктор графа.
 *
 * @param directed Указывает, является ли граф направленным.
 * @param weighted Указывает, является ли граф взвешенным.
 */
Graph::Graph(bool directed, bool weighted)
        : directed(directed), weighted(weighted) {}

/**
 * @brief Добавляет ребро между двумя вершинами.
 *
 * @param u Первая вершина.
 * @param v Вторая вершина.
 * @param weight Вес ребра.
 */
void Graph::addEdge(int u, int v, double weight) {
    adjList[u].push_back(v);
    vertices.insert(u);
    vertices.insert(v);

    if (weighted) {
        edgeWeights[{u, v}] = weight;
    }

    if (!directed) {
        adjList[v].push_back(u);
        if (weighted) {
            edgeWeights[{v, u}] = weight;
        }
    }
}

/**
 * @brief Возвращает количество вершин в графе.
 *
 * @return int Количество вершин.
 */
int Graph::numVertices() const {
    return vertices.size();
}

/**
 * @brief Возвращает соседей заданной вершины.
 *
 * @param u Вершина.
 * @return const std::vector<int>& Список соседей.
 */
const std::vector<int>& Graph::getNeighbors(int u) const {
    static std::vector<int> empty;
    auto it = adjList.find(u);
    if (it != adjList.end())
        return it->second;
    else
        return empty;
}

/**
 * @brief Возвращает вес ребра между двумя вершинами.
 *
 * @param u Первая вершина.
 * @param v Вторая вершина.
 * @return double Вес ребра.
 */
double Graph::getWeight(int u, int v) const {
    if (weighted) {
        auto it = edgeWeights.find({u, v});
        if (it != edgeWeights.end())
            return it->second;
        else
            return std::numeric_limits<double>::infinity(); // Если ребра нет, возвращаем бесконечность
    } else {
        return 1.0; // Вес по умолчанию
    }
}

/**
 * @brief Возвращает множество вершин графа.
 *
 * @return const std::set<int>& Множество вершин.
 */
const std::set<int>& Graph::getVertices() const {
    return vertices;
}

/**
 * @brief Проверяет изоморфизм с другим графом.
 *
 * @param other Другой граф.
 * @return true Если графы изоморфны.
 * @return false Если графы не изоморфны.
 */
bool Graph::isIsomorphicTo(const Graph& other) const {
    if (vertices.size() != other.vertices.size())
        return false;

    std::vector<int> degrees1, degrees2;

    for (int v : vertices)
        degrees1.push_back(adjList.at(v).size());

    for (int v : other.vertices)
        degrees2.push_back(other.adjList.at(v).size());

    std::sort(degrees1.begin(), degrees1.end());
    std::sort(degrees2.begin(), degrees2.end());

    return degrees1 == degrees2;
}

/**
 * @brief Реализует алгоритм Дейкстры для поиска кратчайшего пути от заданной вершины.
 *
 * @param start Начальная вершина.
 * @return std::map<int, double> Расстояния до всех остальных вершин.
 */
std::map<int, double> Graph::dijkstra(int start) const {
    // Инициализация расстояний до бесконечности
    std::map<int, double> distances;
    for (int v : vertices) {
        distances[v] = std::numeric_limits<double>::infinity();
    }
    distances[start] = 0.0;

    // Очередь с приоритетом для выбора вершины с минимальным расстоянием
    using pii = std::pair<double, int>;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [dist_u, u] = pq.top();
        pq.pop();

        if (dist_u > distances[u])
            continue;

        for (int v : getNeighbors(u)) {
            double weight = getWeight(u, v);
            if (distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
                pq.push({distances[v], v});
            }
        }
    }

    return distances;
}
