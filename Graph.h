#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <map>
#include <set>
#include <utility>
#include <limits>

/**
 * @brief Универсальный класс графа.
 *
 * Поддерживает направленные и ненаправленные, взвешенные и невзвешенные графы.
 */
class Graph {
public:
    /**
     * @brief Конструктор графа.
     *
     * @param directed Указывает, является ли граф направленным.
     * @param weighted Указывает, является ли граф взвешенным.
     */
    Graph(bool directed = false, bool weighted = false);

    /**
     * @brief Добавляет ребро между двумя вершинами.
     *
     * @param u Первая вершина.
     * @param v Вторая вершина.
     * @param weight Вес ребра (по умолчанию 1.0).
     */
    void addEdge(int u, int v, double weight = 1.0);

    /**
     * @brief Возвращает количество вершин в графе.
     *
     * @return int Количество вершин.
     */
    int numVertices() const;

    /**
     * @brief Возвращает соседей заданной вершины.
     *
     * @param u Вершина.
     * @return const std::vector<int>& Список соседей.
     */
    const std::vector<int>& getNeighbors(int u) const;

    /**
     * @brief Возвращает вес ребра между двумя вершинами.
     *
     * @param u Первая вершина.
     * @param v Вторая вершина.
     * @return double Вес ребра.
     */
    double getWeight(int u, int v) const;

    /**
     * @brief Возвращает множество вершин графа.
     *
     * @return const std::set<int>& Множество вершин.
     */
    const std::set<int>& getVertices() const;

    /**
     * @brief Проверяет изоморфизм с другим графом.
     *
     * @param other Другой граф.
     * @return true Если графы изоморфны.
     * @return false Если графы не изоморфны.
     */
    bool isIsomorphicTo(const Graph& other) const;

    /**
     * @brief Проверяет, является ли граф направленным.
     *
     * @return true Если граф направленный.
     * @return false Если граф ненаправленный.
     */
    bool isDirected() const { return directed; }

    /**
     * @brief Реализует алгоритм Дейкстры для поиска кратчайшего пути от заданной вершины.
     *
     * @param start Начальная вершина.
     * @return std::map<int, double> Расстояния до всех остальных вершин.
     */
    std::map<int, double> dijkstra(int start) const;

private:
    bool directed; ///< Флаг, указывающий на направленность графа.
    bool weighted; ///< Флаг, указывающий на взвешенность графа.
    std::map<int, std::vector<int>> adjList; ///< Список смежности.
    std::map<std::pair<int, int>, double> edgeWeights; ///< Вес ребер.
    std::set<int> vertices; ///< Множество вершин.
};

#endif // GRAPH_H
