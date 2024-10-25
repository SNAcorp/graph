#ifndef ANTCOLONY_H
#define ANTCOLONY_H

#include "Graph.h"
#include <vector>

/**
 * @brief Класс, реализующий муравьиный алгоритм для поиска оптимального пути в графе.
 */
class AntColony {
public:
    /**
     * @brief Конструктор муравьиной колонии.
     *
     * @param graph Граф, по которому будет выполняться алгоритм.
     * @param numAnts Количество муравьев.
     * @param maxIterations Максимальное количество итераций.
     * @param alpha Влияние феромонов.
     * @param beta Влияние эвристической информации.
     * @param evaporationRate Коэффициент испарения феромонов.
     */
    AntColony(const Graph& graph, int numAnts, int maxIterations, double alpha = 1.0, double beta = 5.0, double evaporationRate = 0.5);

    /**
     * @brief Запускает выполнение муравьиного алгоритма.
     */
    void run();

    /**
     * @brief Возвращает лучший найденный путь.
     *
     * @return const std::vector<int>& Лучший путь.
     */
    const std::vector<int>& getBestPath() const;

    /**
     * @brief Возвращает длину лучшего найденного пути.
     *
     * @return double Длина лучшего пути.
     */
    double getBestPathLength() const;

    /**
     * @brief Возвращает историю длин лучших путей на каждой итерации.
     *
     * @return const std::vector<double>& История длин лучших путей.
     */
    const std::vector<double>& getBestPathLengths() const;

private:
    const Graph& graph; ///< Граф для поиска пути.
    int numAnts; ///< Количество муравьев.
    int maxIterations; ///< Максимальное количество итераций.
    double alpha; ///< Влияние феромонов.
    double beta; ///< Влияние эвристической информации.
    double evaporationRate; ///< Коэффициент испарения феромонов.
    std::vector<std::vector<double>> pheromone; ///< Матрица феромонов.
    std::vector<int> bestPath; ///< Лучший найденный путь.
    double bestPathLength; ///< Длина лучшего найденного пути.
    std::vector<double> bestPathLengths; ///< История длин лучших путей.

    /**
     * @brief Инициализирует матрицу феромонов.
     */
    void initializePheromones();

    /**
     * @brief Строит решение (путь) для одного муравья.
     *
     * @return std::vector<int> Построенный путь.
     */
    std::vector<int> constructSolution();

    /**
     * @brief Обновляет феромоны на основе решений муравьев.
     *
     * @param solutions Решения муравьев.
     * @param lengths Длины путей.
     */
    void updatePheromones(const std::vector<std::vector<int>>& solutions, const std::vector<double>& lengths);

    /**
     * @brief Вычисляет длину пути.
     *
     * @param path Путь.
     * @return double Длина пути.
     */
    double calculatePathLength(const std::vector<int>& path);

    /**
     * @brief Выбирает следующую вершину для муравья.
     *
     * @param currentNode Текущая вершина.
     * @param visited Вектор посещенных вершин.
     * @return int Индекс следующей вершины.
     */
    int selectNextNode(int currentNode, const std::vector<bool>& visited);
};

#endif // ANTCOLONY_H
