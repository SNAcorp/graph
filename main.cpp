#include "Graph.h"
#include "AntColony.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

/**
 * @brief Генерирует случайный взвешенный направленный граф.
 *
 * @param numVertices Количество вершин.
 * @param maxEdgesPerVertex Максимальное количество исходящих ребер для каждой вершины.
 * @param maxWeight Максимальный вес ребра.
 * @return Graph Сгенерированный граф.
 */
Graph generateRandomDirectedGraph(int numVertices, int maxEdgesPerVertex, double maxWeight) {
    Graph graph(true, true); // Направленный, взвешенный граф

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> edgeDist(1, maxEdgesPerVertex);
    std::uniform_int_distribution<> vertexDist(0, numVertices - 1);
    std::uniform_real_distribution<> weightDist(1.0, maxWeight);

    for (int u = 0; u < numVertices; ++u) {
        int numEdges = edgeDist(gen);
        for (int i = 0; i < numEdges; ++i) {
            int v = vertexDist(gen);
            if (u != v) { // Избегаем петель
                double weight = weightDist(gen);
                graph.addEdge(u, v, weight);
            }
        }
    }

    return graph;
}

/**
 * @brief Главная функция программы.
 *
 * Создает граф, запускает муравьиный алгоритм и алгоритм Дейкстры, сохраняет результаты.
 */
int main() {
    // Создаем граф
    Graph graph(true, true); // Ненаправленный, взвешенный

    // Добавляем ребра в граф
    graph.addEdge(0, 1, 3.0);
    graph.addEdge(1, 0, 3.0);

    graph.addEdge(1, 2, 8.0);
    graph.addEdge(2, 1, 3.0);

    graph.addEdge(2, 3, 1.0);
    graph.addEdge(3, 2, 8.0);

    graph.addEdge(3, 4, 1.0);
    graph.addEdge(4, 3, 3.0);

    graph.addEdge(4, 0, 3.0);
    graph.addEdge(0, 4, 1.0);

    graph.addEdge(5, 0, 3.0);

    graph.addEdge(5, 1, 3.0);

    graph.addEdge(1, 5, 3.0);
    graph.addEdge(5, 1, 3.0);

    graph.addEdge(2, 5, 1.0);
    graph.addEdge(5,2, 3.0);

    graph.addEdge(5, 3, 5.0);

    graph.addEdge(5, 4, 4.0);


    // Параметры муравьиного алгоритма
    int numAnts = 10;
    int maxIterations = 100;
    double alpha = 1.0; // Влияние феромонов
    double beta = 5.0;  // Влияние эвристической информации
    double evaporationRate = 0.5; // Коэффициент испарения феромонов
    int numVertices = 10000;
    int maxEdgesPerVertex = 100; // Максимальное количество исходящих ребер для каждой вершины
    double maxWeight = 1000.0;

    Graph randomGraph = generateRandomDirectedGraph(numVertices, maxEdgesPerVertex, maxWeight);
    // Запускаем муравьиный алгоритм
    AntColony antColony(randomGraph, numAnts, maxIterations, alpha, beta, evaporationRate);
    antColony.run();

    // Получаем лучший путь и его длину
    const std::vector<int>& bestPath = antColony.getBestPath();
    double bestLength = antColony.getBestPathLength();

    // Выводим результаты
    std::cout << "Длина лучшего пути (муравьиный алгоритм): " << bestLength << std::endl;
    std::cout << "Лучший путь: ";
    for (int node : bestPath) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    // Сохраняем данные для построения графика в файл
    std::ofstream outFile("path_lengths.csv");
    outFile << "Iteration,PathLength\n";
    const std::vector<double>& bestPathLengths = antColony.getBestPathLengths();
    for (size_t i = 0; i < bestPathLengths.size(); ++i) {
        outFile << i + 1 << "," << bestPathLengths[i] << "\n";
    }
    outFile.close();

    std::cout << "Данные для графика сохранены в файл 'path_lengths.csv'" << std::endl;

    // Запускаем алгоритм Дейкстры от вершины 0
    std::map<int, double> distances = graph.dijkstra(0);

    // Выводим результаты алгоритма Дейкстры
    std::cout << "\nРезультаты алгоритма Дейкстры (от вершины 0):" << std::endl;
    for (const auto& [vertex, distance] : distances) {
        std::cout << "Вершина " << vertex << ", Расстояние: " << distance << std::endl;
    }

    // Создаем второй граф для проверки изоморфизма
    Graph graph2(true, true);
    graph2.addEdge(0, 1, 3.0);
    graph2.addEdge(1, 0, 3.0);

    graph2.addEdge(1, 2, 8.0);
    graph2.addEdge(2, 1, 3.0);

    graph2.addEdge(2, 3, 1.0);
    graph2.addEdge(3, 2, 8.0);

    graph2.addEdge(3, 4, 1.0);
    graph2.addEdge(4, 3, 3.0);

    graph2.addEdge(4, 0, 3.0);
    graph2.addEdge(0, 4, 1.0);

    graph2.addEdge(5, 0, 3.0);

    graph2.addEdge(5, 1, 3.0);

    graph2.addEdge(1, 5, 3.0);
    graph2.addEdge(5, 1, 3.0);

    graph2.addEdge(2, 5, 1.0);
    graph2.addEdge(5,2, 3.0);

    graph2.addEdge(5, 3, 5.0);

    graph2.addEdge(5, 4, 4.0);

    // Проверяем изоморфизм
    if (graph.isIsomorphicTo(graph2)) {
        std::cout << "\nГрафы изоморфны." << std::endl;
    } else {
        std::cout << "\nГрафы не изоморфны." << std::endl;
    }

    // Генерируем случайный направленный граф с 1000 вершинами
//    int numVertices = 10000;
//    int maxEdgesPerVertex = 100; // Максимальное количество исходящих ребер для каждой вершины
//    double maxWeight = 1000.0;

    Graph randomGraph1 = generateRandomDirectedGraph(numVertices, maxEdgesPerVertex, maxWeight);

    Graph randomGraph2 = generateRandomDirectedGraph(numVertices, maxEdgesPerVertex, maxWeight);

    // Измеряем время проверки изоморфизма графа с самим собой
    auto startTime = std::chrono::high_resolution_clock::now();

    bool isIsomorphic = randomGraph.isIsomorphicTo(randomGraph2);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "\nПроверка изоморфизма случайного графа с самим собой: " << (isIsomorphic ? "Изоморфны" : "Не изоморфны") << std::endl;
    std::cout << "Время проверки изоморфизма: " << duration.count() << " миллисекунд" << std::endl;


    return 0;
}
